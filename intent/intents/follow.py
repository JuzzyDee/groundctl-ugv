"""Follow a person in view using OAK-D metric spatial detections.

Subscribes to /state["spatial_detections"] (published by oakd_spatial
via HTTP POST to the bridge). The caller picks the target at push-time
by **target_id** (the track_id from the spatial_detections list) — this
is semantic: Haiku sees the list, picks the specific tracked entity, and
passes its id. Falling back to **target_index** is supported for
backwards compat but is positional-not-semantic and was the root cause
of early follow-the-wrong-subject bugs (see CLA-49): detections[0] might
be Chopper, a horse, or a YOLO-mislabelled elephant rather than the
intended person.

Target lock uses ID as a hint, position as the source of truth. On RVC2
the OAK-D's ZERO_TERM tracker flickers IDs through brief NN misses — ID
matching alone would lose the target constantly. Instead, if the pushed
ID disappears from detections, we look for the detection closest to the
last-known 3D position (within the handover radius) and re-lock onto
that. Works because people don't teleport between frames.

Stops (zero twist) with status="lost target" if no candidate detection
shows up for ~2s. Haiku can pop + re-push with a different id, or push
a different intent (sit, look_at, etc.) to re-engage.

Control model: skid-steer proportional controller with two independent
axes — distance error → linear, bearing error → angular — mixed via
`L = linear - angular`, `R = linear + angular`. Dead zones around zero
error prevent motor chatter. Gains (Kp_dist, Kp_bearing) and speed caps
are tuned conservatively for first-outing grass; raise once the safety
stack has earned trust.

LiDAR safety gate: before any forward motion, check state["lidar_status"].
If `danger`, zero linear velocity (still allow angular — turning in place
toward the target is safe). This is a backstop for when the heartbeat
prompt hasn't noticed the LiDAR tag yet — confirmed necessary from the
first duck-pond field session where follow kept driving into a solid
obstacle while Haiku only noticed the warning on the next beat.
"""

from ..intent_stack import Intent, TickResult, TickContext, register_intent


# Control gains.
#
# KP_BEARING and DEAD_ZONE_BEARING retuned 23 Apr after the first stationary-
# target test (CLA-49). At the original gains (0.02 / 3°), the rover closed
# distance cleanly from 2m to ~1.5m but then oscillated with growing amplitude
# — bearing swung +20 → +26 → +11 → lost. Root cause: ~200ms system latency
# (twist_mux + motor spindown + mechanical inertia) vs 100ms command cadence,
# combined with OAK-centroid jitter of ~4° at 1.5m range that the controller
# was treating as real target motion.
#
# Bumped 0.008 → 0.012 on 26 Apr after the TRT FP16 export + FrameBroadcaster
# refactor cut that ~200ms latency floor — at 0.008 the controller was now
# under-damped, lagging the target instead of oscillating past it. 0.012 is
# midway between the original (0.02) and the latency-limited retune (0.008),
# which the new latency budget can support without re-triggering the CLA-49
# oscillation. Still well below original; dead zone (7°) unchanged.
KP_DISTANCE = 0.3        # linear speed per metre of distance error
KP_BEARING = 0.012       # angular gain per degree of bearing error (+ sign flip in code)
MAX_LINEAR = 0.5         # cap on normalised forward speed (±). Bumped from 0.15 after
                         # first test felt unnecessarily slow — with KP_DISTANCE=0.3
                         # the P-controller wants 0.5+ above ~1.7m distance error.
                         # Previous 0.15 cap was clipping the natural approach profile.
                         # With MAX_LINEAR=0.5 and MAX_ANGULAR=0.35 saturated, mix
                         # becomes L=0.15 R=0.85 — a curve-while-moving gait rather
                         # than a pivot. LiDAR gate still zeros linear on danger.
MAX_ANGULAR = 0.35       # cap on normalised angular speed (±)
DEAD_ZONE_DISTANCE = 0.10  # metres; don't bother correcting below this
DEAD_ZONE_BEARING = 7.0    # degrees; wider than typical OAK centroid jitter (~4° at 1.5m)

# Target-lock parameters.
HANDOVER_RADIUS_M = 3.0  # max 3D distance between last-known and candidate for re-lock
                         # Bumped from 1.5 → 3.0 after CLA-49: tight radius caused
                         # spurious "target missing" when the rover rotated past a
                         # person who also moved reactively. 3m absorbs both sources
                         # of position drift without promiscuously re-locking.
LOST_TIMEOUT_BEATS = 20  # beats of no candidate before reporting "lost" (~2s at 10 Hz)

# Orientation prerequisite. The OAK-D is body-fixed forward, so follow
# only sees the target in spatial_detections when the body faces them.
# If Haiku identifies the target via the gimbal camera at a big pan offset,
# we need to align the body first. Threshold: 20° — inside that, the OAK's
# field of view still covers the target and bearing control handles the
# rest. Outside it, we auto-push face_gimbal as a precondition.
ORIENT_THRESHOLD_DEG = 20.0


@register_intent
class Follow(Intent):
    name = "follow"
    resumable = True  # we want to resume after digression (e.g. LookAt butterfly)

    def start(self, params: dict) -> None:
        # target_id is the semantic pick (track_id from spatial_detections).
        # target_index is the legacy positional pick, used only if target_id
        # is not provided. See CLA-49 — index-based picking confused Chopper
        # and YOLO-mislabelled wildlife for the intended person.
        tid = params.get("target_id")
        self.target_id: str | None = str(tid) if tid is not None else None
        self.target_index = int(params.get("target_index", 0))
        self.target_distance = float(params.get("distance", 1.0))  # metres
        self.max_linear = float(params.get("max_speed", MAX_LINEAR))
        self.last_known_pos: dict | None = None
        self.lost_beats = 0
        self.current_distance: float | None = None
        self.current_bearing: float | None = None
        self._resolved = False  # True once we've latched onto a target at least once
        # Orientation precondition — decided on first tick once we can read state.
        # face_gimbal is pushed as a nav-stack precondition so the body aligns
        # with what Haiku identified via the gimbal camera before follow tries
        # to drive. The ID-hint + position-handover in this intent's target
        # lock absorbs whatever change happens in the spatial_detections list
        # during the body turn.
        self._orient_checked = False

    def _detections(self, state):
        """Extract the spatial_detections list from state, robust to absence."""
        spatial = state.get("spatial_detections") or {}
        return spatial.get("detections") or []

    def _distance_3d(self, pos_a, pos_b):
        """Horizontal 3D distance (x, z plane — ignore vertical offset)."""
        dx = pos_a["x"] - pos_b["x"]
        dz = pos_a["z"] - pos_b["z"]
        return (dx * dx + dz * dz) ** 0.5

    def tick(self, ctx: TickContext) -> TickResult:
        state = ctx.get_state() if ctx.get_state else {}
        detections = self._detections(state)

        # First-tick orientation check: if the gimbal is off-centre beyond
        # the threshold, push face_gimbal first. That intent rotates the
        # body to match the gimbal pan and recentres the gimbal — after it
        # completes, follow resumes with OAK-D pointed where Haiku was
        # looking. Only runs once per start/resume cycle.
        if not self._orient_checked:
            pan = state.get("pan_angle", 0) if state else 0
            if abs(pan) > ORIENT_THRESHOLD_DEG and ctx.push_intent:
                ctx.push_intent("face_gimbal", {})
                self._orient_checked = True
                return TickResult(
                    complete=False,
                    status=f"follow: aligning body (gimbal pan {pan:+.0f}°) before engaging OAK",
                )
            # Below threshold or no push_intent capability — mark checked
            # and fall through to the normal resolve+control logic.
            self._orient_checked = True

        # First tick: resolve target to a specific detection.
        # Prefer target_id (semantic) if provided; fall back to target_index.
        if not self._resolved:
            match = None
            if self.target_id is not None:
                match = next(
                    (d for d in detections if str(d.get("id")) == self.target_id),
                    None,
                )
                if match is None:
                    ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                    return TickResult(
                        complete=True,
                        status=(
                            f"follow: target_id {self.target_id} not in current "
                            f"detections ({len(detections)} present)"
                        ),
                    )
            else:
                if self.target_index < len(detections):
                    match = detections[self.target_index]
                    self.target_id = str(match["id"])
                else:
                    ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                    return TickResult(
                        complete=True,
                        status=(
                            f"follow: target_index {self.target_index} out of "
                            f"range (have {len(detections)} detections)"
                        ),
                    )
            self.last_known_pos = match["position_m"]
            self._resolved = True

        # Target lock: id hint first, position fallback.
        match = next((d for d in detections if str(d.get("id")) == self.target_id), None)

        if match is None and detections and self.last_known_pos is not None:
            # ID lost — try position-based handover within a radius.
            nearest = min(detections, key=lambda d: self._distance_3d(d["position_m"], self.last_known_pos))
            if self._distance_3d(nearest["position_m"], self.last_known_pos) <= HANDOVER_RADIUS_M:
                match = nearest
                # Re-lock onto the new ID so future ticks find it directly.
                self.target_id = str(nearest["id"])

        if match is None:
            # No candidate. Stop, count beats, eventually report lost.
            self.lost_beats += 1
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            if self.lost_beats > LOST_TIMEOUT_BEATS:
                return TickResult(
                    complete=True,
                    status=f"follow: lost target (id={self.target_id}, "
                           f"last known @ {self.last_known_pos})",
                )
            return TickResult(
                complete=False,
                status=f"follow: target missing, holding ({self.lost_beats}/{LOST_TIMEOUT_BEATS})",
            )

        # We have a target this tick. Update state + compute control.
        self.lost_beats = 0
        self.last_known_pos = match["position_m"]
        self.current_distance = float(match["distance_m"])
        self.current_bearing = float(match["bearing_deg"])

        # Proportional control with dead zones + saturation.
        distance_error = self.current_distance - self.target_distance  # + = too far
        bearing_error = self.current_bearing                           # + = target to right

        if abs(distance_error) < DEAD_ZONE_DISTANCE:
            linear = 0.0
        else:
            linear = max(-self.max_linear, min(self.max_linear, distance_error * KP_DISTANCE))

        if abs(bearing_error) < DEAD_ZONE_BEARING:
            angular = 0.0
        else:
            # Sign flip: bearing +ve (target right) → angular -ve (turn CW/right).
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, -bearing_error * KP_BEARING))

        # LiDAR safety gate. If something's in the forward danger zone,
        # zero forward linear velocity — we can still rotate in place to
        # keep tracking the target, but we don't push into the obstacle.
        # This is a backstop for when the heartbeat loop hasn't seen the
        # LiDAR tag yet (the first duck-pond run failed for exactly this
        # reason — follow kept driving into a solid obstacle while Haiku
        # only noticed the warning on the next beat).
        lidar_gated = False
        lidar_status = (state.get("lidar_status") or {}).get("status")
        if lidar_status == "danger" and linear > 0:
            linear = 0.0
            lidar_gated = True

        # Skid-steer mix, per-wheel clamp.
        left = max(-1.0, min(1.0, linear - angular))
        right = max(-1.0, min(1.0, linear + angular))

        ctx.send_command(f'base -c {{"T":1,"L":{left:.2f},"R":{right:.2f}}}')

        gate_tag = " [LIDAR GATED]" if lidar_gated else ""
        return TickResult(
            complete=False,
            status=(
                f"follow id={self.target_id} @ {self.current_distance:.2f}m "
                f"bearing={self.current_bearing:+.1f}° "
                f"(err d={distance_error:+.2f}m b={bearing_error:+.1f}°) "
                f"L={left:.2f} R={right:.2f}{gate_tag}"
            ),
        )

    def status(self) -> str:
        if not self._resolved:
            if self.target_id is not None:
                return f"starting follow (target_id={self.target_id}, distance={self.target_distance}m)"
            return f"starting follow (target_index={self.target_index}, distance={self.target_distance}m)"
        if self.current_distance is None:
            return f"follow id={self.target_id}, acquiring"
        return (
            f"follow id={self.target_id} @ {self.current_distance:.2f}m "
            f"(target {self.target_distance}m)"
        )

    def suspend(self) -> dict:
        return {
            "target_id": self.target_id,
            "last_known_pos": self.last_known_pos,
            "target_distance": self.target_distance,
            "resolved": self._resolved,
        }

    def resume(self, saved: dict) -> None:
        self.target_id = saved.get("target_id")
        self.last_known_pos = saved.get("last_known_pos")
        self.target_distance = saved.get("target_distance", self.target_distance)
        self._resolved = saved.get("resolved", False)
        self.lost_beats = 0
        # Orient check does not re-run after resume — if the body was
        # aligned before face_gimbal ran, it's still aligned now. If the
        # intent above us turned the body, the position-handover will
        # re-lock onto the target from the new vantage.
        self._orient_checked = True

    def cleanup(self) -> None:
        # Intent base class doesn't give us a ctx at cleanup time, so we
        # can't send a zero-twist from here. The next non-follow intent's
        # first tick (or an explicit stop) is responsible for motor state.
        # In practice, Haiku always follows a popped follow with either
        # another motion intent or an emergency_stop / sit, so motors
        # don't coast. Worth revisiting if that assumption breaks.
        pass
