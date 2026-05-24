"""Approach a detected subject and stop at a standoff distance.

A one-shot, closed-loop "go to that thing." Where `follow` maintains a
distance indefinitely (companionship), `approach` drives to a target
standoff and *completes* — the primitive behind "go investigate that,"
"drive up to the person," and eventually go-to-waypoint.

Reuses follow.py's proven proportional control (CLA-49 field-tuned):
distance error -> forward speed, bearing error -> turn, skid-steer mixed,
with the same LiDAR forward-gate, gimbal-orient precondition, and
id-hint / position-handover target lock. The one real addition is
termination: once within `standoff` (+ tolerance) of the target, stop and
complete.

Approach only drives *forward* — if it starts already inside the standoff,
it's arrived. (Unlike follow, it won't reverse to hold a set distance.)

Heading note: bearing control is in the OAK-D body frame (relative), so it
inherits the gyro-relative heading work transparently — no absolute bearing.
"""

import time

from ..intent_stack import Intent, TickResult, TickContext, register_intent

# Gains lifted from follow.py's CLA-49 field-tuned set. Approach can diverge
# these later (it may want crisper closure), but starting from the proven
# values keeps first-outing risk low.
KP_DISTANCE = 0.3        # forward speed per metre of distance error
KP_BEARING = 0.012       # angular gain per degree of bearing error (sign flipped in code)
MAX_LINEAR = 0.5
MAX_ANGULAR = 0.35
DEAD_ZONE_BEARING = 7.0  # degrees; wider than OAK centroid jitter (~4° at 1.5m)

ARRIVAL_TOLERANCE_M = 0.15   # within standoff + this = arrived
LOST_TIMEOUT_BEATS = 20      # ~2s at 10 Hz before giving up on a missing target
HANDOVER_RADIUS_M = 3.0      # position-handover radius when the track id flickers
ORIENT_THRESHOLD_DEG = 20.0  # gimbal pan beyond this -> face_gimbal first


@register_intent
class Approach(Intent):
    name = "approach"
    resumable = False  # spatial state changes when interrupted (as with drive_distance)

    def start(self, params: dict) -> None:
        tid = params.get("target_id")
        self.target_id: str | None = str(tid) if tid is not None else None
        self.target_index = int(params.get("target_index", 0))
        self.standoff = float(params.get("standoff", 1.0))  # metres to stop short of the target
        self.max_linear = float(params.get("max_speed", MAX_LINEAR))
        self.timeout = min(60.0, max(2.0, float(params.get("timeout", 30.0))))
        self.started_at = time.time()
        self.last_known_pos: dict | None = None
        self.lost_beats = 0
        self.current_distance: float | None = None
        self.current_bearing: float | None = None
        self._resolved = False
        self._orient_checked = False

    def _detections(self, state):
        spatial = state.get("spatial_detections") or {}
        return spatial.get("detections") or []

    def _distance_3d(self, a, b):
        dx = a["x"] - b["x"]
        dz = a["z"] - b["z"]
        return (dx * dx + dz * dz) ** 0.5

    def tick(self, ctx: TickContext) -> TickResult:
        state = ctx.get_state() if ctx.get_state else {}
        detections = self._detections(state)

        if time.time() - self.started_at >= self.timeout:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(complete=True, status=f"approach: timeout after {self.timeout:.0f}s")

        # Orient precondition: the OAK-D is body-fixed, so if the gimbal is
        # off-centre beyond threshold, align the body first (face_gimbal).
        if not self._orient_checked:
            pan = state.get("pan_angle", 0) if state else 0
            if abs(pan) > ORIENT_THRESHOLD_DEG and ctx.push_intent:
                ctx.push_intent("face_gimbal", {})
                self._orient_checked = True
                return TickResult(complete=False, status=f"approach: aligning body (gimbal pan {pan:+.0f}°) first")
            self._orient_checked = True

        # Resolve target on first tick: target_id (semantic) preferred, index fallback.
        if not self._resolved:
            match = None
            if self.target_id is not None:
                match = next((d for d in detections if str(d.get("id")) == self.target_id), None)
                if match is None:
                    ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                    return TickResult(complete=True, status=f"approach: target_id {self.target_id} not in detections ({len(detections)} present)")
            elif self.target_index < len(detections):
                match = detections[self.target_index]
                self.target_id = str(match["id"])
            else:
                ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                return TickResult(complete=True, status=f"approach: target_index {self.target_index} out of range ({len(detections)} present)")
            self.last_known_pos = match["position_m"]
            self._resolved = True

        # Target lock: id hint first, position-handover fallback (track ids flicker).
        match = next((d for d in detections if str(d.get("id")) == self.target_id), None)
        if match is None and detections and self.last_known_pos is not None:
            nearest = min(detections, key=lambda d: self._distance_3d(d["position_m"], self.last_known_pos))
            if self._distance_3d(nearest["position_m"], self.last_known_pos) <= HANDOVER_RADIUS_M:
                match = nearest
                self.target_id = str(nearest["id"])

        if match is None:
            self.lost_beats += 1
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            if self.lost_beats > LOST_TIMEOUT_BEATS:
                return TickResult(complete=True, status=f"approach: lost target (id={self.target_id})")
            return TickResult(complete=False, status=f"approach: target missing, holding ({self.lost_beats}/{LOST_TIMEOUT_BEATS})")

        self.lost_beats = 0
        self.last_known_pos = match["position_m"]
        self.current_distance = float(match["distance_m"])
        self.current_bearing = float(match["bearing_deg"])

        # Arrived? Within standoff (+ tolerance) -> stop and complete.
        if self.current_distance <= self.standoff + ARRIVAL_TOLERANCE_M:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(complete=True, status=f"approach: arrived at {self.current_distance:.2f}m (standoff {self.standoff:.2f}m), id={self.target_id}")

        # Proportional control, forward only (approach never reverses).
        distance_error = self.current_distance - self.standoff  # > 0 here
        linear = max(0.0, min(self.max_linear, distance_error * KP_DISTANCE))

        bearing_error = self.current_bearing  # + = target to the right
        if abs(bearing_error) < DEAD_ZONE_BEARING:
            angular = 0.0
        else:
            # Sign flip: target right (+) -> turn CW (angular -).
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, -bearing_error * KP_BEARING))

        # LiDAR forward-gate: never drive into a danger obstacle; turning stays
        # allowed so we keep facing the target. Backstop for heartbeat latency.
        lidar_gated = False
        if (state.get("lidar_status") or {}).get("status") == "danger" and linear > 0:
            linear = 0.0
            lidar_gated = True

        left = max(-1.0, min(1.0, linear - angular))
        right = max(-1.0, min(1.0, linear + angular))
        ctx.send_command(f'base -c {{"T":1,"L":{left:.2f},"R":{right:.2f}}}')

        gate = " [LIDAR GATED]" if lidar_gated else ""
        return TickResult(complete=False, status=(
            f"approach id={self.target_id} @ {self.current_distance:.2f}m -> standoff {self.standoff:.2f}m "
            f"bearing={self.current_bearing:+.1f}° L={left:.2f} R={right:.2f}{gate}"))

    def status(self) -> str:
        if not self._resolved:
            return f"starting approach (target={self.target_id or self.target_index}, standoff={self.standoff}m)"
        if self.current_distance is None:
            return f"approach id={self.target_id}, acquiring"
        return f"approach id={self.target_id} @ {self.current_distance:.2f}m -> {self.standoff:.2f}m standoff"

    def cleanup(self) -> None:
        pass
