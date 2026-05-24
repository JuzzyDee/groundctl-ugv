"""Turn a relative number of degrees — closed-loop on the gyro-integrated heading.

state["heading"] is now gyro-relative (mag-free), so:
- Relative: relative_turn=N degrees from current heading (+ve=right/clockwise) — the working mode.
- Absolute: target_heading=N degrees from north — parked until the GNSS heading source
  returns. A gyro heading has an arbitrary zero, so absolute bearings are meaningless for now.

Combines with drive_distance to give Haiku the orient-to-camera primitive:
  1. Gimbal points at something at pan=-45°
  2. Push turn_to_heading(relative_turn=-45)
  3. Body rotates left to face it
  4. Push drive_distance(distance=2.0) to drive toward it
"""

import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent

TOLERANCE_DEG = 5.0
MAX_DURATION = 15.0


def normalise_angle(a):
    """Wrap angle to [-180, 180]."""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


@register_intent
class TurnToHeading(Intent):
    name = "turn_to_heading"
    resumable = False

    def start(self, params: dict) -> None:
        self.target_heading = params.get("target_heading")  # absolute compass deg
        self.relative_turn = params.get("relative_turn")  # degrees from current
        self.speed = min(0.3, max(0.05, params.get("speed", 0.15)))
        self.timeout = min(MAX_DURATION, max(2.0, params.get("timeout", 10.0)))
        self.started_at = time.time()
        self.start_heading = None
        self.resolved_target = None  # filled on first tick once we know current heading

    def _resolve_target(self, current_heading):
        """Convert relative_turn into an absolute target if needed."""
        if self.target_heading is not None:
            return self.target_heading % 360
        elif self.relative_turn is not None:
            return (current_heading + self.relative_turn) % 360
        else:
            return current_heading  # no-op

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at

        state = ctx.get_state() if ctx.get_state else None
        if not state or "heading" not in state:
            return TickResult(complete=True, status="No heading available — cannot turn_to_heading")

        current = state["heading"]

        if self.start_heading is None:
            self.start_heading = current
            self.resolved_target = self._resolve_target(current)

        # Compute shortest signed angular error
        error = normalise_angle(self.resolved_target - current)

        # Timeout
        if elapsed >= self.timeout:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(
                complete=True,
                status=f"Timeout — at {current:.0f}°, target {self.resolved_target:.0f}° (off by {error:+.0f}°)"
            )

        # Within tolerance?
        if abs(error) <= TOLERANCE_DEG:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(
                complete=True,
                status=f"Reached {current:.0f}° (target {self.resolved_target:.0f}°, took {elapsed:.1f}s)"
            )

        # Skid steer: positive error = turn right (clockwise), negative = left
        if error > 0:
            left, right = self.speed, -self.speed
        else:
            left, right = -self.speed, self.speed

        ctx.send_command(f'base -c {{"T":1,"L":{left},"R":{right}}}')
        return TickResult(status=f"Turning — at {current:.0f}°, target {self.resolved_target:.0f}° ({error:+.0f}° to go)")

    def status(self) -> str:
        if self.resolved_target is None:
            base = "preparing turn"
        else:
            base = f"turning to {self.resolved_target:.0f}°"
        return base

    def cleanup(self) -> None:
        pass
