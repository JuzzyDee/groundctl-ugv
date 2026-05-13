"""Face the direction the gimbal is currently pointing.

The natural pattern: look around with the gimbal, find something interesting,
then face your body that way. This intent does the whole sequence:
  1. Read current gimbal pan from state
  2. Turn body by that amount (closed-loop on magnetometer)
  3. Reset gimbal to pan=0 so it now points forward through the body's new heading

After completion, gimbal is centred and body is facing what the gimbal saw.
Combine with drive_distance to go toward what was looked at.
"""

import time
import math
from ..intent_stack import Intent, TickResult, TickContext, register_intent

TOLERANCE_DEG = 5.0
TURN_SPEED = 0.15
MAX_DURATION = 15.0


def normalise_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


@register_intent
class FaceGimbal(Intent):
    name = "face_gimbal"
    resumable = False

    def start(self, params: dict) -> None:
        self.timeout = min(MAX_DURATION, max(2.0, params.get("timeout", 10.0)))
        self.started_at = time.time()
        self.start_heading = None
        self.target_heading = None
        self.gimbal_pan_to_apply = None
        self.phase = "init"  # init -> turning -> resetting_gimbal -> done

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at

        state = ctx.get_state() if ctx.get_state else None
        if not state:
            return TickResult(complete=True, status="No state available")

        if self.phase == "init":
            current_pan = state.get("pan_angle", 0)
            current_heading = state.get("heading", 0)

            # If gimbal is already centred, nothing to do
            if abs(current_pan) < 2.0:
                return TickResult(complete=True, status="Gimbal already centred — nothing to face")

            # Body needs to rotate by current_pan degrees (positive pan = right of body)
            self.gimbal_pan_to_apply = current_pan
            self.start_heading = current_heading
            self.target_heading = (current_heading + current_pan) % 360
            self.phase = "turning"
            return TickResult(status=f"Will turn body {current_pan:+.0f}° to match gimbal")

        if self.phase == "turning":
            if elapsed >= self.timeout:
                ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                return TickResult(complete=True, status=f"Timeout while turning (target {self.target_heading:.0f}°)")

            current_heading = state.get("heading", 0)
            error = normalise_angle(self.target_heading - current_heading)

            if abs(error) <= TOLERANCE_DEG:
                ctx.send_command('base -c {"T":1,"L":0,"R":0}')
                # Now reset gimbal to forward
                ctx.send_command('base -c {"T":133,"X":0,"Y":0,"SPD":60,"ACC":0.4}')
                self.phase = "resetting_gimbal"
                return TickResult(status=f"Body aligned at {current_heading:.0f}°, centring gimbal")

            # Skid steer turn
            if error > 0:
                left, right = TURN_SPEED, -TURN_SPEED
            else:
                left, right = -TURN_SPEED, TURN_SPEED
            ctx.send_command(f'base -c {{"T":1,"L":{left},"R":{right}}}')
            return TickResult(status=f"Turning — at {current_heading:.0f}°, target {self.target_heading:.0f}° ({error:+.0f}° to go)")

        if self.phase == "resetting_gimbal":
            current_pan = state.get("pan_angle", 0)
            if abs(current_pan) < 2.0:
                return TickResult(complete=True, status=f"Done — body now faces what gimbal saw")
            # Re-send the centre command in case the gimbal didn't move yet
            ctx.send_command('base -c {"T":133,"X":0,"Y":0,"SPD":60,"ACC":0.4}')
            return TickResult(status=f"Waiting for gimbal to centre (currently {current_pan:.0f}°)")

        return TickResult(complete=True, status="Unknown phase")

    def status(self) -> str:
        if self.phase == "init":
            return "preparing to face gimbal direction"
        elif self.phase == "turning":
            return f"turning body to match gimbal ({self.gimbal_pan_to_apply:+.0f}°)"
        elif self.phase == "resetting_gimbal":
            return "centring gimbal"
        return "done"

    def cleanup(self) -> None:
        pass
