"""Drive a specific distance forward or backward, with heading correction.

Closed-loop using /odom feedback for distance and /imu/mag for heading.
The rover drives at the requested speed until it has covered the requested
distance, with steering corrections to hold the initial heading. Stops at
target distance or timeout.

Heading correction prevents drift on uneven ground — when one wheel grips
better than the other, the rover would naturally turn off course. This
intent measures the heading error each tick and corrects via differential
wheel speeds.
"""

import math
import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent

HEADING_CORRECTION_GAIN = 0.005  # how aggressively to correct per degree of error
MAX_CORRECTION = 0.1  # cap correction so we never reverse a wheel for small errors

# A single-tick |heading - start_heading| exceeding this is almost certainly
# a corrupted magnetometer sample, not real rotation. The 2026-05-12 SLTF walk
# logged 100/123 beats with Δheading>20° and a clean pattern of ~180° atomic
# flips (89°↔269°, 92°↔276°) — the X+Y magnetometer-sign-flip signature of
# task #58 (ESP32 IMU JSON corruption). Without this gate, drive_distance
# saturates its P-controller against the phantom 180° error and arcs the
# rover into a U-turn (observed during the walk). Skip correction this tick
# instead. Root-cause fix is on the source side; this just stops the symptom.
HEADING_OUTLIER_DEG = 30.0


def normalise_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


@register_intent
class DriveDistance(Intent):
    name = "drive_distance"
    resumable = False  # spatial state changes when interrupted

    def start(self, params: dict) -> None:
        self.target_distance = params.get("distance", 1.0)  # metres, can be negative for reverse
        self.speed = min(0.3, max(0.05, params.get("speed", 0.15)))
        self.timeout = min(60.0, max(2.0, params.get("timeout", 30.0)))
        self.hold_heading = params.get("hold_heading", True)  # disable for free turns
        self.started_at = time.time()
        self.start_position = None  # captured on first tick
        self.start_heading = None  # captured on first tick
        self.distance_covered = 0.0
        self.completed_reason = None

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at

        state = ctx.get_state() if ctx.get_state else None
        if not state or "position" not in state:
            return TickResult(
                complete=True,
                status="No odometry available — cannot drive_distance"
            )

        current_pos = state["position"]
        current_heading = state.get("heading", None)

        # Capture starting state on first tick
        if self.start_position is None:
            self.start_position = {"x": current_pos["x"], "y": current_pos["y"]}
            if current_heading is not None:
                self.start_heading = current_heading

        # Distance from start
        dx = current_pos["x"] - self.start_position["x"]
        dy = current_pos["y"] - self.start_position["y"]
        self.distance_covered = math.sqrt(dx*dx + dy*dy)

        target_abs = abs(self.target_distance)
        direction = 1 if self.target_distance >= 0 else -1
        signed_speed = self.speed * direction

        # Timeout check
        if elapsed >= self.timeout:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            self.completed_reason = "timeout"
            return TickResult(
                complete=True,
                status=f"Timeout after {elapsed:.1f}s, covered {self.distance_covered:.2f}m of {target_abs:.2f}m"
            )

        # Distance reached?
        if self.distance_covered >= target_abs:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            self.completed_reason = "reached"
            return TickResult(
                complete=True,
                status=f"Reached {self.distance_covered:.2f}m in {elapsed:.1f}s"
            )

        # Compute heading correction (skid steer)
        correction = 0.0
        heading_error_str = ""
        if self.hold_heading and self.start_heading is not None and current_heading is not None:
            error = normalise_angle(self.start_heading - current_heading)
            if abs(error) > HEADING_OUTLIER_DEG:
                # Phantom flip — drive straight this tick, don't saturate.
                heading_error_str = f" (heading err {error:+.1f}° REJECTED)"
            else:
                correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, error * HEADING_CORRECTION_GAIN))
                heading_error_str = f" (heading err {error:+.1f}°)"

        # Apply correction: if drifted left (positive error), boost right wheel / slow left
        # Reversed when going backward
        left = signed_speed - correction * direction
        right = signed_speed + correction * direction

        cmd = f'base -c {{"T":1,"L":{left:.3f},"R":{right:.3f}}}'
        ctx.send_command(cmd)
        remaining = target_abs - self.distance_covered
        return TickResult(
            status=f"Driving — {self.distance_covered:.2f}/{target_abs:.2f}m ({remaining:.2f}m to go){heading_error_str}"
        )

    def status(self) -> str:
        if self.start_position is None:
            return f"starting drive to {self.target_distance:+.2f}m"
        return f"drove {self.distance_covered:.2f}/{abs(self.target_distance):.2f}m"

    def cleanup(self) -> None:
        pass
