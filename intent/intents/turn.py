"""Turn — spin in place using differential drive."""

import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class Turn(Intent):
    name = "turn"
    resumable = False  # orientation unknown without IMU/GPS

    def start(self, params: dict) -> None:
        self.direction = params.get("direction", "right")
        self.speed = min(0.3, max(0.05, params.get("speed", 0.15)))
        self.duration = min(5.0, max(0.3, params.get("duration", 1.0)))
        self.started_at = time.time()

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at
        if elapsed >= self.duration:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(complete=True, status=f"Turned {self.direction} for {elapsed:.1f}s")

        if self.direction == "left":
            left, right = -self.speed, self.speed
        else:
            left, right = self.speed, -self.speed

        ctx.send_command(f'base -c {{"T":1,"L":{left},"R":{right}}}')
        return TickResult(status=f"Turning {self.direction} {elapsed:.1f}/{self.duration:.1f}s")

    def suspend(self) -> dict:
        return {
            "elapsed": time.time() - self.started_at,
            "direction": self.direction,
            "speed": self.speed,
            "duration": self.duration,
        }

    def resume(self, saved: dict) -> None:
        self.direction = saved["direction"]
        self.speed = saved["speed"]
        self.duration = saved["duration"]
        self.started_at = time.time() - saved["elapsed"]

    def status(self) -> str:
        elapsed = time.time() - self.started_at
        return f"turning {self.direction}, {elapsed:.1f}/{self.duration:.1f}s"

    def cleanup(self) -> None:
        pass
