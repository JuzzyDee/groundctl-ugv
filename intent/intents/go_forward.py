"""Go forward — drive straight at a given speed for a duration."""

import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class GoForward(Intent):
    name = "go_forward"
    resumable = False  # unsafe to resume without GPS — spatial context has changed

    def start(self, params: dict) -> None:
        self.speed = min(0.3, max(0.0, params.get("speed", 0.15)))
        self.duration = min(30.0, max(0.5, params.get("duration", 3.0)))
        self.started_at = time.time()

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at
        if elapsed >= self.duration:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(complete=True, status=f"Drove forward for {elapsed:.1f}s")

        ctx.send_command(f'base -c {{"T":1,"L":{self.speed},"R":{self.speed}}}')
        return TickResult(status=f"Driving forward {elapsed:.1f}/{self.duration:.1f}s")

    def suspend(self) -> dict:
        ctx_time = time.time() - self.started_at
        return {"elapsed": ctx_time, "speed": self.speed, "duration": self.duration}

    def resume(self, saved: dict) -> None:
        self.speed = saved["speed"]
        self.duration = saved["duration"]
        self.started_at = time.time() - saved["elapsed"]

    def status(self) -> str:
        elapsed = time.time() - self.started_at
        return f"forward at {self.speed:.2f}, {elapsed:.1f}/{self.duration:.1f}s"

    def cleanup(self) -> None:
        pass
