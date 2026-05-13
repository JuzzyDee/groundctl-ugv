"""Sit — stay in place and observe for a duration. The rover equivalent of sitting on a bench."""

import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class Sit(Intent):
    name = "sit"

    def start(self, params: dict) -> None:
        self.duration = min(300.0, max(5.0, params.get("duration", 60.0)))
        self.reason = params.get("reason", "observing")
        self.started_at = time.time()

    def tick(self, ctx: TickContext) -> TickResult:
        elapsed = time.time() - self.started_at
        if elapsed >= self.duration:
            return TickResult(complete=True, status=f"Sat for {elapsed:.0f}s ({self.reason})")
        remaining = self.duration - elapsed
        return TickResult(status=f"{self.reason}, {remaining:.0f}s remaining")

    def suspend(self) -> dict:
        return {
            "elapsed": time.time() - self.started_at,
            "duration": self.duration,
            "reason": self.reason,
        }

    def resume(self, saved: dict) -> None:
        self.duration = saved["duration"]
        self.reason = saved["reason"]
        self.started_at = time.time() - saved["elapsed"]

    def status(self) -> str:
        elapsed = time.time() - self.started_at
        return f"{self.reason}, {elapsed:.0f}/{self.duration:.0f}s"

    def cleanup(self) -> None:
        pass
