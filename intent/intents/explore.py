"""Explore — wander around, looking at things. The default active behaviour.

Makes small random movements and gimbal sweeps. Designed to be the "background
intent" that gets interrupted by more specific intents (follow person, go to
place) and resumed when they complete.
"""

import time
import random
from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class Explore(Intent):
    name = "explore"

    def start(self, params: dict) -> None:
        self.speed = min(0.2, max(0.05, params.get("speed", 0.1)))
        self.duration = params.get("duration", 0.0)  # 0 = indefinite
        self.started_at = time.time()
        self.last_action_at = 0.0
        self.action_interval = params.get("interval", 16.0)  # seconds between movements
        self.ticks = 0

    def tick(self, ctx: TickContext) -> TickResult:
        self.ticks += 1
        elapsed = time.time() - self.started_at

        if self.duration > 0 and elapsed >= self.duration:
            ctx.send_command('base -c {"T":1,"L":0,"R":0}')
            return TickResult(complete=True, status=f"Explored for {elapsed:.0f}s")

        since_action = time.time() - self.last_action_at
        if since_action >= self.action_interval:
            self.last_action_at = time.time()
            action = random.choice(["nudge", "turn", "look"])

            if action == "nudge":
                ctx.send_command(
                    f'base -c {{"T":1,"L":{self.speed},"R":{self.speed}}}'
                )
            elif action == "turn":
                direction = random.choice([-1, 1])
                ctx.send_command(
                    f'base -c {{"T":1,"L":{self.speed * direction},"R":{-self.speed * direction}}}'
                )
            elif action == "look":
                pan = random.randint(-60, 60)
                tilt = random.randint(-20, 30)
                ctx.send_command(
                    f'base -c {{"T":133,"X":{pan},"Y":{tilt},"SPD":60,"ACC":0.4}}'
                )
                return TickResult(status=f"Exploring — looking around (pan={pan})")

            return TickResult(status=f"Exploring — {action}")

        return TickResult(status=f"Exploring — observing ({self.ticks} ticks)")

    def suspend(self) -> dict:
        return {
            "elapsed": time.time() - self.started_at,
            "speed": self.speed,
            "duration": self.duration,
            "ticks": self.ticks,
            "action_interval": self.action_interval,
        }

    def resume(self, saved: dict) -> None:
        self.speed = saved["speed"]
        self.duration = saved["duration"]
        self.ticks = saved["ticks"]
        self.action_interval = saved["action_interval"]
        self.started_at = time.time() - saved["elapsed"]
        self.last_action_at = 0.0

    def status(self) -> str:
        elapsed = time.time() - self.started_at
        if self.duration > 0:
            return f"exploring, {elapsed:.0f}/{self.duration:.0f}s"
        return f"exploring, {elapsed:.0f}s"

    def cleanup(self) -> None:
        pass
