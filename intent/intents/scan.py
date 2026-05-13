"""Scan — sweep the gimbal across the field of view.

Use when Haiku wants to "look around" — it automatically pans through a
sequence of positions, holding each so the camera can capture stable frames.
The heartbeat sees a different angle each beat, so Haiku can react to what
appears in any of them.

Defaults: -60° to +60° in 30° steps, slight tilt down. Returns to centre when done.
"""

import time
from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class Scan(Intent):
    name = "scan"
    category = "attention"
    resumable = False

    def start(self, params: dict) -> None:
        self.range = min(90, max(15, params.get("range", 60)))
        self.steps = min(10, max(3, params.get("steps", 5)))
        self.tilt = max(-30, min(30, params.get("tilt", -10)))
        # Hold should match heartbeat interval so each beat sees a different angle.
        # Default 15s aligns with the 12s heartbeat plus inference time.
        self.hold_seconds = max(5.0, min(30.0, params.get("hold", 15.0)))

        # Build pan sequence: e.g. for range=60, steps=5 → [-60, -30, 0, 30, 60, 0]
        # Always end at 0 to leave gimbal centred
        step_size = (self.range * 2) / (self.steps - 1)
        self.positions = [round(-self.range + step_size * i) for i in range(self.steps)]
        self.positions.append(0)  # return to centre

        self.current_index = 0
        self.last_move_at = 0.0
        self.started_at = time.time()

    def tick(self, ctx: TickContext) -> TickResult:
        now = time.time()
        if self.current_index >= len(self.positions):
            return TickResult(complete=True, status=f"Scan complete ({len(self.positions)-1} positions)")

        # Time to move to next position?
        if now - self.last_move_at >= self.hold_seconds:
            target = self.positions[self.current_index]
            ctx.send_command(
                f'base -c {{"T":133,"X":{target},"Y":{self.tilt},"SPD":80,"ACC":0.5}}'
            )
            self.current_index += 1
            self.last_move_at = now
            elapsed = now - self.started_at
            if self.current_index >= len(self.positions):
                return TickResult(
                    complete=True,
                    status=f"Scan complete — last position pan={target}° (took {elapsed:.0f}s)"
                )
            return TickResult(status=f"Scanning pan={target}° tilt={self.tilt}° ({self.current_index}/{len(self.positions)})")

        return TickResult(status=f"Holding pan={self.positions[self.current_index-1]}°")

    def status(self) -> str:
        if self.current_index < len(self.positions):
            return f"scanning {self.current_index}/{len(self.positions)}"
        return "scan complete"

    def cleanup(self) -> None:
        pass
