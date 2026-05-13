"""Look at — point the gimbal at a pan/tilt position. Completes in one tick."""

from ..intent_stack import Intent, TickResult, TickContext, register_intent


@register_intent
class LookAt(Intent):
    name = "look_at"
    category = "attention"

    def start(self, params: dict) -> None:
        self.pan = max(-90, min(90, params.get("pan", 0)))
        self.tilt = max(-45, min(90, params.get("tilt", 0)))
        self.done = False

    def tick(self, ctx: TickContext) -> TickResult:
        if self.done:
            return TickResult(complete=True, status=f"Looking at pan={self.pan:.0f} tilt={self.tilt:.0f}")

        ctx.send_command(
            f'base -c {{"T":133,"X":{self.pan},"Y":{self.tilt},"SPD":60,"ACC":0.4}}'
        )
        self.done = True
        return TickResult(complete=True, status=f"Pointed at pan={self.pan:.0f} tilt={self.tilt:.0f}")

    def status(self) -> str:
        return f"pan={self.pan:.0f} tilt={self.tilt:.0f}"

    def cleanup(self) -> None:
        pass
