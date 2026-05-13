"""Intent stacks — the rover's working memory for sustained behaviours.

Two independent stacks run in parallel: **nav** (wheels, body, locomotion)
and **attention** (gimbal, sustained gaze). Each stack is a classic
push/pop with suspend/resume. Per tick, the top of each stack runs
independently — nav commands wheels, attention commands the gimbal,
no cross-stack coordination needed because the actuators are physically
decoupled (OAK-D is body-fixed forward, gimbal is pan-tilt up top).

This is a deliberate divergence from biology: humans have one pair of
eyes that must serve both nav and attention. The rover has dedicated
nav stereo (OAK-D) plus an attention gimbal, so we get parallelism
biology can't.

Intents declare their category via a class attribute::

    class Scan(Intent):
        name = "scan"
        category = "attention"

Default is "nav" — most motion-oriented intents are wheels-bound.

New capabilities are added by writing an Intent subclass in
groundctl/intents/ and decorating it with @register_intent. The
heartbeat adds the corresponding tool definition, and Haiku can start
using it immediately.

Usage::

    from groundctl.intent_stack import DualStack
    from groundctl import intents  # auto-registers all intents

    stack = DualStack(send_command_fn, get_state_fn)
    stack.push("follow", {"target_index": 0})      # routes to nav
    stack.push("scan", {})                          # routes to attention

    # Each heartbeat:
    stack.tick(beat_num)   # ticks both stacks
    print(stack.status())
"""

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass


NAV = "nav"
ATTENTION = "attention"
_CATEGORIES = (NAV, ATTENTION)


@dataclass
class TickContext:
    """State available to intents each heartbeat.

    send_command(cmd_str): send a raw rover command (translates through bridge to ROS2)
    get_state(): returns latest rover state dict — {voltage, position, heading, pan_angle, ...}
    push_intent(name, params): push another intent as a precondition. Routes by category.
    """
    beat: int
    timestamp: float
    send_command: callable
    get_state: callable = None
    push_intent: callable = None


@dataclass
class TickResult:
    """What happened during a tick."""
    complete: bool = False
    status: str = ""


class Intent(ABC):
    """Base class for all rover intents.

    Each intent declares a ``category`` (``"nav"`` or ``"attention"``) that
    determines which stack it lives on. Default is ``"nav"``.

    To create a new intent:
    1. Subclass Intent in groundctl/intents/your_intent.py
    2. Set ``name = "your_intent"`` and ``category = "nav"`` / ``"attention"``
    3. Decorate with @register_intent
    4. Implement start(), tick(), and status()
    5. Add corresponding tool to heartbeat.py TOOLS

    That's it. Haiku can now push it onto the right stack.
    """
    name: str = "unnamed"
    category: str = NAV
    resumable: bool = True  # False = discard on resume (unsafe without GPS)

    @abstractmethod
    def start(self, params: dict) -> None:
        """Initialise from parameters. Called once when pushed."""

    @abstractmethod
    def tick(self, ctx: TickContext) -> TickResult:
        """Execute one step. Called each heartbeat while on top of its stack."""

    def suspend(self) -> dict:
        """Save state before being pushed down. Override if stateful."""
        return {}

    def resume(self, saved: dict) -> None:
        """Restore state when returning to top. Override if stateful."""

    @abstractmethod
    def status(self) -> str:
        """One-line status for Haiku's context window."""

    def cleanup(self) -> None:
        """Called when popped or completed. Override for resource cleanup."""


# --- Registry ---

_registry: dict[str, type[Intent]] = {}


def register_intent(cls):
    """Decorator that registers an Intent subclass by its name."""
    if not hasattr(cls, "name") or cls.name == "unnamed":
        raise ValueError(f"{cls.__name__} must set a 'name' class attribute")
    if getattr(cls, "category", NAV) not in _CATEGORIES:
        raise ValueError(
            f"{cls.__name__}.category must be one of {_CATEGORIES}, "
            f"got {cls.category!r}"
        )
    _registry[cls.name] = cls
    return cls


def get_intent_class(name: str) -> type[Intent] | None:
    return _registry.get(name)


def list_intents() -> list[str]:
    return sorted(_registry.keys())


def list_intents_by_category() -> dict[str, list[str]]:
    out = {NAV: [], ATTENTION: []}
    for name, cls in sorted(_registry.items()):
        out[getattr(cls, "category", NAV)].append(name)
    return out


# --- Single-stack slot ---

class _StackSlot:
    """One stack — either the nav slot or the attention slot. Push/pop with
    suspend/resume. Owned by DualStack; not used directly."""

    def __init__(self, label: str, send_command, get_state=None, push_intent=None):
        self.label = label
        self._stack: list[tuple[Intent, dict | None]] = []
        self._send_command = send_command
        self._get_state = get_state
        self._push_intent = push_intent  # DualStack.push, so intents can add preconditions
        # Set to True by tick() when an intent naturally completes (TickResult.complete=True)
        # and gets popped. Cleared by the heartbeat's check_events() after it reads the flag
        # to fire an "intent_complete" event — which re-triggers the heartbeat immediately
        # rather than waiting out the full idle interval. Does NOT fire for explicit pops
        # triggered by Haiku (pop_intent tool), because those are Haiku's own decisions
        # and don't need a follow-up beat to react to. See CLA-50.
        self.just_completed = False

    def push(self, intent: Intent) -> str:
        """Push an already-constructed, already-started intent onto this slot."""
        if self._stack:
            current, _ = self._stack[-1]
            saved = current.suspend()
            self._stack[-1] = (current, saved)

        self._stack.append((intent, None))
        return f"{self.label}: started {intent.name} — {intent.status()}"

    def pop(self) -> str:
        if not self._stack:
            return f"{self.label}: nothing to pop"

        intent, _ = self._stack.pop()
        intent.cleanup()
        result = f"{self.label}: completed {intent.name}"

        # Cascade through non-resumable intents (unsafe without GPS)
        while self._stack:
            prev, saved = self._stack[-1]
            if not prev.resumable:
                self._stack.pop()
                prev.cleanup()
                result += f", discarded {prev.name} (not resumable)"
                continue
            if saved:
                prev.resume(saved)
                self._stack[-1] = (prev, None)
            result += f", resuming {prev.name}"
            break

        return result

    def peek(self) -> Intent | None:
        return self._stack[-1][0] if self._stack else None

    def tick(self, beat: int) -> str:
        if not self._stack:
            return ""

        intent, _ = self._stack[-1]
        ctx = TickContext(
            beat=beat,
            timestamp=time.time(),
            send_command=self._send_command,
            get_state=self._get_state,
            push_intent=self._push_intent,
        )
        result = intent.tick(ctx)

        if result.complete:
            # Natural completion — flag so the heartbeat's check_events() can
            # re-trigger inference immediately rather than waiting out the
            # idle interval. Explicit pops (via DualStack.pop from Haiku) go
            # through pop() directly and do NOT set this flag.
            self.just_completed = True
            return self.pop()

        return result.status

    def status_lines(self) -> list[str]:
        if not self._stack:
            return [f"  {self.label}: idle"]
        lines = []
        for i, (intent, _) in enumerate(reversed(self._stack)):
            prefix = "active" if i == 0 else f"suspended ({i})"
            lines.append(f"  {self.label}: {prefix} — {intent.name} ({intent.status()})")
        return lines

    @property
    def depth(self) -> int:
        return len(self._stack)

    @property
    def is_empty(self) -> bool:
        return len(self._stack) == 0

    @property
    def active_intent(self) -> str:
        return self._stack[-1][0].name if self._stack else "idle"

    def clear(self):
        while self._stack:
            self.pop()


# --- Dual-stack container ---

class DualStack:
    """Two independent intent stacks: nav (wheels) and attention (gimbal).

    Push by intent name — routing is automatic based on the intent class's
    ``category`` attribute. Pop and clear are stack-targeted so the caller
    can choose which stack to act on.
    """

    def __init__(self, send_command, get_state=None):
        self._send_command = send_command
        self._get_state = get_state
        # Slots get push_intent callback so intents can add preconditions
        # (e.g. follow pushing face_gimbal if the gimbal is off-centre).
        self.nav = _StackSlot(
            "nav", send_command, get_state, push_intent=self.push
        )
        self.attention = _StackSlot(
            "attention", send_command, get_state, push_intent=self.push
        )

    def _slot(self, category: str) -> _StackSlot:
        if category == NAV:
            return self.nav
        if category == ATTENTION:
            return self.attention
        raise ValueError(f"unknown category {category!r}")

    def push(self, intent_name: str, params: dict = None) -> str:
        cls = get_intent_class(intent_name)
        if not cls:
            available = ", ".join(list_intents()) or "none"
            return f"Unknown intent '{intent_name}'. Available: {available}"

        intent = cls()
        intent.start(params or {})
        slot = self._slot(getattr(cls, "category", NAV))
        return slot.push(intent)

    def pop(self, stack: str = NAV) -> str:
        return self._slot(stack).pop()

    def clear(self, stack: str = "all"):
        if stack == "all":
            self.nav.clear()
            self.attention.clear()
        else:
            self._slot(stack).clear()

    def tick(self, beat: int) -> dict[str, str]:
        """Tick both stacks. Returns {category: status_line} for any with output."""
        results = {}
        nav_out = self.nav.tick(beat)
        if nav_out:
            results[NAV] = nav_out
        att_out = self.attention.tick(beat)
        if att_out:
            results[ATTENTION] = att_out
        return results

    def status(self) -> str:
        """Formatted both-stacks status for Haiku's context window."""
        lines = ["Intent stacks:"]
        lines.extend(self.nav.status_lines())
        lines.extend(self.attention.status_lines())
        return "\n".join(lines)

    @property
    def is_empty(self) -> bool:
        return self.nav.is_empty and self.attention.is_empty

    @property
    def depth(self) -> int:
        return self.nav.depth + self.attention.depth


# --- Backwards-compat alias ---
# Older code may import IntentStack. Keep the name working so the refactor
# lands atomically without needing lockstep edits everywhere.
IntentStack = DualStack
