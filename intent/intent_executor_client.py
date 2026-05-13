"""Thin HTTP client for the rover-side intent_executor.

Mimics the DualStack interface so heartbeat.py (and other callers) can
treat it as a drop-in replacement for the in-process DualStack. All
mutations and queries go over HTTP to the executor's Flask server.

Heartbeat used to own the DualStack and tick it locally. Now the
executor on the rover owns it and ticks at 10Hz natively. This client
just sends commands and reads status — no tick, no local stack state.
"""

import requests


class _SlotProxy:
    """Mimics _StackSlot's externally-accessed attributes (just_completed).

    Heartbeat reads slot.just_completed and writes False to consume it.
    With the executor authoritative, reads come from /intent/events and
    writes are intercepted: setting False is satisfied by the consume=true
    semantics on the GET.
    """

    def __init__(self, label: str, client: "ExecutorClient"):
        self._label = label  # "nav" or "attention"
        self._client = client
        self._cached_just_completed = False

    @property
    def just_completed(self) -> bool:
        # Trigger a fresh non-consuming peek so polling callers get current
        # state without consuming the flag prematurely.
        events = self._client._fetch_events(consume=False)
        return bool(events.get(f"{self._label}_just_completed", False))

    @just_completed.setter
    def just_completed(self, value: bool) -> None:
        # Only meaningful write is "False" to consume. Force a consuming
        # fetch in that case so the executor clears its flag.
        if value is False:
            self._client._fetch_events(consume=True)


class ExecutorClient:
    """Drop-in replacement for DualStack from the heartbeat side.

    Push, pop, clear, status all proxy to HTTP. Tick is removed — the
    executor ticks itself at 10Hz. The .nav and .attention properties
    expose _SlotProxy objects so existing code that reads
    intent_stack.nav.just_completed continues to work.
    """

    def __init__(self, base_url: str, timeout: float = 2.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.nav = _SlotProxy("nav", self)
        self.attention = _SlotProxy("attention", self)
        # Cached snapshot fields refreshed by status() / properties.
        self._cached_status = "Intent stacks:\n  (executor offline)"
        self._cached_is_empty = True
        self._cached_depth = 0

    # ------------------------------------------------------------------
    # Mutations
    # ------------------------------------------------------------------
    def push(self, intent_name: str, params: dict | None = None) -> str:
        try:
            r = requests.post(
                f"{self.base_url}/intent/push",
                json={"intent": intent_name, "params": params or {}},
                timeout=self.timeout,
            )
            data = r.json()
            return data.get("result", data.get("message", "executor error"))
        except Exception as e:
            return f"executor unreachable: {e}"

    def pop(self, stack: str = "nav") -> str:
        try:
            r = requests.post(
                f"{self.base_url}/intent/pop",
                json={"stack": stack},
                timeout=self.timeout,
            )
            data = r.json()
            return data.get("result", data.get("message", "executor error"))
        except Exception as e:
            return f"executor unreachable: {e}"

    def clear(self, stack: str = "all") -> None:
        try:
            requests.post(
                f"{self.base_url}/intent/clear",
                json={"stack": stack},
                timeout=self.timeout,
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Queries (refresh cache + return)
    # ------------------------------------------------------------------
    def status(self) -> str:
        self._refresh_status()
        return self._cached_status

    @property
    def is_empty(self) -> bool:
        self._refresh_status()
        return self._cached_is_empty

    @property
    def depth(self) -> int:
        self._refresh_status()
        return self._cached_depth

    def _refresh_status(self) -> None:
        try:
            r = requests.get(f"{self.base_url}/intent/status", timeout=self.timeout)
            data = r.json()
            self._cached_status = data.get("stack_status", self._cached_status)
            self._cached_is_empty = bool(data.get("is_empty", True))
            self._cached_depth = int(data.get("depth", 0))
        except Exception:
            # Keep last-known values on transient failure. Executor coming
            # back will refresh on the next call.
            pass

    # ------------------------------------------------------------------
    # Events — backing the slot proxies' just_completed attribute.
    # ------------------------------------------------------------------
    def _fetch_events(self, consume: bool) -> dict:
        try:
            r = requests.get(
                f"{self.base_url}/intent/events",
                params={"consume": "true" if consume else "false"},
                timeout=self.timeout,
            )
            return r.json()
        except Exception:
            return {}

    # ------------------------------------------------------------------
    # No-op tick — DualStack had this; we keep the method for accidental
    # callers but it does nothing because the executor ticks itself.
    # ------------------------------------------------------------------
    def tick(self, beat: int) -> dict:
        return {}
