"""Intent stack runtime and built-in intent implementations.

The Python package that owns intent management for the rover. Two layers:

* `intent_stack` — the base `Intent` class, the `DualStack` runtime that
  manages push/pop/suspend/resume, and the `@register_intent` decorator
  that surfaces an intent to the heartbeat's tool definitions.
* `intents.*` — concrete intent implementations. Each subclasses `Intent`
  and uses `@register_intent` to make itself available. The package
  auto-discovers and imports anything dropped into it.
"""

