"""Auto-discover and register all intents in this package.

Drop a new .py file in here with a @register_intent class, and it's available
to the heartbeat immediately. No imports to update, no registry to maintain.
"""

import importlib
import pkgutil
from pathlib import Path

_pkg_dir = str(Path(__file__).parent)
for mod_info in pkgutil.iter_modules([_pkg_dir]):
    importlib.import_module(f".{mod_info.name}", __package__)
