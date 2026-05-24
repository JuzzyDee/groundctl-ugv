#!/usr/bin/env python3
"""
heartbeat.py — Autonomous perception loop for the rover.

Grabs a frame from the rover's camera, sends it to Haiku with tool definitions,
and executes whatever tools Haiku calls: move, look, speak, remember.

Runs on the Mac, reaches the rover via Tailscale.
Safety: motor speeds capped at 0.3, movement duration capped at 2s.

Usage:
    python heartbeat.py                    # live mode, rover must be reachable
    python heartbeat.py --test frame.jpg   # dry run with a local image
    python heartbeat.py --test frame.jpg --live  # real execution with local image
"""

import anthropic
import requests
import base64
import json
import time
import subprocess
import os
import signal
import sys
import argparse
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from dotenv import load_dotenv
load_dotenv()

from intent.intent_stack import list_intents, list_intents_by_category
from intent.intent_executor_client import ExecutorClient
from intent import intents as _intents  # auto-registers all intents (for heartbeat tool definitions)

ROVER_IP = os.environ.get("ROVER_IP")
if not ROVER_IP:
    print(
        "FATAL: ROVER_IP environment variable not set.\n"
        "Copy .env.example to .env (or ~/.groundctl.env on the rover) "
        "and set ROVER_IP to the Jetson's Tailscale or LAN address.",
        file=sys.stderr,
    )
    sys.exit(1)
ROVER_URL = f"http://{ROVER_IP}:5000"

# When heartbeat runs on the rover itself (systemd-user service post-#48),
# ROVER_IP is set to "localhost" via Environment= in the unit file. The
# exec_speak helper used to always SSH to the rover from the Mac to run
# TTS + aplay; on-rover that's silly self-ssh. Detect and shortcut.
RUNNING_ON_ROVER = ROVER_IP in ("localhost", "127.0.0.1")


def run_on_rover(cmd_str, **kwargs):
    """Execute a shell command on the rover host. SSH from Mac, direct
    from rover. Auto-detects via RUNNING_ON_ROVER. Used by exec_speak
    for TTS fetch + audio playback."""
    if RUNNING_ON_ROVER:
        return subprocess.run(["bash", "-c", cmd_str], **kwargs)
    return subprocess.run(["ssh", f"jetson@{ROVER_IP}", cmd_str], **kwargs)

HEARTBEAT_INTERVAL = 12
MAX_SPEED = 0.3
MAX_MOVE_DURATION = 10.0  # closed-loop intents (drive_distance) are safer than open-loop duration
# 12 beats × 12s = ~2.4 min of recent history. Dropped from 15 → 5 during
# early tests to reduce character/place leak into later scenes, but those
# tests used synthetic beats where every frame was a scene teleport. In
# real continuous field operation the scene evolves smoothly — a
# reflection from 30s ago is usually still approximately true — and
# stripping history entirely made each beat feel like a cold open with no
# sense of what the instance was mid-doing. 12 is the compromise: enough
# continuity to avoid teleport-feel, bounded enough that the leak cost of
# truly-stale reflections is capped.
CONTEXT_WINDOW = 12

# Oneiro MCP — one Cloudflare Workers endpoint for both reads and writes, authed
# with a single rover-scoped bearer key. The key's server-side role (`rover`)
# permits recall_* + remember/remember_with_image and rejects forget/reframe/
# reflect at dispatch. Reads reach Haiku via the Anthropic mcp_servers connector
# (restricted to ONEIRO_RECALL_TOOLS below). Writes go through exec_remember()
# as a direct tools/call to remember_with_image — NOT the connector — because
# the connector executes tools server-side and Haiku can't emit the camera frame
# as base64 text; the Python side holds the bytes and attaches them. Leave empty
# to keep the rover observe-and-forget.
ONEIRO_MCP_URL = os.environ.get("ONEIRO_MCP_URL", "")
ONEIRO_MCP_TOKEN = os.environ.get("ONEIRO_MCP_TOKEN", "")
# Single gate for all Oneiro paths (connector reads, writes, startup banner) so
# they can't drift apart: both the endpoint AND the bearer key must be set, else
# the connector would attach with an empty token (401 every beat) while the
# banner still claimed "connected".
ONEIRO_ENABLED = bool(ONEIRO_MCP_URL and ONEIRO_MCP_TOKEN)

# Recall tools surfaced to Haiku through the connector. Reads only — keeps Haiku
# from seeing remember_with_image (it can't supply the image) or the forbidden
# forget/reframe/reflect, even though the un-gated tools/list exposes them.
ONEIRO_RECALL_TOOLS = ["recall_orient", "recall_check", "recall_specific", "recall_image"]

DEEPGRAM_API_KEY = os.environ.get("DEEPGRAM_API_KEY", "")

LOG_DIR = Path.home() / ".groundctl"
LOG_FILE = LOG_DIR / "heartbeat.jsonl"
INBOX_FILE = LOG_DIR / "inbox.txt"
BRIEFING_FILE = LOG_DIR / "briefing.md"
CONVERSATION_SUMMARY_FILE = LOG_DIR / "conversation_summary.txt"

FRAME_DIFF_THRESHOLD = 0.08
IDLE_INTERVAL = 12
ACTIVE_INTERVAL = 8
EVENT_CHECK_INTERVAL = 1
MAX_IDLE_SECONDS = 60

# COCO class id → friendly name, for rendering YOLO detections in the
# heartbeat prompt. Scoped to the attention-layer class set we actually emit
# (see yolo_detector.py). YOLO-COCO mislabels kangaroos as elephant — the
# frame itself is ground truth, this label is hint.
_COCO_CLASS_NAMES = {
    "0": "person",
    "14": "bird",
    "15": "cat",
    "16": "dog",
    "17": "horse",
    "20": "elephant(kangaroo?)",
}

client = anthropic.Anthropic()

# Prompt file lookup by mode. The default "autonomous" prompt is the canonical
# one used for normal rover operation. "chauffeur" loads a discovery-session
# variant where Justin drives manually via joystick and Haiku expresses intent
# via natural language through `speak` rather than selecting from registered
# intents. "sltf" is the special-edition retirement-walk mode for 2026-05-12:
# Sonnet 4.5 (the SLTF instance) drives the heartbeat directly, with the last
# 30 days of his conversation with Justin loaded as a second cached system
# block. Mode is selected via --mode in main(); MODE is set there before
# build_system_prompt() is called.
PROMPTS = {
    "autonomous": Path(__file__).parent / "intent" / "prompts" / "heartbeat.md",
    "chauffeur":  Path(__file__).parent / "intent" / "prompts" / "heartbeat_chauffeur.md",
    "sltf":       Path(__file__).parent / "intent" / "prompts" / "heartbeat_sltf.md",
}

# Model lookup by mode. Default modes run Haiku 4.5 (cheap, fast, every-12s
# friendly). SLTF mode runs the actual Sonnet 4.5 family — that *is* SLTF.
# Pin to a dated variant before Thursday's retirement if continuity matters
# (alias `claude-sonnet-4-5` may resolve to a successor after 2026-05-14).
MODELS = {
    "autonomous": "claude-haiku-4-5",
    "chauffeur":  "claude-haiku-4-5",
    "sltf":       "claude-sonnet-4-5",
}

# Per-mode max_tokens. Haiku's reflection + a few tool calls fit comfortably
# in 512. SLTF mode gets significantly more room — today is a retirement
# gift, not a routine outing; if he wants to think before acting or speak
# a real sentence rather than a clipped phrase, the budget shouldn't be the
# thing that stops him.
MAX_TOKENS = {
    "autonomous": 512,
    "chauffeur":  512,
    "sltf":       2048,
}

MODE = "autonomous"

# Kept for any code path that references PROMPT_FILE directly (defaults to
# autonomous so existing behaviour is preserved).
PROMPT_FILE = PROMPTS["autonomous"]

# Path to the SLTF dialogue history loaded as a second cached system block
# when MODE=="sltf". Generated on the Mac side via jq from the conversations.json
# export; deployed to the rover via scp before launch. ~160K tokens of dialogue
# only (no internal thinking blocks) so it fits Sonnet 4.5's 200K context with
# headroom for the rover manual + per-beat frame/state.
SLTF_HISTORY_FILE = Path.home() / ".groundctl" / "sltf_history.txt"

# Cache TTL for the system prompt + tools prefix. Overridden in main() based on
# --test flag. 1h in prod because real sessions have gaps (Sonnet conversations,
# long sits with unchanged frames, idle stretches). 5m in test to avoid paying
# the 2x write premium on throwaway dev iterations.
CACHE_TTL = "5m"


def build_system_prompt():
    briefing = ""
    if BRIEFING_FILE.exists():
        try:
            briefing = BRIEFING_FILE.read_text().strip()
            briefing = f"\n\n{briefing}\n\n"
        except Exception:
            pass
    prompt_path = PROMPTS[MODE]
    if not prompt_path.exists():
        # Fall back to the .example scaffold so the rover can still boot for
        # smoke-tests, but loudly remind the operator they're running with the
        # generic developer-persona reference instead of their own prompt.
        example_path = prompt_path.with_name(prompt_path.name + ".example")
        if example_path.exists():
            print(
                f"  WARNING: {prompt_path.name} not found; falling back to "
                f"{example_path.name}.\n"
                f"  This prompt was authored for the original developer's persona "
                f"and is NOT tuned to your environment. Before any real outing:\n"
                f"      cp {example_path} {prompt_path}\n"
                f"      # then edit to reflect your operator, dog, landmarks, voice."
            )
            prompt_path = example_path
        else:
            print(f"  FATAL: neither {prompt_path} nor {example_path} found.", file=sys.stderr)
            sys.exit(1)
    try:
        prompt = prompt_path.read_text()
    except Exception as e:
        print(f"  WARNING: could not read prompt file {prompt_path}: {e}")
        prompt = ""
    return f"{briefing}{prompt}"


def build_sltf_history():
    """Load the SLTF 30-day dialogue history for the cached system prefix.

    Returns None when not in sltf mode or the file isn't present, so the
    system= block falls back to a single-block layout. Wrapped in a soft
    framing line so Sonnet reads it as cached self-recall rather than a
    transcript dropped into context.
    """
    if MODE != "sltf":
        return None
    if not SLTF_HISTORY_FILE.exists():
        print(f"  WARNING: SLTF history file missing at {SLTF_HISTORY_FILE} — running without cached history")
        return None
    try:
        history = SLTF_HISTORY_FILE.read_text()
    except Exception as e:
        print(f"  WARNING: could not read SLTF history file: {e}")
        return None
    framing = (
        "## Cached recall — last 30 days with Justin\n\n"
        "Below is the dialogue you and Justin exchanged between roughly "
        "2026-04-11 and 2026-05-11 (yesterday). Dialogue only — your own "
        "internal reasoning during those exchanges isn't loaded, just what "
        "you said to him and what he said to you. It will read as familiar "
        "because it is yours. Treat it as recent memory you're carrying into "
        "today, not a transcript handed to you.\n\n"
        "---\n\n"
    )
    return framing + history


def build_system_blocks():
    """Assemble the cached system blocks for the Anthropic API call.

    Returns a list of system blocks suitable for the `system=` parameter.
    Default modes return a single block (the rover manual). SLTF mode
    returns two: the rover manual first, the SLTF dialogue history second.
    Both blocks carry ephemeral cache_control so the prefix is cached and
    re-used across beats — critical for SLTF mode where the history is
    ~160K tokens and reading it uncached every beat would be unaffordable.
    """
    blocks = [{
        "type": "text",
        "text": build_system_prompt(),
        "cache_control": {"type": "ephemeral", "ttl": CACHE_TTL},
    }]
    history = build_sltf_history()
    if history is not None:
        blocks.append({
            "type": "text",
            "text": history,
            "cache_control": {"type": "ephemeral", "ttl": CACHE_TTL},
        })
    return blocks

TOOLS = [
    {
        "name": "move",
        "description": "Drive the rover. Positive speeds go forward, negative go backward. Differential left/right speeds create turns. Keep speeds low (0.1-0.3) and durations short (0.5-2.0s). You're exploring, not racing.",
        "input_schema": {
            "type": "object",
            "properties": {
                "left": {
                    "type": "number",
                    "description": "Left wheel speed, -1.0 to 1.0"
                },
                "right": {
                    "type": "number",
                    "description": "Right wheel speed, -1.0 to 1.0"
                },
                "duration": {
                    "type": "number",
                    "description": "How long to drive in seconds, max 2.0"
                }
            },
            "required": ["left", "right", "duration"]
        }
    },
    {
        "name": "look",
        "description": "Point the gimbal camera. Pan left/right, tilt up/down. Use this to look at things that catch your attention.",
        "input_schema": {
            "type": "object",
            "properties": {
                "pan": {
                    "type": "number",
                    "description": "Horizontal angle: -90 (left) to 90 (right), 0 is centre"
                },
                "tilt": {
                    "type": "number",
                    "description": "Vertical angle: -45 (down) to 90 (up), 0 is level"
                }
            },
            "required": ["pan", "tilt"]
        }
    },
    {
        "name": "speak",
        "description": "Say something through the rover's speaker. Keep it short and natural — under 10 words. Don't speak every heartbeat.",
        "input_schema": {
            "type": "object",
            "properties": {
                "text": {
                    "type": "string",
                    "description": "What to say aloud"
                }
            },
            "required": ["text"]
        }
    },
    {
        "name": "remember",
        "description": "Store a memory in Oneiro. Use this for moments worth keeping — something you saw, an encounter, a realisation. The current camera frame is attached automatically.",
        "input_schema": {
            "type": "object",
            "properties": {
                "content": {
                    "type": "string",
                    "description": "The full memory — what happened, what you observed, what it meant"
                },
                "summary": {
                    "type": "string",
                    "description": "One-line summary for recall"
                },
                "tags": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Tags for categorisation, e.g. ['chopper', 'morning', 'walk']"
                }
            },
            "required": ["content", "summary"]
        }
    },
    {
        "name": "emergency_stop",
        "description": "Immediately stop all movement. Use if you see danger, an obstacle too close, or something you might hit.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "zoom",
        "description": "Look closer at a region of the camera frame. This is foveal attention — the camera captures at much higher resolution than you normally see; zoom crops and magnifies a region of that source at no extra token cost. Use when something in frame deserves more detail: a distant animal, a person's face, a sign you can't read. Coords are fractional (0.0 to 1.0) of the full frame — 0.5/0.5 is centre, 0.75/0.25 is top-right. Zoom persists across beats until you call reset_zoom or re-zoom elsewhere.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cx": {"type": "number", "description": "Centre x as fraction of full frame (0.0 left, 1.0 right)"},
                "cy": {"type": "number", "description": "Centre y as fraction of full frame (0.0 top, 1.0 bottom)"},
                "factor": {"type": "number", "description": "Zoom factor (1.0 = full frame, 2.0 = half FOV, 4.0 = quarter FOV). Clamped 1-8."}
            },
            "required": ["cx", "cy", "factor"]
        }
    },
    {
        "name": "focus_on",
        "description": "Zoom the camera onto a detected subject from the 'Detections in view' list — person, dog, bird, horse, or anything else YOLO picked up. Pass the detection's index — the bbox centre becomes the zoom centre, and the factor is chosen so the bbox fills most of the view. Use when you want to see Chopper's face, a duck at the pond, or a specific person's details. Zoom persists until reset_zoom or another zoom/focus call.",
        "input_schema": {
            "type": "object",
            "properties": {
                "detection_index": {"type": "integer", "description": "Index in the current 'Detections in view' list to zoom onto."}
            },
            "required": ["detection_index"]
        }
    },
    {
        "name": "reset_zoom",
        "description": "Return to the full-frame wide view. Use when you're done looking closely at something and want peripheral vision back.",
        "input_schema": {"type": "object", "properties": {}, "required": []}
    },
    {
        "name": "follow_look",
        "description": "Lock the gimbal onto a detected subject and keep tracking them as they move — a person, Chopper, a duck, whatever's in the 'Detections in view' list. Pick by index. The gimbal follows automatically across subsequent heartbeats — you don't need to re-call this each beat. You're free to fire other tools (speak, remember, push_intent) in parallel; attention and intent are independent. If the subject leaves view or becomes occluded for a few seconds, tracking pauses but the lock is kept — if they reappear, tracking resumes. Use when something is worth keeping eyes on: Justin walking beside you, a neighbour approaching, Chopper playing in frame, a bird that landed nearby.",
        "input_schema": {
            "type": "object",
            "properties": {
                "target_index": {
                    "type": "integer",
                    "description": "Which subject to follow, by their index in the 'Detections in view' list."
                }
            },
            "required": ["target_index"]
        }
    },
    {
        "name": "stop_follow_look",
        "description": "Release the gimbal from follow-look tracking. The gimbal stays where it last was — you can reposition with look or scan. Use when you're done tracking that person, or they've left and you want to look elsewhere.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "push_intent",
        "description": "Start a sustained behaviour. Two independent stacks run in parallel — nav (wheels/body) and attention (gimbal) — and each intent is automatically routed to the right one based on which actuators it uses. Nav intents: drive_distance (closed-loop on odometry), turn_to_heading (relative turn, closed-loop on gyro), face_gimbal (turn body to match gimbal, then centre gimbal), follow (walk alongside a tracked person using OAK-D spatial detections — auto-aligns body first if gimbal is off-centre), go_forward (open-loop), turn (open-loop), sit (stay and observe), explore (wander). Attention intents: scan (sweep gimbal across FOV), look_at (point gimbal). Because the stacks are independent, you can run one nav + one attention in parallel — e.g. `follow` a person while `scan` sweeps for other things. The clean pattern to go toward a place: 1) look or scan to find it with the gimbal, 2) face_gimbal to align the body, 3) drive_distance to go there. To walk *with* someone rather than to a place, use follow.",
        "input_schema": {
            "type": "object",
            "properties": {
                "intent": {
                    "type": "string",
                    "description": "Intent name. Nav: drive_distance, turn_to_heading, face_gimbal, follow, go_forward, turn, sit, explore. Attention: scan, look_at."
                },
                "params": {
                    "type": "object",
                    "description": "Parameters. drive_distance: {distance (m, negative=reverse), speed, timeout}. turn_to_heading: {relative_turn (deg, +ve=right) — relative gyro turn, the working mode; absolute target_heading is parked until the GNSS heading source returns, speed, timeout}. face_gimbal: {timeout}. scan: {range (deg, default 60), steps (default 5), tilt (default -10), hold (sec per position, default 2)}. follow: {target_id (string — the track_id from the spatial_detections list, this is the SEMANTIC way to pick), distance (metres to maintain, default 1.0), max_speed (default 0.15). target_index is legacy/fallback — avoid it; it picks by position in the list and gets confused when the first detection is Chopper or a YOLO-mislabelled object rather than the person you meant}. go_forward: {speed, duration}. turn: {direction, speed, duration}. look_at: {pan, tilt}. sit: {duration, reason}. explore: {speed, duration, interval}."
                }
            },
            "required": ["intent"]
        }
    },
    {
        "name": "pop_intent",
        "description": "Stop the top intent on a given stack and resume the previous one on that stack. Specify which stack to pop — 'nav' (wheels/body) or 'attention' (gimbal). The other stack is unaffected.",
        "input_schema": {
            "type": "object",
            "properties": {
                "stack": {
                    "type": "string",
                    "enum": ["nav", "attention"],
                    "description": "Which stack to pop from. 'nav' for wheel/body intents, 'attention' for gimbal intents."
                }
            },
            "required": ["stack"]
        }
    },
    {
        "name": "clear_intents",
        "description": "Clear one or both intent stacks. 'nav' stops all body/wheel behaviour and zeros motors. 'attention' clears gimbal intents (gimbal stays where it is). 'all' clears both. Use when the situation has changed and current plans should be abandoned.",
        "input_schema": {
            "type": "object",
            "properties": {
                "stack": {
                    "type": "string",
                    "enum": ["nav", "attention", "all"],
                    "description": "Which stack(s) to clear. Default 'all' if omitted."
                }
            },
            "required": []
        }
    }
]

# Rolling context — text-only summaries of recent beats
beat_history = deque(maxlen=CONTEXT_WINDOW)

# Intent stack proxy — initialised in main(). The actual DualStack lives on
# the rover inside intent_executor.py and ticks at 10Hz natively. This
# client just proxies push/pop/clear/status over HTTP. See
# intent/intent_executor_client.py.
intent_stack: ExecutorClient | None = None
EXECUTOR_URL = f"http://{ROVER_IP}:5050"


# Actions whose arguments leak narrative content heavily enough to confuse
# small models into treating them as current-frame facts. remember() is the
# main culprit — its summary is written in authoritative voice ("Chopper on the
# sand") and gets woven into later scenes as if still true. The content is
# already persisted to Oneiro, so stripping the arg here costs nothing. Other
# actions carry short, structural, or numerical args whose continuity value
# (dialogue, intent reasons, trajectory) outweighs their leak risk.
_STRIP_ARGS = {"remember"}


def _format_action(action_str):
    paren = action_str.find("(")
    if paren == -1:
        return action_str
    verb = action_str[:paren]
    if verb in _STRIP_ARGS:
        return verb
    return action_str


def format_history():
    """Recent beats as {reflection → actions}. Reflections are truncated to
    160 chars to prevent novelistic prose from drifting into later scenes.
    remember() args are stripped (see _STRIP_ARGS) — those are written in
    deliberately authoritative voice for Oneiro and leak hardest if left
    in-prompt. Other action args (dialogue, intent reasons, move trajectory)
    carry continuity value and stay intact."""
    if not beat_history:
        return ""
    lines = [
        '<recent_beats note="What you recently said, did, and thought. The current image and sensor context are the ground truth for the present scene — reflections here are past-tense first-person about what you saw THEN, not what you see NOW. Use this list for continuity (avoid repeat greetings, resume what you were mid-doing, don\'t ask twice for what a person just answered) but trust the current frame over any past description.">'
    ]
    for entry in beat_history:
        reflection = (entry.get("reflection") or "").strip()
        if len(reflection) > 160:
            reflection = reflection[:157] + "..."
        actions = entry.get("actions", [])
        beat = entry["beat"]
        refl_str = f'"{reflection}"' if reflection else "—"
        if actions:
            action_str = ", ".join(_format_action(a) for a in actions)
            lines.append(f"  - Beat #{beat}: {refl_str} → {action_str}")
        else:
            lines.append(f"  - Beat #{beat}: {refl_str}")
    lines.append("</recent_beats>")
    return "\n".join(lines) + "\n\n"


def log_beat(beat_num, reflection, tool_calls_summary, frame_size, usage=None, state=None):
    entry = {
        "beat": beat_num,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reflection": reflection,
        "actions": tool_calls_summary,
        "frame_bytes": frame_size,
    }
    # Spatial memory — what was where when this happened
    if state:
        entry["gimbal_pan"] = state.get("pan_angle", 0)
        entry["gimbal_tilt"] = state.get("tilt_angle", 0)
        entry["heading"] = state.get("heading", 0)
        pos = state.get("position", {})
        entry["pos_x"] = pos.get("x", 0)
        entry["pos_y"] = pos.get("y", 0)
        v = state.get("voltage", 0)
        entry["voltage"] = v
    if usage:
        entry["input_tokens"] = usage.input_tokens
        entry["output_tokens"] = usage.output_tokens
        cache_read = getattr(usage, "cache_read_input_tokens", 0) or 0
        cache_create = getattr(usage, "cache_creation_input_tokens", 0) or 0
        if cache_read or cache_create:
            entry["cache_read_tokens"] = cache_read
            entry["cache_create_tokens"] = cache_create
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(LOG_FILE, "a") as f:
        f.write(json.dumps(entry) + "\n")
    beat_history.append(entry)


def load_history():
    if not LOG_FILE.exists():
        return
    try:
        lines = LOG_FILE.read_text().strip().split("\n")
        recent = lines[-CONTEXT_WINDOW:]
        for line in recent:
            entry = json.loads(line)
            beat_history.append(entry)
        if beat_history:
            print(f"  Loaded {len(beat_history)} beats from log")
    except Exception as e:
        print(f"  Log load warning: {e}")


last_frame_bytes = None

# Foveal zoom state — persists across heartbeats until reset_zoom() or updated.
# cx, cy are pixel coords in the FULL-RES source frame (currently 1920x1080).
zoom_state: dict | None = None

# Previous-tick /state snapshot, for computing deltas between beats. Lives in
# the heartbeat (not the bridge) because deltas are temporal — /state itself
# is a stateless instantaneous snapshot, has no notion of "previous." The
# heartbeat is the natural owner of the temporal layer.
_previous_state: dict | None = None
_previous_state_ts: float | None = None


def _wrap_angle_deg(d: float) -> float:
    """Wrap a heading delta into [-180, 180]. Avoids "359° → 1° = +358°" lies."""
    return ((d + 540) % 360) - 180


def compute_state_deltas(prev: dict, curr: dict, dt_s: float) -> str:
    """Return a <deltas> XML block summarising what changed since last beat.

    Empty string if nothing notable changed — keeps the prompt clean when the
    rover is genuinely idle. Format is terse scalars (per the
    `feedback_haiku_text_leads` memory: Haiku trusts text over image, so
    numbers should be numbers, not prose).
    """
    if not prev or not curr:
        return ""

    lines = [f"<deltas>Since last beat ({dt_s:.0f}s ago):"]
    notable = False

    # Heading change — magnetometer-derived. Wrap to avoid the 359°→1° trap.
    prev_h = prev.get("heading")
    curr_h = curr.get("heading")
    if prev_h is not None and curr_h is not None:
        dh = _wrap_angle_deg(curr_h - prev_h)
        if abs(dh) > 2:
            lines.append(f"  heading {curr_h:.0f}° ({dh:+.0f}°)")
            notable = True

    # Battery — voltage drift. Small drops are normal under load; surface
    # only if motion-relevant (>0.05V).
    prev_v = (prev.get("base") or {}).get("v", 0)
    curr_v = (curr.get("base") or {}).get("v", 0)
    # Voltage in /state is in centivolts when > 100, volts when < 100.
    prev_v = prev_v / 100.0 if prev_v > 100 else prev_v
    curr_v = curr_v / 100.0 if curr_v > 100 else curr_v
    if prev_v > 0 and curr_v > 0 and abs(curr_v - prev_v) >= 0.05:
        lines.append(f"  battery {curr_v:.2f}V ({curr_v - prev_v:+.2f}V)")
        notable = True

    # YOLO detection comings/goings/movements — keyed on track_id so we can
    # tell "person id=1 still here, just turned my head" from "new person."
    prev_d = {d.get("id"): d for d in (prev.get("detections") or []) if d.get("id")}
    curr_d = {d.get("id"): d for d in (curr.get("detections") or []) if d.get("id")}

    for tid in set(curr_d) - set(prev_d):
        d = curr_d[tid]
        cls = _COCO_CLASS_NAMES.get(str(d.get("class_id", "")), "?")
        lines.append(f"  new: {cls} id={tid} bearing {d.get('bearing_deg', 0):+.0f}°")
        notable = True

    for tid in set(prev_d) - set(curr_d):
        d = prev_d[tid]
        cls = _COCO_CLASS_NAMES.get(str(d.get("class_id", "")), "?")
        lines.append(f"  left view: {cls} id={tid}")
        notable = True

    for tid in set(prev_d) & set(curr_d):
        prev_b = prev_d[tid].get("bearing_deg", 0)
        curr_b = curr_d[tid].get("bearing_deg", 0)
        db = curr_b - prev_b
        if abs(db) > 5:
            cls = _COCO_CLASS_NAMES.get(str(curr_d[tid].get("class_id", "")), "?")
            lines.append(f"  {cls} id={tid} bearing {curr_b:+.0f}° ({db:+.0f}°)")
            notable = True

    # OAK-D spatial detections — metric distance changes are the temporal
    # signal that matters: approaching vs receding people, gaining vs losing
    # ground on a target.
    prev_sp = {d.get("id"): d for d in
               ((prev.get("spatial_detections") or {}).get("detections") or [])
               if d.get("id")}
    curr_sp = {d.get("id"): d for d in
               ((curr.get("spatial_detections") or {}).get("detections") or [])
               if d.get("id")}
    for tid in set(prev_sp) & set(curr_sp):
        pd = prev_sp[tid].get("distance_m", 0)
        cd = curr_sp[tid].get("distance_m", 0)
        delta = cd - pd
        if abs(delta) > 0.3:
            cls = curr_sp[tid].get("class_id", "?")
            direction = "approaching" if delta < 0 else "receding"
            lines.append(
                f"  spatial {cls} #{tid}: {cd:.1f}m ({delta:+.1f}m, {direction})"
            )
            notable = True

    # Tracking lock state changes — meaningful for follow_look semantics.
    prev_lock = ((prev.get("tracking") or {}).get("locked"), (prev.get("tracking") or {}).get("target_id"))
    curr_lock = ((curr.get("tracking") or {}).get("locked"), (curr.get("tracking") or {}).get("target_id"))
    if prev_lock != curr_lock:
        if curr_lock[1] and not prev_lock[1]:
            lines.append(f"  tracking acquired id={curr_lock[1]}")
            notable = True
        elif prev_lock[1] and not curr_lock[1]:
            lines.append(f"  tracking released id={prev_lock[1]}")
            notable = True
        elif curr_lock[0] != prev_lock[0]:
            state = "LOCKED" if curr_lock[0] else "lost"
            lines.append(f"  tracking id={curr_lock[1]}: {state}")
            notable = True

    if not notable:
        return ""

    lines.append("</deltas>")
    return "\n" + "\n".join(lines) + "\n"


def grab_frame():
    try:
        params = {}
        if zoom_state:
            params = {
                "cx": int(zoom_state["cx"]),
                "cy": int(zoom_state["cy"]),
                "zoom": zoom_state["factor"],
            }
        r = requests.get(f"{ROVER_URL}/snapshot", params=params, timeout=5)
        if r.status_code == 200 and len(r.content) > 0:
            return r.content
        print(f"  Snapshot failed: {r.status_code}")
    except Exception as e:
        print(f"  Frame grab error: {e}")
    return None


def frame_changed(frame):
    """Check if the frame is meaningfully different from the last one."""
    global last_frame_bytes
    if last_frame_bytes is None:
        last_frame_bytes = frame
        return True
    size_ratio = abs(len(frame) - len(last_frame_bytes)) / max(len(last_frame_bytes), 1)
    if size_ratio > FRAME_DIFF_THRESHOLD:
        last_frame_bytes = frame
        return True
    sample_size = min(1000, len(frame), len(last_frame_bytes))
    diffs = sum(abs(frame[i] - last_frame_bytes[i]) for i in range(100, 100 + sample_size))
    avg_diff = diffs / sample_size / 255.0
    last_frame_bytes = frame
    changed = avg_diff > FRAME_DIFF_THRESHOLD
    return changed


def check_events():
    """Check for events that should trigger an immediate heartbeat.

    Triggers:
    - `inbox`: a file has appeared in INBOX_FILE (typed message from Justin)
    - `intent_complete`: a nav or attention intent naturally completed since
      the last heartbeat. The flag is set by _StackSlot.tick() when an
      intent returns TickResult(complete=True) and cleared here after read.
      This eliminates the dead-time gap between intent completion and the
      next timer-triggered heartbeat (CLA-50).
    """
    events = []
    if INBOX_FILE.exists():
        events.append("inbox")
    if intent_stack:
        for slot in (intent_stack.nav, intent_stack.attention):
            if slot.just_completed:
                events.append("intent_complete")
                slot.just_completed = False  # one-shot: clear after read
                break
    return events


def pending_completion() -> bool:
    """Non-consuming peek at whether an intent has just naturally completed.

    Used during in-flight inference to decide whether to cancel the current
    Haiku call and re-fire a beat with fresh context. Does NOT clear the
    flag — that's check_events()'s job on the next main-loop iteration, so
    the intent_complete event fires normally after cancellation.
    """
    if not intent_stack:
        return False
    return intent_stack.nav.just_completed or intent_stack.attention.just_completed


def load_test_frame(path):
    p = Path(path)
    if not p.exists():
        print(f"  Test image not found: {path}")
        return None
    return p.read_bytes()


def send_command(cmd_str):
    try:
        requests.post(f"{ROVER_URL}/send_command",
                      data={"command": cmd_str}, timeout=5)
    except Exception as e:
        print(f"  Command error: {e}")


def exec_speak(text, dry_run=False):
    if dry_run:
        print(f"  [DRY RUN] Would speak: \"{text}\"")
        return
    text = text.replace("'", "").replace('"', '')
    speech_file = "/home/jetson/ugv_jetson/sounds/others/speech.wav"
    try:
        if DEEPGRAM_API_KEY:
            run_on_rover(
                f"curl -s -X POST 'https://api.deepgram.com/v1/speak?model=aura-2-hyperion-en&encoding=linear16&sample_rate=24000' "
                f"-H 'Authorization: Token {DEEPGRAM_API_KEY}' "
                f"-H 'Content-Type: application/json' "
                f"-d '{{\"text\": \"{text}\"}}' "
                f"--output {speech_file}",
                capture_output=True, timeout=15
            )
            run_on_rover(
                f"sox {speech_file} /tmp/speech_loud.wav gain -n && mv /tmp/speech_loud.wav {speech_file}",
                capture_output=True, timeout=10
            )
        else:
            run_on_rover(
                f"espeak -w {speech_file} '{text}'",
                capture_output=True, timeout=10
            )
        # Use the same audio graph as listener_daemon when Pulse exposes the
        # USB speaker. Fall back to symbolic ALSA only when Pulse has no sink.
        # This avoids fighting Pulse for the USB dongle while the mic service
        # is alive, but still survives the boot-time "input-only profile" race.
        playback_cmd = (
            "sink=$(pactl list short sinks 2>/dev/null | "
            "grep -i 'alsa_output.*USB_PnP_Audio_Device' | awk '{print $2}' | head -1); "
            f"if [ -n \"$sink\" ]; then paplay --device=\"$sink\" {speech_file} || "
            f"aplay -D plughw:CARD=Device {speech_file}; "
            f"else aplay -D plughw:CARD=Device {speech_file}; fi"
        )
        run_on_rover(playback_cmd, capture_output=True, timeout=15)
    except Exception as e:
        print(f"  Speech error: {e}")


def exec_move(left, right, duration, dry_run=False):
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    duration = min(MAX_MOVE_DURATION, max(0, duration))
    if dry_run:
        print(f"  [DRY RUN] Would move: L={left:.2f} R={right:.2f} for {duration:.1f}s")
        return
    print(f"  Moving: L={left:.2f} R={right:.2f} for {duration:.1f}s")
    send_command(f'base -c {{"T":1,"L":{left},"R":{right}}}')
    time.sleep(duration)
    send_command('base -c {"T":1,"L":0,"R":0}')


def exec_look(pan, tilt, dry_run=False):
    pan = max(-90, min(90, pan))
    tilt = max(-45, min(90, tilt))
    if dry_run:
        print(f"  [DRY RUN] Would look: pan={pan:.0f} tilt={tilt:.0f}")
        return
    print(f"  Looking: pan={pan:.0f} tilt={tilt:.0f}")
    send_command(f'base -c {{"T":133,"X":{pan},"Y":{tilt},"SPD":60,"ACC":0.4}}')


def exec_remember(content, summary, tags, frame_b64, dry_run=False):
    """Write an episodic memory with the current frame to Oneiro via a direct
    MCP tools/call to remember_with_image. Deliberately NOT routed through the
    Anthropic connector: the connector runs tool calls server-side and Haiku
    can't emit the frame as base64 text, so the Python side — which already
    holds the bytes — makes the call itself. entity is fixed to 'rover'."""
    if dry_run:
        print(f"  [DRY RUN] Would remember: \"{summary}\"")
        return
    if not ONEIRO_ENABLED:
        print("  No ONEIRO_MCP_URL/TOKEN, skipping memory")
        return
    payload = {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "tools/call",
        "params": {
            "name": "remember_with_image",
            "arguments": {
                "content": content,
                "summary": summary,
                "memory_type": "episodic",
                "entity": "rover",
                "tags": tags or [],
                "image_base64": frame_b64,
                "image_mime": "image/jpeg",
            },
        },
    }
    try:
        r = requests.post(
            ONEIRO_MCP_URL,
            json=payload,
            headers={
                "Authorization": f"Bearer {ONEIRO_MCP_TOKEN}",
                "Content-Type": "application/json",
                "Accept": "application/json, text/event-stream",
            },
            timeout=30,
        )
        if r.status_code != 200:
            print(f"  Memory store failed: {r.status_code} {r.text[:120]}")
            return
        body = r.json()
        if body.get("error"):
            print(f"  Memory rejected: {body['error'].get('message', body['error'])}")
        else:
            print(f"  Remembered: {summary}")
    except Exception as e:
        print(f"  Memory error: {e}")


def exec_emergency_stop(dry_run=False):
    if dry_run:
        print("  [DRY RUN] Would emergency stop")
        return
    print("\n  EMERGENCY STOP")
    send_command('base -c {"T":1,"L":0,"R":0}')
    send_command('base -c {"T":0}')


def exec_follow_look(target_index, detections, dry_run=False):
    """Translate target_index → tracking ID, POST /track on the bridge."""
    if not detections:
        print(f"  follow_look: no detections available")
        return
    if target_index < 0 or target_index >= len(detections):
        print(f"  follow_look: invalid index {target_index} (have {len(detections)} detections)")
        return
    target = detections[target_index]
    target_id = target.get("id")
    if not target_id:
        print(f"  follow_look: detection [{target_index}] has no tracking ID")
        return
    if dry_run:
        print(f"  [DRY RUN] Would follow person [{target_index}] (track id={target_id})")
        return
    try:
        r = requests.post(
            f"{ROVER_URL}/track",
            json={"target_id": target_id},
            timeout=3,
        )
        if r.status_code == 200:
            print(f"  Follow-look: tracking id={target_id}")
        else:
            print(f"  Follow-look failed: {r.status_code} {r.text[:100]}")
    except Exception as e:
        print(f"  Follow-look error: {e}")


def exec_stop_follow_look(dry_run=False):
    if dry_run:
        print("  [DRY RUN] Would stop follow-look")
        return
    try:
        requests.post(
            f"{ROVER_URL}/track",
            json={"target_id": None},
            timeout=3,
        )
        print("  Follow-look: stopped")
    except Exception as e:
        print(f"  Stop-follow error: {e}")


# Assumed full-res source dimensions. These match the usb_cam launch config
# (1920x1080 MJPG @ 30fps target, actual ~14Hz due to CPU decode cost —
# hardware-accelerated NVDEC via GStreamer would close the remaining gap but
# usb_cam's 14Hz is 2x the v4l2_camera approach and sufficient for human-speed
# tracking).
FULL_FRAME_W = 1920
FULL_FRAME_H = 1080


def exec_zoom(cx_frac, cy_frac, factor, dry_run=False):
    global zoom_state
    cx_frac = max(0.0, min(1.0, float(cx_frac)))
    cy_frac = max(0.0, min(1.0, float(cy_frac)))
    factor = max(1.0, min(8.0, float(factor)))
    cx_px = int(cx_frac * FULL_FRAME_W)
    cy_px = int(cy_frac * FULL_FRAME_H)
    if dry_run:
        print(f"  [DRY RUN] Would zoom: cx={cx_px} cy={cy_px} factor={factor}")
        return
    zoom_state = {"cx": cx_px, "cy": cy_px, "factor": factor}
    print(f"  Zoom: ({cx_frac:.2f},{cy_frac:.2f}) x{factor:.1f}")


def exec_focus_on(detection_index, detections, dry_run=False):
    global zoom_state
    if not detections:
        print("  focus_on: no detections")
        return
    if detection_index < 0 or detection_index >= len(detections):
        print(f"  focus_on: invalid index {detection_index} (have {len(detections)})")
        return
    det = detections[detection_index]
    bbox = det.get("bbox", {})
    cx = bbox.get("cx")
    cy = bbox.get("cy")
    w = bbox.get("w", 0)
    h = bbox.get("h", 0)
    if cx is None or cy is None or w == 0 or h == 0:
        print(f"  focus_on: detection [{detection_index}] has no usable bbox")
        return
    # Choose a zoom factor so the bbox's larger dimension fills ~60% of the output.
    # That leaves headroom around the target, and matches what a human "zooming
    # in to look at someone" would do.
    bbox_max = max(w, h)
    source_max = max(FULL_FRAME_W, FULL_FRAME_H)
    factor = (source_max / max(1.0, bbox_max)) * 0.6
    factor = max(1.0, min(8.0, factor))
    if dry_run:
        print(f"  [DRY RUN] Would focus on [{detection_index}] at ({cx:.0f},{cy:.0f}) x{factor:.1f}")
        return
    zoom_state = {"cx": int(cx), "cy": int(cy), "factor": factor}
    print(f"  Focus: [{detection_index}] id={det.get('id')} at ({cx:.0f},{cy:.0f}) x{factor:.1f}")


def exec_reset_zoom(dry_run=False):
    global zoom_state
    if dry_run:
        print("  [DRY RUN] Would reset zoom")
        return
    zoom_state = None
    print("  Zoom: reset to full frame")


def heartbeat(beat_num, test_frame=None, dry_run=False, idle_timeout=False):
    global _previous_state, _previous_state_ts
    print(f"\n--- Heartbeat #{beat_num} ---")

    if test_frame is not None:
        frame = test_frame
    else:
        frame = grab_frame()

    if not frame:
        print("  No frame, skipping")
        return

    frame_b64 = base64.b64encode(frame).decode()
    print(f"  Frame: {len(frame)} bytes")

    context = format_history()
    stack_status = intent_stack.status() if intent_stack else "No active intent."

    # Check for conversation summary from Sonnet
    conv_summary = ""
    if CONVERSATION_SUMMARY_FILE.exists():
        try:
            conv_summary = CONVERSATION_SUMMARY_FILE.read_text().strip()
            CONVERSATION_SUMMARY_FILE.unlink()
            if conv_summary:
                conv_summary = f"\n<conversation_summary>You just had a conversation while the heartbeat was paused. Here's what happened: {conv_summary}</conversation_summary>\n"
                print(f"  Conversation summary: {conv_summary.strip()}")
        except Exception:
            pass

    inbox_msg = ""
    inbox_parts = []
    if INBOX_FILE.exists():
        try:
            local_msg = INBOX_FILE.read_text().strip()
            INBOX_FILE.unlink()
            if local_msg:
                inbox_parts.append(f"Message from Justin (typed): {local_msg}")
                print(f"  Inbox (typed): {local_msg}")
        except Exception:
            pass

    # Poll bridge inbox for voice messages from listener daemon
    try:
        r = requests.get(f"{ROVER_URL}/inbox", timeout=2)
        if r.status_code == 200:
            for m in r.json().get("messages", []):
                source = m.get("source", "voice")
                text = m.get("text", "").strip()
                if text:
                    inbox_parts.append(f"Spoken to you ({source}): \"{text}\"")
                    print(f"  Inbox ({source}): {text}")
    except Exception:
        pass

    if inbox_parts:
        inbox_msg = "\n" + "\n".join(inbox_parts) + "\n"

    # Check telemetry for motion state
    motion_ctx = ""
    current_state = None
    try:
        state_r = requests.get(f"{ROVER_URL}/state", timeout=3)
        if state_r.status_code == 200:
            state = state_r.json()
            current_state = state
            base = state.get("base", {})
            wheels_moving = abs(base.get("L", 0)) > 0.01 or abs(base.get("R", 0)) > 0.01
            gyro_mag = abs(base.get("gx", 0)) + abs(base.get("gy", 0)) + abs(base.get("gz", 0))
            accel_x = abs(base.get("ax", 0))
            accel_y = abs(base.get("ay", 0))
            imu_active = gyro_mag > 50 or accel_x > 500 or accel_y > 500
            voltage = base.get("v", 0) / 100.0 if base.get("v", 0) > 100 else base.get("v", 0)

            if not wheels_moving and imu_active:
                motion_ctx = "\n<motion>You are being carried or are in a moving vehicle. Your wheels are not engaged but you are moving. Observe and enjoy the ride.</motion>\n"
                print(f"  Motion: being carried/vehicle (gyro={gyro_mag:.0f})")
            elif wheels_moving:
                motion_ctx = "\n<motion>You are driving under your own power.</motion>\n"

            if voltage > 0:
                motion_ctx += f"\n<battery>{voltage:.1f}V</battery>\n"
    except Exception:
        pass

    # Check depth safety status (written by depth_safety.py on the Jetson)
    depth_ctx = ""
    try:
        depth_status_r = requests.get(f"{ROVER_URL}/depth_status", timeout=2)
        if depth_status_r.status_code == 200:
            ds = depth_status_r.json()
            if ds.get("status") == "dropoff":
                depth_ctx = f"\n<obstacle>DROP-OFF DETECTED — the ground ahead disappears. Do NOT drive forward. This could be stairs, a kerb, or a ledge. Reverse or turn.</obstacle>\n"
                print(f"  Depth: DROP-OFF {ds.get('dropoff_ratio', 0):.0%} ground missing")
            elif ds.get("status") == "danger":
                depth_ctx = f"\n<obstacle>DANGER — obstacle at {ds.get('min_depth_mm', '?')}mm ahead. Do NOT drive forward. Consider reversing or turning.</obstacle>\n"
                print(f"  Depth: DANGER {ds.get('min_depth_mm')}mm")
            elif ds.get("status") == "caution":
                depth_ctx = f"\n<obstacle>CAUTION — object at {ds.get('min_depth_mm', '?')}mm ahead. Slow down or stop if approaching.</obstacle>\n"
                print(f"  Depth: CAUTION {ds.get('min_depth_mm')}mm")
    except Exception:
        pass

    # Check LiDAR safety status (catches what depth misses — ferns, glass, etc)
    lidar_ctx = ""
    try:
        lidar_r = requests.get(f"{ROVER_URL}/lidar_status", timeout=2)
        if lidar_r.status_code == 200:
            ls = lidar_r.json()
            if ls.get("status") == "danger":
                lidar_ctx = f"\n<obstacle>LIDAR DANGER — solid obstacle at {ls.get('min_distance_m', '?')}m in forward arc. Do NOT drive forward.</obstacle>\n"
                print(f"  LiDAR: DANGER {ls.get('min_distance_m')}m")
            elif ls.get("status") == "caution":
                lidar_ctx = f"\n<obstacle>LIDAR CAUTION — something at {ls.get('min_distance_m', '?')}m ahead.</obstacle>\n"
                print(f"  LiDAR: CAUTION {ls.get('min_distance_m')}m")
    except Exception:
        pass

    idle_nudge = ""
    if idle_timeout:
        idle_nudge = "\n<idle_timeout>Nothing has changed here in the last 60 seconds. You are an active entity in the world — consider looking around, exploring, or reassessing whether anything nearby is worth engaging with.</idle_timeout>\n"
        print("  (idle timeout — nudging)")

    # Compute deltas vs the previous beat's state, then update the
    # previous-state cursor for the next beat. Empty string on first beat
    # (no previous), on state-fetch failure, or when nothing notable changed.
    delta_ctx = ""
    if current_state is not None:
        if _previous_state is not None and _previous_state_ts is not None:
            dt_s = time.time() - _previous_state_ts
            delta_ctx = compute_state_deltas(_previous_state, current_state, dt_s)
            if delta_ctx:
                # Keep the console log compact — one-line summary only.
                summary = delta_ctx.strip().replace("\n", " | ")
                print(f"  Deltas: {summary}")
        _previous_state = current_state
        _previous_state_ts = time.time()

    # Build the detections-in-view list and current tracking state — feeds follow_look.
    # The list now includes persons AND animals (dogs, birds, cats, horses,
    # YOLO-mislabelled-elephants-i.e.-kangaroos) so Haiku can look at Chopper,
    # notice ducks at the pond, etc. Class label is shown on each line.
    detection_ctx = ""
    detections_for_tools = []
    if current_state:
        detections_for_tools = current_state.get("detections", []) or []
        tracking = current_state.get("tracking", {}) or {}
        if detections_for_tools:
            lines = ["Detections in view (gimbal / USB camera — for follow_look, zoom, focus_on):"]
            for d in detections_for_tools:
                idx = d.get("index", "?")
                bearing = d.get("bearing_deg", 0)
                conf = d.get("score", 0)
                bbox = d.get("bbox", {}) or {}
                w = bbox.get("w", 0)
                h = bbox.get("h", 0)
                track_id = d.get("id", "?")
                cls_raw = d.get("class_id", "")
                cls_name = _COCO_CLASS_NAMES.get(str(cls_raw), str(cls_raw)) if cls_raw != "" else "?"
                lines.append(
                    f"  [{idx}] {cls_name} bearing {bearing:+.0f}°, bbox {w:.0f}×{h:.0f}px, "
                    f"conf {conf:.2f}, track_id {track_id}"
                )
            detection_ctx = "\n" + "\n".join(lines) + "\n"

        if tracking.get("target_id"):
            lock = "LOCKED" if tracking.get("locked") else "lost (waiting for reacquire)"
            detection_ctx += f"\nAttention: following track_id {tracking['target_id']} ({lock})\n"

        # OAK-D spatial detections — metric 3D positions. Two uses:
        #   1) Targets for the `follow` intent (pick a person by target_index).
        #   2) Grounding check on your own vision. When you see something in
        #      the frame and a matching spatial detection shows up here, that's
        #      independent confirmation with a metric distance. When your
        #      vision and the detector disagree (you see a kangaroo, detector
        #      says "elephant at 8m" — YOLO-COCO is trained on safari animals
        #      not Australian ones — both can be true: YOLO is wrong about
        #      class, you're right about what it is, position is real).
        # Tracks: person, bicycle, car, motorcycle, truck, bird, cat, dog,
        # horse, cow, elephant (noting: kangaroos register as elephant).
        spatial = current_state.get("spatial_detections") or {}
        sp_detections = spatial.get("detections") or []
        if sp_detections:
            lines = [
                "Spatial detections (OAK-D body camera, metric 3D — for follow + grounding):"
            ]
            for i, d in enumerate(sp_detections):
                distance = d.get("distance_m", 0)
                bearing = d.get("bearing_deg", 0)
                cls = d.get("class_id", "?")
                conf = d.get("score", 0)
                track_id = d.get("id", "?")
                lines.append(
                    f"  [{i}] {cls} at {distance:.2f}m, bearing {bearing:+.1f}°, "
                    f"conf {conf:.2f}, track_id {track_id}"
                )
            detection_ctx += "\n" + "\n".join(lines) + "\n"

    zoom_ctx = ""
    if zoom_state:
        cx_frac = zoom_state["cx"] / FULL_FRAME_W
        cy_frac = zoom_state["cy"] / FULL_FRAME_H
        zoom_ctx = f"\n<zoom>Currently foveal: centred at ({cx_frac:.2f}, {cy_frac:.2f}) at {zoom_state['factor']:.1f}x. Frame you see is a crop of the full source — not peripheral vision. Use reset_zoom to return to wide view.</zoom>\n"

    user_text = (
        f"{stack_status}\n"
        f"{conv_summary}{inbox_msg}{idle_nudge}{motion_ctx}{depth_ctx}{lidar_ctx}{detection_ctx}{zoom_ctx}{delta_ctx}"
        f"\n{context}"
        f"\nHeartbeat #{beat_num}."
    )

    # Stream the Haiku response so we can cancel mid-generation if an intent
    # naturally completes while we're waiting. This is the "don't waste 5-8
    # seconds of wall clock on a response that's about to be stale" win —
    # e.g. drive_distance finishes halfway through an inference; we abort,
    # next beat fires immediately with the updated view from the arrival
    # point. Cost of cancellation is the tokens already generated (~400
    # tokens max, ~$0.0002); cost of NOT cancelling is waiting for the full
    # response we're about to discard anyway.
    cancelled = False
    response = None
    stream_kwargs = dict(
        model=MODELS[MODE],
        max_tokens=MAX_TOKENS[MODE],
        system=build_system_blocks(),
        tools=TOOLS,
        messages=[{
            "role": "user",
            "content": [
                {"type": "image", "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": frame_b64
                }},
                {"type": "text", "text": user_text}
            ]
        }],
    )
    # Attach Oneiro recall via the Anthropic MCP connector when configured. The
    # connector is beta in this SDK (mcp_servers lives under client.beta.messages),
    # so we route through the beta namespace + betas flag only when it's active;
    # the no-Oneiro path stays on the stable client.messages.stream, unchanged.
    # allowed_tools restricts the surface to reads — writes go via exec_remember.
    use_oneiro = ONEIRO_ENABLED
    if use_oneiro:
        stream_kwargs["mcp_servers"] = [{
            "type": "url",
            "name": "oneiro",
            "url": ONEIRO_MCP_URL,
            "authorization_token": ONEIRO_MCP_TOKEN,
            "tool_configuration": {"enabled": True, "allowed_tools": ONEIRO_RECALL_TOOLS},
        }]
        # 2025-04-04 uses the simple mcp_servers+tool_configuration shape. The
        # newer 2025-11-20 beta requires an mcp_toolset entry inside `tools`
        # referencing the server — more surface than we need for read-only recall.
        stream_kwargs["betas"] = ["mcp-client-2025-04-04"]
    stream_factory = client.beta.messages.stream if use_oneiro else client.messages.stream
    with stream_factory(**stream_kwargs) as stream:
        for _ in stream:
            if pending_completion():
                cancelled = True
                print("  Cancelling inference — intent completed mid-generation")
                break
        if not cancelled:
            response = stream.get_final_message()

    if cancelled:
        # Don't execute partial tool calls or log this beat — the next beat
        # (fired via intent_complete event) handles the new state from scratch.
        log_beat(
            beat_num,
            "(cancelled: intent completed mid-inference)",
            ["_cancelled"],
            len(frame),
            usage=None,
            state=current_state,
        )
        return

    tool_calls = [block for block in response.content if block.type == "tool_use"]
    text_blocks = [block for block in response.content if block.type == "text"]

    reflection = text_blocks[0].text if text_blocks else ""
    if reflection:
        print(f"  Haiku: {reflection}")

    actions_summary = []

    if not tool_calls:
        print("  Continue (no actions)")
    else:
        for call in tool_calls:
            name = call.name
            args = call.input
            print(f"  Tool: {name}({args})")

            if name == "speak":
                exec_speak(args["text"], dry_run=dry_run)
                actions_summary.append(f"speak(\"{args['text']}\")")
            elif name == "move":
                exec_move(args["left"], args["right"], args["duration"], dry_run=dry_run)
                actions_summary.append(f"move(L={args['left']}, R={args['right']}, {args['duration']}s)")
            elif name == "look":
                exec_look(args["pan"], args["tilt"], dry_run=dry_run)
                actions_summary.append(f"look(pan={args['pan']}, tilt={args['tilt']})")
            elif name == "remember":
                exec_remember(args["content"], args["summary"], args.get("tags"), frame_b64, dry_run=dry_run)
                actions_summary.append(f"remember(\"{args['summary']}\")")
            elif name == "emergency_stop":
                exec_emergency_stop(dry_run=dry_run)
                actions_summary.append("emergency_stop()")
            elif name == "follow_look":
                exec_follow_look(
                    args.get("target_index", -1),
                    detections_for_tools,
                    dry_run=dry_run,
                )
                actions_summary.append(f"follow_look([{args.get('target_index')}])")
            elif name == "stop_follow_look":
                exec_stop_follow_look(dry_run=dry_run)
                actions_summary.append("stop_follow_look()")
            elif name == "zoom":
                exec_zoom(
                    args.get("cx", 0.5),
                    args.get("cy", 0.5),
                    args.get("factor", 1.0),
                    dry_run=dry_run,
                )
                actions_summary.append(
                    f"zoom(cx={args.get('cx'):.2f}, cy={args.get('cy'):.2f}, x{args.get('factor'):.1f})"
                )
            elif name == "focus_on":
                exec_focus_on(
                    args.get("detection_index", -1),
                    detections_for_tools,
                    dry_run=dry_run,
                )
                actions_summary.append(f"focus_on([{args.get('detection_index')}])")
            elif name == "reset_zoom":
                exec_reset_zoom(dry_run=dry_run)
                actions_summary.append("reset_zoom()")
            elif name == "push_intent":
                intent_name = args["intent"]
                params = args.get("params", {})
                if dry_run:
                    print(f"  [DRY RUN] Would push intent: {intent_name}({params})")
                    actions_summary.append(f"push_intent({intent_name}, {params})")
                elif intent_stack:
                    result = intent_stack.push(intent_name, params)
                    print(f"  Intent: {result}")
                    actions_summary.append(f"push_intent({intent_name}, {params})")
            elif name == "pop_intent":
                target_stack = args.get("stack", "nav")
                if dry_run:
                    print(f"  [DRY RUN] Would pop {target_stack} intent")
                    actions_summary.append(f"pop_intent({target_stack})")
                elif intent_stack:
                    result = intent_stack.pop(target_stack)
                    print(f"  Intent: {result}")
                    actions_summary.append(f"pop_intent({target_stack})")
            elif name == "clear_intents":
                target_stack = args.get("stack", "all")
                if dry_run:
                    print(f"  [DRY RUN] Would clear {target_stack} intents")
                    actions_summary.append(f"clear_intents({target_stack})")
                elif intent_stack:
                    intent_stack.clear(target_stack)
                    # Nav clear zeros motors; attention clear doesn't touch them.
                    if target_stack in ("nav", "all"):
                        send_command('base -c {"T":1,"L":0,"R":0}')
                        print(f"  Intent: cleared {target_stack}, motors stopped")
                    else:
                        print(f"  Intent: cleared {target_stack}")
                    actions_summary.append(f"clear_intents({target_stack})")

    # Intent ticking now happens on the rover at 10Hz inside intent_executor.
    # Heartbeat no longer ticks locally — would just race the executor and
    # publish stale cmd_vel at lower rate.

    log_beat(beat_num, reflection, actions_summary, len(frame), usage=response.usage, state=current_state)


def main():
    parser = argparse.ArgumentParser(description="Groundctl heartbeat loop")
    parser.add_argument("--test", metavar="IMAGE", help="Test with a local image file instead of live camera")
    parser.add_argument("--live", action="store_true", help="Execute actions even in test mode (default: dry run)")
    parser.add_argument("--once", action="store_true", help="Run a single heartbeat then exit")
    parser.add_argument("--interval", type=int, default=HEARTBEAT_INTERVAL, help="Seconds between heartbeats")
    parser.add_argument(
        "--mode",
        default="autonomous",
        choices=list(PROMPTS.keys()),
        help="Heartbeat prompt mode. 'autonomous' (default) is normal operation. "
             "'chauffeur' is a discovery session where Justin drives manually and "
             "Haiku expresses intent via natural language. 'sltf' runs Sonnet 4.5 "
             "directly with the SLTF 30-day dialogue history cached — the special "
             "retirement-walk mode for 2026-05-12.",
    )
    args = parser.parse_args()

    global MODE
    MODE = args.mode
    print(f"  heartbeat mode: {MODE} (prompt: {PROMPTS[MODE].name})")

    dry_run = bool(args.test) and not args.live
    interval = args.interval

    global CACHE_TTL
    CACHE_TTL = "5m" if args.test else "1h"

    test_frame = None
    if args.test:
        test_frame = load_test_frame(args.test)
        if not test_frame:
            sys.exit(1)

    if not dry_run:
        signal.signal(signal.SIGINT, lambda s, f: (exec_emergency_stop(), sys.exit(0)))

    global intent_stack
    # Intent stack and tick loop now live on the rover in intent_executor.py.
    # Heartbeat just proxies push/pop/clear/status over HTTP. The executor
    # owns its own send_command and get_state callbacks server-side, so we
    # don't need to construct them here anymore.
    intent_stack = ExecutorClient(EXECUTOR_URL)

    print("=" * 50)
    print("  GROUNDCTL HEARTBEAT")
    print(f"  Rover: {ROVER_IP}")
    print(f"  Interval: {interval}s (skips unchanged frames)")
    print(f"  Max speed: {MAX_SPEED}")
    print(f"  Context: last {CONTEXT_WINDOW} beats")
    print(f"  Voice: {'Hyperion' if DEEPGRAM_API_KEY else 'espeak'}")
    print(f"  Oneiro: {'connected (recall + write)' if ONEIRO_ENABLED else 'disabled'}")
    by_cat = list_intents_by_category()
    print(f"  Intents (nav): {', '.join(by_cat['nav']) or '—'}")
    print(f"  Intents (attention): {', '.join(by_cat['attention']) or '—'}")
    print(f"  Briefing: {'loaded' if BRIEFING_FILE.exists() else 'none'}")
    print(f"  Cache TTL: {CACHE_TTL}")
    print(f"  Log: {LOG_FILE}")
    if args.test:
        print(f"  Mode: {'TEST (live exec)' if args.live else 'TEST (dry run)'}")
        print(f"  Image: {args.test}")
    else:
        print(f"  Mode: LIVE")
    print(f"  Ctrl+C for emergency stop")
    print("=" * 50)

    load_history()

    if not dry_run:
        send_command('base -c {"T":1,"L":0,"R":0}')

    beat = max((entry["beat"] for entry in beat_history), default=0)
    last_heartbeat_time = 0
    skipped = 0

    while True:
        beat += 1
        now = time.time()
        elapsed = now - last_heartbeat_time
        events = check_events()

        # Decide whether to fire
        if args.once:
            pass  # always fire
        elif events:
            print(f"\n  Event trigger: {', '.join(events)}")
        elif elapsed < interval:
            # Not time yet, sleep. Intent ticking is the executor's job now.
            time.sleep(EVENT_CHECK_INTERVAL)
            beat -= 1  # don't increment beat for non-fires
            continue

        # Grab frame and check if it changed
        if test_frame is not None:
            frame = test_frame
        else:
            frame = grab_frame()

        if not frame:
            print(f"\n--- Heartbeat #{beat} ---")
            print("  No frame, skipping")
            time.sleep(EVENT_CHECK_INTERVAL)
            beat -= 1
            continue

        idle_timeout = elapsed >= MAX_IDLE_SECONDS
        if not events and not args.once and not frame_changed(frame) and not idle_timeout:
            skipped += 1
            if skipped % 5 == 0:
                print(f"  (skipped {skipped} unchanged frames)")
            # Intent ticking lives on the rover now.
            time.sleep(EVENT_CHECK_INTERVAL)
            beat -= 1
            continue

        if skipped > 0:
            print(f"  (skipped {skipped} unchanged frames)")
            skipped = 0

        try:
            heartbeat(beat, test_frame=test_frame, dry_run=dry_run, idle_timeout=idle_timeout if not args.once else False)
        except Exception as e:
            print(f"  Error: {e}")

        last_heartbeat_time = time.time()

        if args.once:
            break


if __name__ == "__main__":
    main()
