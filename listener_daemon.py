#!/usr/bin/env python3
"""
listener_daemon.py — Always-on speech listener using Deepgram Nova-2.

Captures from the default PulseAudio source, streams audio to Deepgram's
streaming STT over a websocket, detects the wake word in final
transcriptions, and posts the utterance to the bridge's /inbox endpoint.

Nova-2 gets ~5-8% WER on conversational English and handles Australian
accents and proper nouns (kangaroos, place names, neighbour names) that
local Whisper tiny turned into "tango rules". Streaming latency is
~300-500ms end-to-end. Cost is ~$0.0043/min while actively transcribing,
which is negligible for the rover's use case.

Env:
    DEEPGRAM_API_KEY  — required
    PULSE_SOURCE      — optional; pins capture to a specific pulse source
                        (see earlier diagnosis: default source keeps
                        reverting to onboard audio input on this Jetson)

Usage:
    python listener_daemon.py
    python listener_daemon.py --language en-AU
    python listener_daemon.py --wake claude "hey claude" "oi claude"
"""

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlencode

import pyaudio
import requests
import websocket

sys.stdout.reconfigure(line_buffering=True)

DEFAULT_WAKE_WORDS = [
    "claude", "hey claude", "oi claude",
    "clawed", "hey clawed", "oi clawed",
    "claud", "hey claud", "oi claud",
    "cooled", "hey cooled", "oi cooled",
    "clod", "hey clod", "oi clod",
    "rover", "hey rover", "oi rover",
]
BRIDGE_URL = "http://localhost:5000"
PIDFILE = Path("/tmp/listener_daemon.pid")

# Capture format matches what we send to Deepgram. 16 kHz mono PCM is the
# industry-standard low-bandwidth choice; Nova-2 is trained for it and it
# halves the upstream cost vs 48 kHz.
CAPTURE_RATE = 16000
CAPTURE_CHANNELS = 1
CAPTURE_FORMAT = pyaudio.paInt16
CHUNK_MS = 60
CHUNK_FRAMES = int(CAPTURE_RATE * CHUNK_MS / 1000)


def check_singleton():
    if PIDFILE.exists():
        try:
            old_pid = int(PIDFILE.read_text().strip())
            os.kill(old_pid, 0)
            print(f"Another listener is already running (PID {old_pid}). Exiting.")
            sys.exit(1)
        except (OSError, ValueError):
            pass
    PIDFILE.write_text(str(os.getpid()))


def find_input_device(pa, name_hint):
    """Find a pyaudio input device by substring match.

    name_hint may be a single string (legacy) or a comma-separated list of
    fallback substrings tried in order. The fallback form is preferred so
    boot-time pulse profile lottery (USB PnP loaded as output-only) doesn't
    take down the listener — first hit on the USB ALSA device, fall back
    to pulse only if direct ALSA isn't enumerated.
    """
    hints = [h.strip() for h in name_hint.split(",") if h.strip()]
    for hint in hints:
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if info["maxInputChannels"] > 0 and hint.lower() in info["name"].lower():
                return i, info
    return None, None


def is_wake_word(text, wake_words):
    low = text.lower().strip()
    return any(w in low for w in wake_words)


def post_to_bridge(bridge_url, text):
    try:
        r = requests.post(
            f"{bridge_url}/inbox",
            json={"text": text, "source": "voice"},
            timeout=3,
        )
        if r.status_code == 200:
            print(f"  → inbox: \"{text}\"")
        else:
            print(f"  → inbox failed: {r.status_code}")
    except Exception as e:
        print(f"  → inbox error: {e}")


def build_ws_url(language, sample_rate=CAPTURE_RATE):
    params = {
        "model": "nova-2",
        "language": language,
        "encoding": "linear16",
        "sample_rate": sample_rate,
        "channels": CAPTURE_CHANNELS,
        "smart_format": "true",
        # interim_results + utterance_end_ms is Deepgram's pattern for
        # "keep going past emphasis pauses, only commit when the speaker
        # actually stops talking". Without it, a 400ms mid-sentence pause
        # would finalise the transcript early — e.g. "hey Claude [pause]
        # go see the kangaroos" gets split into two finals and the second
        # (which carries the actual intent) misses the wake word.
        "interim_results": "true",
        "endpointing": "400",
        "utterance_end_ms": "1200",
    }
    return f"wss://api.deepgram.com/v1/listen?{urlencode(params)}"


def main():
    parser = argparse.ArgumentParser(description="Deepgram Nova-2 speech listener")
    parser.add_argument("--wake", nargs="+", default=DEFAULT_WAKE_WORDS, help="Wake words")
    parser.add_argument("--bridge", default=BRIDGE_URL, help="Bridge URL")
    parser.add_argument(
        "--device",
        default="USB PnP Audio,USB Audio,pulse",
        help="Comma-separated substrings of input device name, tried in order. "
             "Defaults prefer the USB PnP ALSA device directly so a boot where "
             "pulse misclassifies the USB card as output-only doesn't kill capture. "
             "Pulse is the last-resort fallback.",
    )
    parser.add_argument("--device-index", type=int, default=None, help="Explicit device index")
    parser.add_argument("--language", default="en", help="Deepgram language code (en, en-AU, en-US, etc.)")
    args = parser.parse_args()

    api_key = os.environ.get("DEEPGRAM_API_KEY")
    if not api_key:
        print("[listener] DEEPGRAM_API_KEY not set in environment", file=sys.stderr)
        sys.exit(1)

    check_singleton()

    print("=" * 50)
    print("  LISTENER DAEMON (Deepgram Nova-2)")
    print(f"  Wake words: {args.wake}")
    print(f"  Language: {args.language}")
    print(f"  Bridge: {args.bridge}")
    print("=" * 50)

    pa = pyaudio.PyAudio()
    try:
        if args.device_index is not None:
            device_index = args.device_index
            info = pa.get_device_info_by_index(device_index)
        else:
            device_index, info = find_input_device(pa, args.device)
            if device_index is None:
                print(f"[listener] No input device matching '{args.device}'. Run with --device-index.")
                sys.exit(1)
        native_rate = int(info.get("defaultSampleRate") or CAPTURE_RATE)
        print(f"[listener] Mic: {info['name']} (device {device_index}, {native_rate}Hz native)")

        # Try the listener's preferred rate first (16k — Nova-2 friendly).
        # USB Audio Class devices via raw ALSA `hw:` can refuse arbitrary
        # rates (no auto-resample) — when pulse mediates we get free
        # resampling, but direct ALSA hits the device's actual supported
        # rate list. Fall back to the device's native rate; Deepgram
        # handles resampling server-side.
        open_rate = CAPTURE_RATE
        try:
            stream = pa.open(
                format=CAPTURE_FORMAT,
                channels=CAPTURE_CHANNELS,
                rate=open_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK_FRAMES,
            )
        except OSError as e:
            if native_rate and native_rate != CAPTURE_RATE:
                print(f"[listener] {CAPTURE_RATE}Hz rejected ({e}); falling back to native {native_rate}Hz")
                open_rate = native_rate
                stream = pa.open(
                    format=CAPTURE_FORMAT,
                    channels=CAPTURE_CHANNELS,
                    rate=open_rate,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=int(open_rate * CHUNK_MS / 1000),
                )
            else:
                raise
        # The rate we actually opened at goes to Deepgram so server-side
        # resampling matches what we're sending.
        actual_capture_rate = open_rate
    except Exception:
        PIDFILE.unlink(missing_ok=True)
        raise

    # Websocket lifecycle: run_forever drives the event loop on the main
    # thread; a worker thread reads the mic and sends audio frames. The
    # worker exits when the ws closes (send raises). Graceful: on SIGINT,
    # we close the ws and the worker unblocks out of stream.read.
    mic_thread = {"handle": None}
    ws_holder = {"handle": None}

    def on_open(ws):
        print("[listener] Deepgram connected — listening")

        def pump():
            try:
                while True:
                    data = stream.read(CHUNK_FRAMES, exception_on_overflow=False)
                    ws.send(data, opcode=websocket.ABNF.OPCODE_BINARY)
            except Exception as e:
                # Normal on close — suppress the "socket is closed" spam.
                if "closed" not in str(e).lower():
                    print(f"[listener] audio pump exited: {e}")

        t = threading.Thread(target=pump, daemon=True)
        t.start()
        mic_thread["handle"] = t

    # Accumulator for final transcript fragments within a single spoken
    # utterance. Deepgram may emit several is_final=true Results per
    # utterance (one per mid-sentence pause > endpointing ms). We hold
    # them here and only commit when UtteranceEnd confirms the speaker
    # has truly stopped.
    pending_finals: list[str] = []

    def on_message(ws, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return

        mtype = data.get("type")
        if mtype == "Results":
            try:
                alt = data["channel"]["alternatives"][0]
            except (KeyError, IndexError):
                return
            text = (alt.get("transcript") or "").strip()
            is_final = data.get("is_final", False)
            if text and is_final:
                pending_finals.append(text)
        elif mtype == "UtteranceEnd":
            if pending_finals:
                full = " ".join(pending_finals).strip()
                pending_finals.clear()
                print(f"[listener] Heard: \"{full}\"")
                if is_wake_word(full, args.wake):
                    print("[listener] Wake word detected!")
                    post_to_bridge(args.bridge, full)

    def on_error(ws, err):
        print(f"[listener] WebSocket error: {err}")

    def on_close(ws, code, msg):
        print(f"[listener] WebSocket closed ({code}): {msg}")

    ws = websocket.WebSocketApp(
        build_ws_url(args.language, sample_rate=actual_capture_rate),
        header={"Authorization": f"Token {api_key}"},
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws_holder["handle"] = ws

    try:
        # reconnect=5: on transport-level close, websocket-client waits 5s
        # and re-runs the handshake. Deepgram drops idle connections after
        # ~1min; this keeps us alive across a hotspot handoff too.
        ws.run_forever(reconnect=5, ping_interval=20, ping_timeout=10)
    except KeyboardInterrupt:
        print("\n[listener] Shutting down")
    finally:
        try:
            ws.send(json.dumps({"type": "CloseStream"}))
        except Exception:
            pass
        try:
            ws.close()
        except Exception:
            pass
        try:
            stream.stop_stream()
            stream.close()
        except Exception:
            pass
        pa.terminate()
        PIDFILE.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
