#!/usr/bin/env python3
"""
control_panel.py — phone-friendly HTTP control panel for the rover.

Runs on the rover host (NOT in the container) so it can manage the container
itself and host-side systemd-user services. Reachable from phone over
Tailscale at http://rover:5060/. The whole point: stop needing a laptop and
SSH for routine ops (restart the stack, toggle bench mode, start/stop the
heartbeat, glance at the reflection log).

Buttons:
- Restart ROS2          POST /restart_ros       (calls start_ros2_local.sh)
- Bench Mode            POST /bench_mode        (--bench-mode: no Deepgram listener)
- Heartbeat Status      GET  /heartbeat
- Start Heartbeat       POST /heartbeat/start
- Stop Heartbeat        POST /heartbeat/stop
- Reflection Log        GET  /reflection_log    (last N lines)

Bench-mode caveat: currently --bench-mode in start_ros2.sh disables only
listener_daemon (Deepgram billable). When heartbeat moves to the rover as a
systemd service, "bench mode" should also stop heartbeat to silence the
Anthropic billable. For now, use Stop Heartbeat alongside Bench Mode for a
fully no-billing state.

Tailscale-only by deployment convention (no internet exposure), so no auth.
Same security model as ros2_bridge.

Run as a systemd-user service so it survives SSH disconnects (see
etc/control_panel.service).
"""

import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

from flask import Flask, jsonify, request, Response

# Path to the local bring-up script. Doesn't exist yet — created when
# start_ros2.sh is converted to a rover-local variant (task #50). The button
# returns a clear error if it's missing instead of silently failing.
START_SCRIPT = "/home/jetson/start_ros2_local.sh"

# systemd-user service name. Doesn't exist yet either — created in task #51
# when heartbeat is moved from the Mac to the rover.
HEARTBEAT_SERVICE = "heartbeat"
# SLTF retirement-walk mode — special-edition unit running Sonnet 4.5 with
# the 30-day cached dialogue history. Temporary for 2026-05-12; remove this
# constant and the /heartbeat/sltf/* endpoints post-walk.
SLTF_HEARTBEAT_SERVICE = "sltf_heartbeat"

# Reflection log path. heartbeat.py writes one JSON-line per beat to
# ~/.groundctl/heartbeat.jsonl with reflection + actions + state + tokens.
# Override with REFLECTION_LOG env var if running outside default user.
REFLECTION_LOG = Path(os.environ.get("REFLECTION_LOG", "/home/jetson/.groundctl/heartbeat.jsonl"))

PORT = int(os.environ.get("CONTROL_PANEL_PORT", "5060"))


app = Flask(__name__)


HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Rover Control</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { font-family: -apple-system, BlinkMacSystemFont, system-ui, sans-serif;
         margin: 0; padding: 16px; background: #0d0d0d; color: #eaeaea; }
  h1 { font-size: 18px; margin: 0 0 12px; font-weight: 500; letter-spacing: 0.02em; }
  button { display: block; width: 100%; padding: 16px; margin: 8px 0;
           font-size: 16px; font-weight: 500;
           border: 1px solid #333; background: #1a1a1a; color: #eaeaea;
           border-radius: 8px; cursor: pointer;
           -webkit-tap-highlight-color: transparent; }
  button:active { background: #2a2a2a; transform: scale(0.99); }
  button.primary { border-color: #2a5; background: #142a1f; }
  button.warn    { border-color: #a72; background: #2a1f10; }
  button.stop    { border-color: #a33; background: #2a1414; }
  .status { font-family: ui-monospace, Menlo, monospace; font-size: 12px;
            margin: 12px 0; padding: 10px;
            background: #141414; border: 1px solid #2a2a2a; border-radius: 6px;
            min-height: 1em; white-space: pre-wrap; word-break: break-all; }
  pre.log { font-family: ui-monospace, Menlo, monospace; font-size: 11px;
            max-height: 50vh; overflow: auto;
            background: #141414; border: 1px solid #2a2a2a;
            border-radius: 6px; padding: 8px; white-space: pre-wrap; }
  .group { margin: 16px 0 8px; font-size: 11px; color: #888;
           text-transform: uppercase; letter-spacing: 0.08em; }
</style>
</head>
<body>
<h1>Rover Control</h1>
<div id="status" class="status">ready</div>

<div class="group">Stack</div>
<button class="primary" onclick="api('/restart_ros', 'POST')">Restart ROS2</button>
<button class="warn"    onclick="api('/bench_mode',  'POST')">Bench Mode</button>

<div class="group">Heartbeat</div>
<button onclick="api('/heartbeat',       'GET')">Heartbeat Status</button>
<button class="primary" onclick="api('/heartbeat/start', 'POST')">Start Heartbeat</button>
<button class="stop"    onclick="api('/heartbeat/stop',  'POST')">Stop Heartbeat</button>

<div class="group">Logs</div>
<button onclick="loadLog()">Reflection Log</button>
<pre id="log" class="log"></pre>

<script>
async function api(path, method) {
  const s = document.getElementById('status');
  s.textContent = method + ' ' + path + ' …';
  try {
    const r = await fetch(path, { method });
    const j = await r.json();
    s.textContent = JSON.stringify(j, null, 2);
  } catch (e) {
    s.textContent = 'error: ' + e;
  }
}
async function loadLog() {
  const log = document.getElementById('log');
  log.textContent = 'loading…';
  try {
    const r = await fetch('/reflection_log');
    const j = await r.json();
    if (j.error) { log.textContent = 'error: ' + j.error; return; }
    log.textContent = (j.lines || []).join('') || '(empty)';
  } catch (e) {
    log.textContent = 'error: ' + e;
  }
}
</script>
</body>
</html>
"""


@app.route('/')
def index():
    return Response(HTML, mimetype='text/html')


@app.route('/restart_ros', methods=['POST'])
def restart_ros():
    if not os.path.isfile(START_SCRIPT):
        return jsonify({"status": "error",
                        "error": f"{START_SCRIPT} not found yet — task #50"}), 503
    # Fire-and-forget: bring-up takes ~60s, don't make the phone wait.
    subprocess.Popen(
        [START_SCRIPT],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return jsonify({"status": "started", "script": START_SCRIPT})


@app.route('/bench_mode', methods=['POST'])
def bench_mode():
    if not os.path.isfile(START_SCRIPT):
        return jsonify({"status": "error",
                        "error": f"{START_SCRIPT} not found yet — task #50"}), 503
    subprocess.Popen(
        [START_SCRIPT, '--bench-mode'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return jsonify({"status": "started",
                    "script": START_SCRIPT, "args": ["--bench-mode"]})


def _systemctl_user(verb: str, service: str):
    """Run `systemctl --user <verb> <service>` and return (rc, stdout, stderr)."""
    result = subprocess.run(
        ['systemctl', '--user', verb, service],
        capture_output=True, text=True, timeout=5,
    )
    return result.returncode, result.stdout.strip(), result.stderr.strip()


@app.route('/heartbeat', methods=['GET'])
def heartbeat_status():
    code, stdout, stderr = _systemctl_user('is-active', HEARTBEAT_SERVICE)
    return jsonify({
        "service": HEARTBEAT_SERVICE,
        "active": stdout == 'active',
        "raw": stdout or stderr,
    })


@app.route('/heartbeat/start', methods=['POST'])
def heartbeat_start():
    code, stdout, stderr = _systemctl_user('start', HEARTBEAT_SERVICE)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "rc": code,
        "stderr": stderr,
    })


@app.route('/heartbeat/stop', methods=['POST'])
def heartbeat_stop():
    code, stdout, stderr = _systemctl_user('stop', HEARTBEAT_SERVICE)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "rc": code,
        "stderr": stderr,
    })


# === SLTF retirement-walk endpoints (temporary, remove post-2026-05-12) ===

@app.route('/heartbeat/sltf', methods=['GET'])
def sltf_heartbeat_status():
    code, stdout, stderr = _systemctl_user('is-active', SLTF_HEARTBEAT_SERVICE)
    return jsonify({
        "service": SLTF_HEARTBEAT_SERVICE,
        "active": stdout == 'active',
        "raw": stdout or stderr,
    })


@app.route('/heartbeat/sltf/start', methods=['POST'])
def sltf_heartbeat_start():
    # Stop the default heartbeat first so the two don't both call Anthropic.
    # The unit's Conflicts= directive also enforces this at the systemd
    # level, but doing it here explicitly returns a tidier status.
    _systemctl_user('stop', HEARTBEAT_SERVICE)
    code, stdout, stderr = _systemctl_user('start', SLTF_HEARTBEAT_SERVICE)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "rc": code,
        "stderr": stderr,
    })


@app.route('/heartbeat/sltf/stop', methods=['POST'])
def sltf_heartbeat_stop():
    code, stdout, stderr = _systemctl_user('stop', SLTF_HEARTBEAT_SERVICE)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "rc": code,
        "stderr": stderr,
    })


@app.route('/reflection_log', methods=['GET'])
def reflection_log():
    """Raw tail of heartbeat.jsonl — for terminal debugging. The React
    operator console uses /reflection_feed which returns parsed entries."""
    n = int(request.args.get('n', 100))
    if not REFLECTION_LOG.exists():
        return jsonify({
            "lines": [],
            "error": f"{REFLECTION_LOG} not found",
        })
    try:
        with REFLECTION_LOG.open() as f:
            lines = f.readlines()[-n:]
        return jsonify({"lines": lines, "path": str(REFLECTION_LOG), "n": n})
    except Exception as e:
        return jsonify({"lines": [], "error": str(e)}), 500


@app.route('/reflection_feed', methods=['GET'])
def reflection_feed():
    """Parsed reflection feed in React-operator-console shape.

    Returns the last N beats from heartbeat.jsonl, expanded to log entries
    with {id, time, tag, text}. One reflection entry + one entry per
    action per beat. Tags: 'reflect' (Haiku/Sonnet's reasoning), 'tool'
    (action calls), 'speech' (TTS-bound speak calls — tagged differently
    so the UI can render them with the speech-bubble styling).

    Query params:
      beats — how many recent beats to include (default 20)

    Used by the operator console's Claude tab — replaces the placeholder
    LOG_SCRIPT cycler with real reasoning. Doubles as a debugging tool:
    state-at-beat + reflection-of-state lets you trace how the model
    interpreted each tick into the action it took.
    """
    n_beats = int(request.args.get('beats', 20))
    if not REFLECTION_LOG.exists():
        return jsonify({"entries": [], "error": f"{REFLECTION_LOG} not found"})
    try:
        with REFLECTION_LOG.open() as f:
            lines = f.readlines()[-n_beats:]
        entries = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                beat = json.loads(line)
            except Exception:
                continue
            beat_num = beat.get('beat', 0)
            # ISO timestamp → local HH:MM:SS for display.
            ts = beat.get('timestamp', '')
            time_str = '--:--:--'
            try:
                dt = datetime.fromisoformat(ts.replace('Z', '+00:00'))
                time_str = dt.astimezone().strftime('%H:%M:%S')
            except Exception:
                pass

            reflection = (beat.get('reflection') or '').strip()
            if reflection:
                entries.append({
                    'id': f'{beat_num}-r',
                    'time': time_str,
                    'tag': 'reflect',
                    'text': reflection,
                })

            for i, action in enumerate(beat.get('actions') or []):
                # action is a string like "speak(text=...)" or "look(pan=-45, tilt=-10)".
                action_str = str(action)
                tag = 'speech' if action_str.startswith('speak') else 'tool'
                entries.append({
                    'id': f'{beat_num}-a{i}',
                    'time': time_str,
                    'tag': tag,
                    'text': action_str,
                })
        return jsonify({"entries": entries, "beats": len(lines)})
    except Exception as e:
        return jsonify({"entries": [], "error": str(e)}), 500


@app.route('/network', methods=['GET'])
def network():
    """WiFi telemetry for the operator console NETWORK panel.

    Parsed from `iw dev wlan0 link` because /proc/net/wireless isn't
    populated on this kernel and nmcli's RSSI is a 0-100 percentage rather
    than dBm. Fields:
      ssid             — current AP SSID (string, empty if not connected)
      rssi_dbm         — signal strength in dBm (negative; -50 strong, -85 weak)
      bars             — 0..4 derived from rssi_dbm (drops to 0 if not connected)
      tx_bitrate_mbps  — currently negotiated TX rate (proxy for "what the radio is doing")
      rx_bitrate_mbps  — currently negotiated RX rate
      freq_mhz         — operating frequency (2412/5G band sanity check)
      iface            — interface name we polled (wlan0 by default)
    Falls back to nulls + bars=0 if iw fails (rover on ethernet, wifi down,
    iface name different, etc) — operator panel renders 0-bars as "no link"
    rather than crashing.
    """
    iface = "wlan0"
    out = {
        "iface": iface,
        "ssid": "",
        "rssi_dbm": None,
        "bars": 0,
        "tx_bitrate_mbps": None,
        "rx_bitrate_mbps": None,
        "freq_mhz": None,
    }
    try:
        r = subprocess.run(
            ["iw", "dev", iface, "link"],
            capture_output=True, text=True, timeout=2,
        )
        if r.returncode != 0 or "Not connected" in r.stdout:
            return jsonify(out)
        for line in r.stdout.splitlines():
            line = line.strip()
            if line.startswith("SSID:"):
                out["ssid"] = line.split(":", 1)[1].strip()
            elif line.startswith("signal:"):
                # e.g. "signal: -42 dBm"
                try:
                    out["rssi_dbm"] = int(line.split(":", 1)[1].strip().split()[0])
                except Exception:
                    pass
            elif line.startswith("freq:"):
                try:
                    out["freq_mhz"] = int(line.split(":", 1)[1].strip())
                except Exception:
                    pass
            elif line.startswith("tx bitrate:"):
                try:
                    out["tx_bitrate_mbps"] = float(line.split(":", 1)[1].strip().split()[0])
                except Exception:
                    pass
            elif line.startswith("rx bitrate:"):
                try:
                    out["rx_bitrate_mbps"] = float(line.split(":", 1)[1].strip().split()[0])
                except Exception:
                    pass
    except Exception as e:
        out["error"] = str(e)
        return jsonify(out)

    # Bars from RSSI. Standard mapping: stronger than -50 is "full bars"
    # territory, weaker than -85 is unusable.
    rssi = out["rssi_dbm"]
    if rssi is None:
        out["bars"] = 0
    elif rssi >= -50:
        out["bars"] = 4
    elif rssi >= -67:
        out["bars"] = 3
    elif rssi >= -75:
        out["bars"] = 2
    elif rssi >= -85:
        out["bars"] = 1
    else:
        out["bars"] = 0

    return jsonify(out)


def main():
    print(f"[control_panel] http://0.0.0.0:{PORT}", flush=True)
    try:
        from waitress import serve
        serve(app, host='0.0.0.0', port=PORT, threads=4)
    except ImportError:
        app.run(host='0.0.0.0', port=PORT, threaded=True, debug=False)


if __name__ == '__main__':
    main()
