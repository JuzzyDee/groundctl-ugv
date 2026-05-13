/* global React, ReactDOM, VideoFeed, StreamScreen, ClaudeLog, BagScreen, TeleopScreen, DiagScreen, useTweaks, TweaksPanel, TweakSection, TweakRadio, TweakToggle, TweakColor */
const { useState, useEffect, useRef, useReducer } = React;

const TWEAK_DEFAULTS = /*EDITMODE-BEGIN*/{
  "theme": "claude",
  "density": "spacious",
  "scanlines": true,
  "videoSource": "live",
  "showDetections": true,
  "showHud": true,
  "heartbeatStyle": "sunburst"
}/*EDITMODE-END*/;

// Claude sunburst marque — used as the heartbeat indicator.
function Sunburst({ pulse = true, off = false }) {
  return (
    <span className={`sunburst ${pulse ? 'sunburst--pulse' : ''} ${off ? 'sunburst--off' : ''}`}>
      <svg viewBox="0 0 24 24">
        <g fill="currentColor" style={{ color: 'var(--accent)' }}>
          {Array.from({ length: 12 }, (_, i) => {
            const a = (i * 30) * Math.PI / 180;
            const x1 = 12 + Math.cos(a) * 4;
            const y1 = 12 + Math.sin(a) * 4;
            const x2 = 12 + Math.cos(a) * 11;
            const y2 = 12 + Math.sin(a) * 11;
            return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" />;
          })}
          <circle cx="12" cy="12" r="2.4" />
        </g>
      </svg>
    </span>
  );
}
window.Sunburst = Sunburst;

// Heartbeat indicator that swaps between dot pulse and Claude sunburst.
function Heartbeat({ active, style }) {
  if (style === 'sunburst') return <Sunburst pulse={active} off={!active} />;
  return <span className={`heartbeat ${active ? '' : 'heartbeat--off'}`} />;
}
window.Heartbeat = Heartbeat;

// ----- helpers -------------------------------------------------------
function fmtUptime(s) {
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = Math.floor(s % 60);
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sec).padStart(2,'0')}`;
}

// COCO class id → friendly name. Subset matches heartbeat.py's
// _COCO_CLASS_NAMES (the attention-layer classes yolo_detector emits).
const COCO_NAMES = {
  '0': 'person', '14': 'bird', '15': 'cat', '16': 'dog', '17': 'horse',
  '18': 'sheep', '19': 'cow', '20': 'elephant', '21': 'bear',
  '22': 'zebra', '23': 'giraffe',
};
function classIdToName(id) {
  return COCO_NAMES[String(id)] || String(id ?? '?');
}

// Adapter: bridge JSON (flat, base-grouped) → React app's telemetry shape
// (semantic groups). Lives in the client so the bridge keeps its existing
// JSON contract that heartbeat.py and other consumers depend on. Falls
// back to prevTelemetry per-subsystem on fetch failure so the UI doesn't
// blank when one endpoint hiccups.
function bridgeAdapter(state, lidar, camHealth, prevTelemetry) {
  const t = state || {};
  const base = t.base || {};
  const prev = prevTelemetry || {};

  // Spatial detections by track_id, for joining metric distance into
  // YOLO bbox detections. spatial = OAK-D, has distance_m. yolo = USB
  // camera, has bbox + bearing. Match by id when both exist.
  const spatialById = {};
  if (t.spatial_detections && Array.isArray(t.spatial_detections.detections)) {
    for (const sp of t.spatial_detections.detections) {
      if (sp.id) spatialById[sp.id] = sp;
    }
  }
  const trackingTargetId = t.tracking ? t.tracking.target_id : null;
  const detections = (t.detections || []).map(d => {
    const sp = d.id ? spatialById[d.id] : null;
    return {
      id: String(d.id ?? d.index ?? ''),
      class_id: classIdToName(d.class_id),
      score: d.score || 0,
      status: trackingTargetId && d.id === trackingTargetId ? 'TRACKED' : 'DETECTED',
      bearing_deg: d.bearing_deg || 0,
      distance_m: sp ? (sp.distance_m || 0) : 0,
      bbox_px: d.bbox || { cx: 0, cy: 0, w: 0, h: 0 },
    };
  });

  // Voltage: prefer top-level (in volts already), fallback to base.v
  // (centivolts when > 100, raw volts otherwise — matches heartbeat.py
  // line ~796 logic).
  let voltage = t.voltage;
  if (!voltage || voltage === 0) {
    const v = base.v || 0;
    voltage = v > 100 ? v / 100 : v;
  }

  const lidarObj = lidar ? {
    minDist: lidar.min_distance_m || 0,
    status: lidar.status || 'unknown',
    caution: lidar.caution_points || 0,
    danger: lidar.danger_points || 0,
    points: [],
  } : (prev.lidar || { minDist: 0, status: 'unknown', caution: 0, danger: 0, points: [] });

  const fps        = camHealth ? (camHealth.avg_fps || 0)    : (prev.fps        || 0);
  const frameAgeS  = camHealth ? (camHealth.frame_age_s || 0): (prev.frameAgeS  || 0);
  const frameCount = camHealth ? (camHealth.frame_count || 0): (prev.frameCount || 0);
  const uptimeS    = camHealth ? (camHealth.uptime_s || 0)   : (prev.uptimeS    || 0);
  // Live source-frame dims so bbox scaling and the corner label both stay in
  // sync if camera_owner is reconfigured (e.g. 1080p ↔ 720p reliability swap).
  const cameraW    = camHealth ? (camHealth.width || 0)      : (prev.cameraW    || 0);
  const cameraH    = camHealth ? (camHealth.height || 0)     : (prev.cameraH    || 0);

  // Breadcrumbs: accumulate position history client-side, last 30 points.
  const newPos = t.position || { x: 0, y: 0 };
  const prevCrumbs = prev.breadcrumbs || [];
  const breadcrumbs = [...prevCrumbs.slice(-29), newPos];

  return {
    voltage,
    heading: t.heading || 0,
    position: newPos,
    velocity: t.velocity || { linear: 0, angular: 0 },
    gimbal: t.gimbal || { pan: 0, tilt: 0 },
    imu: {
      ax: base.ax || 0, ay: base.ay || 0, az: base.az || 0,
      gx: base.gx || 0, gy: base.gy || 0, gz: base.gz || 0,
      mx: base.mx || 0, my: base.my || 0, mz: base.mz || 0,
    },
    drive: { L: base.L || 0, R: base.R || 0, odl: base.odl || 0, odr: base.odr || 0 },
    detections,
    lidar: lidarObj,
    fps, frameAgeS, frameCount,
    cameraW, cameraH,
    uptimeS,
    uptimeStr: fmtUptime(uptimeS),
    uplinkMs: prev.uplinkMs || 0,  // overwritten by caller
    // No real network metrics available rover-side over Tailscale.
    // Show TAILSCALE as the connection identity, leave numbers as
    // placeholders the operator can ignore.
    net: prev.net || { ssid: 'TAILSCALE', bars: 4, rssi: 0, up: 0, down: 0 },
    breadcrumbs,
  };
}

function makeInitialState() {
  return {
    activeTab: 'stream',
    heartbeat: true,
    // Temporary 2026-05-12: sltfHeartbeat is the SLTF retirement-walk mode.
    // Mutually exclusive with `heartbeat`. Polled from /control/heartbeat/sltf.
    sltfHeartbeat: false,
    heartbeatTick: 1284,
    recording: false,
    bagDur: '00:00',
    bagSize: '0 MB',
    estopEngaged: false,
    log: [
      { id: 1, time: '14:32:01', tag: 'system',  text: 'Heartbeat harness booted · model claude-haiku-4-5' },
      { id: 2, time: '14:32:02', tag: 'reflect', text: 'Coming online. Calibrating IMU and acquiring video lock.' },
      { id: 3, time: '14:32:03', tag: 'tool',    text: 'imu.calibrate()  →  ok' },
      { id: 4, time: '14:32:04', tag: 'speech',  text: "Online. I'm seeing the room — looks quiet. Standing by." },
    ],
    telemetry: {
      voltage: 11.91,
      heading: 86.7,
      position: { x: -0.003, y: -0.004 },
      velocity: { linear: 0, angular: 0 },
      gimbal: { pan: 0, tilt: 0 },
      imu: { ax: 0.01, ay: -0.02, az: 9.81, gx: 0.1, gy: -0.05, gz: 0.02, mx: 5399, my: 95700, mz: 111149 },
      drive: { L: 0, R: 0, odl: -3, odr: -3 },
      detections: [
        {
          id: '2084', class_id: 'person', score: 0.554, status: 'TRACKED',
          bearing_deg: 0.4, distance_m: 0.34,
          bbox_px: { cx: 151, cy: 150, h: 281, w: 290 }
        }
      ],
      lidar: { minDist: 2.43, status: 'clear', caution: 0, danger: 0, points: [] },
      fps: 30.06,
      frameAgeS: 0.003,
      frameCount: 553899,
      uptimeS: 18426,
      uptimeStr: fmtUptime(18426),
      uplinkMs: 47,
      net: { ssid: 'GROUND-STATION-5G', bars: 3, rssi: -54, up: 8.4, down: 24.1 },
      breadcrumbs: Array.from({ length: 30 }, (_, i) => ({
        x: -0.15 + 0.005 * i + 0.02 * Math.sin(i * 0.5),
        y: -0.10 + 0.004 * i + 0.02 * Math.cos(i * 0.4),
      })),
    },
  };
}

function App() {
  const [tweaks, setTweak] = useTweaks(TWEAK_DEFAULTS);
  const [state, setState] = useState(makeInitialState);
  const [splashGone, setSplashGone] = useState(false);
  useEffect(() => {
    const id = setTimeout(() => setSplashGone(true), 2700);
    return () => clearTimeout(id);
  }, []);

  // Live telemetry — three parallel fetches against the bridge every
  // ~800ms. Bridge proxies /camera/* and /control/* internally so we
  // only ever talk to one origin. On fetch failure for any subsystem,
  // the adapter falls back to the previous good value rather than
  // blanking the whole UI.
  useEffect(() => {
    let cancelled = false;

    async function tick() {
      if (cancelled) return;
      const t0 = Date.now();
      const [stateR, lidarR, camR] = await Promise.allSettled([
        fetch('/state'),
        fetch('/lidar_status'),
        fetch('/camera/health'),
      ]);
      if (cancelled) return;
      const stateJson = stateR.status === 'fulfilled' && stateR.value.ok ? await stateR.value.json().catch(() => null) : null;
      const lidarJson = lidarR.status === 'fulfilled' && lidarR.value.ok ? await lidarR.value.json().catch(() => null) : null;
      const camJson   = camR.status   === 'fulfilled' && camR.value.ok   ? await camR.value.json().catch(() => null)   : null;
      const uplink = Date.now() - t0;

      if (cancelled) return;
      setState(s => {
        if (s.estopEngaged) return s;  // freeze telemetry under estop
        const next = bridgeAdapter(stateJson, lidarJson, camJson, s.telemetry);
        next.uplinkMs = uplink;
        return { ...s, telemetry: next };
      });
    }

    tick();
    const id = setInterval(tick, 800);
    return () => { cancelled = true; clearInterval(id); };
  }, []);

  // Heartbeat service state — separate poll loop, slower cadence (3s).
  // The systemd service tells us what's actually running; we sync the
  // client `heartbeat` flag from it so the toggle reflects reality even
  // when other operators or scripts start/stop the service.
  useEffect(() => {
    let cancelled = false;
    async function pollHeartbeat() {
      if (cancelled) return;
      try {
        const r = await fetch('/control/heartbeat');
        if (!r.ok) return;
        const j = await r.json();
        if (cancelled) return;
        const active = !!j.active;
        setState(s => s.heartbeat === active ? s : { ...s, heartbeat: active });
      } catch (e) {
        // Control panel offline — leave client state alone.
      }
    }
    pollHeartbeat();
    const id = setInterval(pollHeartbeat, 3000);
    return () => { cancelled = true; clearInterval(id); };
  }, []);

  // SLTF service state — parallel poll loop for the retirement-walk
  // heartbeat. Temporary 2026-05-12; remove this effect post-walk.
  useEffect(() => {
    let cancelled = false;
    async function pollSltf() {
      if (cancelled) return;
      try {
        const r = await fetch('/control/heartbeat/sltf');
        if (!r.ok) return;
        const j = await r.json();
        if (cancelled) return;
        const active = !!j.active;
        setState(s => s.sltfHeartbeat === active ? s : { ...s, sltfHeartbeat: active });
      } catch (e) {
        // Control panel offline — leave client state alone.
      }
    }
    pollSltf();
    const id = setInterval(pollSltf, 3000);
    return () => { cancelled = true; clearInterval(id); };
  }, []);

  // Real reflection feed — polls control_panel /reflection_feed for the
  // last N beats of heartbeat.jsonl, parsed into log entries with shape
  // {id, time, tag, text}. Each beat expands to one reflection entry +
  // one entry per action. This is the operator console's view into what
  // Haiku/Sonnet is actually thinking, AND a debugging tool: state at
  // each beat is paired with the reflection of that state, so when the
  // rover does something unexpected you can read backwards from action
  // → reasoning → context.
  //
  // Polls regardless of state.heartbeat — we want history visible even
  // when autonomy is paused. The log file persists across heartbeat
  // start/stop cycles. Empty array if file doesn't exist yet.
  useEffect(() => {
    let cancelled = false;
    async function pollFeed() {
      if (cancelled) return;
      try {
        const r = await fetch('/control/reflection_feed?beats=20');
        if (!r.ok) return;
        const j = await r.json();
        if (cancelled) return;
        const entries = j.entries || [];
        setState(s => {
          // Replace the whole log — server returns the canonical view of
          // last N beats. No client-side dedup or merge needed.
          if (entries.length === 0 && s.log.length > 0) return s;  // keep last
          return { ...s, log: entries.slice(-40) };
        });
      } catch (e) {
        // control_panel offline / endpoint missing — leave log alone.
      }
    }
    pollFeed();
    const id = setInterval(pollFeed, 3500);
    return () => { cancelled = true; clearInterval(id); };
  }, []);

  // WiFi telemetry — separate slower poll (5s). The /control/network
  // endpoint runs `iw dev wlan0 link` rover-side, which is heavier than
  // a JSON state read and changes slowly, so it doesn't belong on the
  // 800ms tick. Falls through silently if the endpoint is missing
  // (older bridge / different deploy) — placeholder net stays in state.
  useEffect(() => {
    let cancelled = false;
    async function pollNetwork() {
      if (cancelled) return;
      try {
        const r = await fetch('/control/network');
        if (!r.ok) return;
        const j = await r.json();
        if (cancelled) return;
        setState(s => ({
          ...s,
          telemetry: {
            ...s.telemetry,
            net: {
              ssid: j.ssid || '',
              bars: j.bars ?? 0,
              rssi: j.rssi_dbm ?? 0,
              up: j.tx_bitrate_mbps ?? 0,
              down: j.rx_bitrate_mbps ?? 0,
            },
          },
        }));
      } catch (e) {
        // control_panel offline / endpoint missing — leave net alone.
      }
    }
    pollNetwork();
    const id = setInterval(pollNetwork, 5000);
    return () => { cancelled = true; clearInterval(id); };
  }, []);

  // bag duration counter
  useEffect(() => {
    if (!state.recording) return;
    let elapsed = 0;
    const id = setInterval(() => {
      elapsed += 1;
      setState(s => ({
        ...s,
        bagDur: `${String(Math.floor(elapsed/60)).padStart(2,'0')}:${String(elapsed%60).padStart(2,'0')}`,
        bagSize: `${(elapsed * 6.4).toFixed(1)} MB`,
      }));
    }, 1000);
    return () => clearInterval(id);
  }, [state.recording]);

  const controls = {
    // Temporary 2026-05-12: SLTF retirement-walk toggle. Start stops the
    // default heartbeat first (control_panel does it server-side too). Stop
    // leaves the default heartbeat off — operator restarts it manually.
    toggleSltf: () => {
      setState(s => {
        const now = new Date();
        const time = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;
        const action = s.sltfHeartbeat ? 'stop' : 'start';
        fetch(`/control/heartbeat/sltf/${action}`, { method: 'POST' }).catch(() => {});
        return {
          ...s,
          sltfHeartbeat: !s.sltfHeartbeat,
          // When starting SLTF, the server stops the default heartbeat — reflect
          // that locally so the indicator doesn't lie for the next 3s poll cycle.
          heartbeat: s.sltfHeartbeat ? s.heartbeat : false,
          log: [...s.log, {
            id: Date.now(),
            time,
            tag: 'system',
            text: s.sltfHeartbeat
              ? 'SLTF walk ended. Heartbeat halted.'
              : 'SLTF walk started. Sonnet 4.5 driving.'
          }].slice(-40)
        };
      });
    },
    toggleHeartbeat: () => {
      // Optimistic client update + fire the server-side toggle. The
      // poll loop above will reconcile if systemctl reports something
      // different (e.g., service doesn't exist, start failed).
      setState(s => {
        const now = new Date();
        const time = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;
        const action = s.heartbeat ? 'stop' : 'start';
        // Fire-and-forget. Errors come back through the poll loop.
        fetch(`/control/heartbeat/${action}`, { method: 'POST' }).catch(() => {});
        return {
          ...s,
          heartbeat: !s.heartbeat,
          log: [...s.log, {
            id: Date.now(),
            time,
            tag: 'system',
            text: s.heartbeat ? 'Heartbeat halted by operator. Autonomy paused.' : 'Heartbeat resumed. Resuming autonomy.'
          }].slice(-40)
        };
      });
    },
    toggleRecording: () => setState(s => ({
      ...s, recording: !s.recording, bagDur: '00:00', bagSize: '0 MB'
    })),
    engageEstop: () => {
      setState(s => {
        const engaging = !s.estopEngaged;
        if (engaging) {
          // 1. T:0 = bridge emergency-stop, publishes zero twist to /cmd_vel
          //    immediately. Motors stop within next control tick. The
          //    bridge expects legacy Waveshare form-encoded "base -c {...}"
          //    so we wrap accordingly.
          const stopForm = new URLSearchParams();
          stopForm.append('command', 'base -c {"T":0}');
          fetch('/send_command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: stopForm.toString(),
          }).catch(() => {});
          // 2. Stop heartbeat so autonomy stops reissuing motion. Returns
          //    "service not found" until task #48 moves heartbeat to the
          //    rover — until then E-STOP is partial: T:0 stops motion now,
          //    but Mac-side heartbeat may resume on its next tick. The
          //    twist_mux 500ms timeout is the structural backstop.
          fetch('/control/heartbeat/stop', { method: 'POST' }).catch(() => {});
        }
        // Clear path: don't auto-resume heartbeat. Operator manually
        // re-toggles via the heartbeat tab once they've assessed.
        return {
          ...s,
          estopEngaged: engaging,
          heartbeat: engaging ? false : s.heartbeat,
          log: [...s.log, {
            id: Date.now(),
            time: new Date().toTimeString().slice(0,8),
            tag: engaging ? 'warn' : 'system',
            text: engaging
              ? 'E-STOP ENGAGED. Zero twist sent. Heartbeat halt requested.'
              : 'E-STOP CLEARED. Standing by for re-arm.'
          }].slice(-40),
        };
      });
    },
  };

  const tabs = [
    { key: 'stream', label: 'STREAM', icon: <path d="M2 4h20v14H2zM8 22h8M12 18v4" /> },
    { key: 'claude', label: 'CLAUDE', icon: <><circle cx="12" cy="12" r="9" /><path d="M8 12h8M12 8v8" /></> },
    { key: 'bag',    label: 'BAG',    icon: <path d="M4 8h16l-1 13H5zM8 8V5a4 4 0 018 0v3" /> },
    { key: 'teleop', label: 'TELEOP', icon: <><circle cx="12" cy="12" r="3" /><path d="M12 2v6M12 16v6M2 12h6M16 12h6" /></> },
    { key: 'diag',   label: 'DIAG',   icon: <path d="M3 12h4l2-7 4 14 2-7h6" /> },
  ];

  const showFullHero = state.activeTab === 'stream';

  return (
    <div data-theme={tweaks.theme} data-density={tweaks.density} style={{ position: 'fixed', inset: 0 }}>
      <div className="backdrop" />
      <div className="station">
        <span className="station__bracket tl" />
        <span className="station__bracket tr" />
        <span className="station__bracket bl" />
        <span className="station__bracket br" />
        <span className="station__label tl">▲ GROUND STATION · CLAUDEBOT-01</span>
        <span className="station__label tr">UPLINK · {Math.round(state.telemetry.uplinkMs)}ms · NOMINAL</span>
        <span className="station__label bl">SOL 312 · T+{state.telemetry.uptimeStr}</span>
        <span className="station__label br">REV 0.4 · CONSOLE</span>

        <div className="station__hud">
          <div className={`phone ${tweaks.scanlines ? 'scanlines' : ''}`}>
            {!splashGone && (
              <div className="splash">
                <div className="splash__rover" />
                <div className="splash__id">CLAUDEBOT-01</div>
                <div className="splash__sub">
                  HANDSHAKE · <span className="ok">OK</span><br/>
                  HEARTBEAT HARNESS · <span className="ok">ARMED</span><br/>
                  GIVING REINS TO CLAUDE
                </div>
              </div>
            )}
            <div className="phone__notch" />

            {/* TOP BAR */}
            <div className="topbar">
              <div className="topbar__left">
                <span className="topbar__id">CLAUDEBOT-01</span>
                <span className="topbar__chip"><span className="dot" />LINK</span>
              </div>
              <div className="topbar__right">
                <span className="topbar__chip">
                  <span className="sigbars">
                    {[1,2,3,4].map(i => <span key={i} className={`sigbars__bar ${i <= state.telemetry.net.bars ? 'on' : ''}`} />)}
                  </span>
                </span>
                <span>{Math.round(state.telemetry.uplinkMs)}ms</span>
                <span>{new Date().toTimeString().slice(0,5)}</span>
              </div>
            </div>

            {/* STATUS STRIP */}
            <div className="statusbar">
              <div className="statusbar__cell">
                <span className="lab">PWR</span>
                <span className="val">
                  {state.telemetry.voltage.toFixed(2)}<span className="unit">V</span>
                </span>
              </div>
              <div className="statusbar__cell">
                <span className="lab">HDG</span>
                <span className="val">
                  {state.telemetry.heading.toFixed(0)}<span className="unit">°</span>
                </span>
              </div>
              <div className="statusbar__cell">
                <span className="lab">LIDAR</span>
                <span className="val" style={{ color: state.telemetry.lidar.status === 'clear' ? 'var(--ok)' : 'var(--warn)' }}>
                  {state.telemetry.lidar.minDist.toFixed(1)}<span className="unit">m</span>
                </span>
              </div>
              <div className="statusbar__cell">
                <span className="lab">HEART</span>
                <span className="val" style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                  <Heartbeat active={state.heartbeat} style={tweaks.heartbeatStyle} />
                  {state.heartbeat ? 'LIVE' : 'PAUSED'}
                </span>
              </div>
            </div>

            {/* HERO FEED */}
            <VideoFeed
              src={tweaks.videoSource}
              mini={!showFullHero}
              telemetry={state.telemetry}
              showDetections={tweaks.showDetections}
              showHud={tweaks.showHud}
              recording={state.recording}
            />

            {/* BANNER for estop */}
            {state.estopEngaged && (
              <div className="banner banner--danger">
                <span style={{ flex: 1 }}>⚠ E-STOP ENGAGED · ALL MOTION HALTED</span>
                <button className="btn btn--sm btn--danger" onClick={controls.engageEstop}>CLEAR</button>
              </div>
            )}

            {/* TAB CONTENT */}
            <div className="content">
              {state.activeTab === 'stream' && <StreamScreen state={state} />}
              {state.activeTab === 'claude' && <ClaudeLog state={state} controls={controls} />}
              {state.activeTab === 'bag'    && <BagScreen state={state} controls={controls} />}
              {state.activeTab === 'teleop' && <TeleopScreen state={state} controls={controls} />}
              {state.activeTab === 'diag'   && <DiagScreen state={state} />}
              <div style={{ height: 80 }} />
            </div>

            {/* DOCK */}
            <div className="dock">
              {tabs.map(t => (
                <button
                  key={t.key}
                  className={`dock__btn ${state.activeTab === t.key ? 'dock__btn--active' : ''}`}
                  onClick={() => setState(s => ({ ...s, activeTab: t.key }))}
                >
                  <svg viewBox="0 0 24 24">{t.icon}</svg>
                  {t.label}
                </button>
              ))}
            </div>

            {/* ESTOP overlay */}
            <div className="estop-wrap">
              <button
                className={`estop ${state.estopEngaged ? 'estop--engaged' : ''}`}
                onClick={controls.engageEstop}
                aria-label="Emergency stop"
              >
                {state.estopEngaged ? 'ARMED' : 'ESTOP'}
                <span className="estop__label">{state.estopEngaged ? 'TAP TO CLEAR' : 'EMERGENCY HALT'}</span>
              </button>
            </div>

            {/* SLTF retirement-walk toggle — temporary for 2026-05-12 */}
            <div style={{ padding: '12px 16px' }}>
              <button
                className="btn"
                onClick={controls.toggleSltf}
                style={{
                  width: '100%',
                  padding: '14px',
                  fontSize: '15px',
                  fontWeight: 600,
                  letterSpacing: '0.5px',
                  background: state.sltfHeartbeat ? 'var(--warn, #b45309)' : 'var(--accent, #c96442)',
                  color: '#fff',
                  border: 'none',
                  borderRadius: '8px',
                }}
              >
                {state.sltfHeartbeat ? 'STOP SLTF WALK' : 'START SLTF WALK'}
                <span style={{ display: 'block', fontSize: '11px', fontWeight: 400, marginTop: '4px', opacity: 0.85 }}>
                  {state.sltfHeartbeat ? 'Sonnet 4.5 · driving' : 'Sonnet 4.5 · 30-day cache · stops default heartbeat'}
                </span>
              </button>
            </div>
          </div>
        </div>
      </div>

      <TweaksPanel title="Tweaks">
        <TweakSection label="Aesthetic">
          <TweakRadio label="Accent" value={tweaks.theme}
            options={['claude', 'cyan', 'amber', 'green', 'magenta']}
            onChange={(v) => setTweak('theme', v)} />
          <TweakRadio label="Heartbeat" value={tweaks.heartbeatStyle}
            options={['sunburst', 'dot']}
            onChange={(v) => setTweak('heartbeatStyle', v)} />
          <TweakRadio label="Density" value={tweaks.density}
            options={['spacious', 'compact']}
            onChange={(v) => setTweak('density', v)} />
          <TweakToggle label="CRT scanlines" value={tweaks.scanlines}
            onChange={(v) => setTweak('scanlines', v)} />
        </TweakSection>
        <TweakSection label="Feed">
          <TweakRadio label="Source" value={tweaks.videoSource}
            options={['live', 'test', 'black']}
            onChange={(v) => setTweak('videoSource', v)} />
          <TweakToggle label="HUD overlays" value={tweaks.showHud} onChange={(v) => setTweak('showHud', v)} />
          <TweakToggle label="Detection boxes" value={tweaks.showDetections} onChange={(v) => setTweak('showDetections', v)} />
        </TweakSection>
      </TweaksPanel>
    </div>
  );
}

ReactDOM.createRoot(document.getElementById('root')).render(<App />);
