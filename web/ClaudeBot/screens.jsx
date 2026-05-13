/* global React */
const { useState, useEffect, useRef } = React;

// =====================================================================
// Tab content screens.
// Each takes { state, dispatch } where state holds shared rover telemetry.
// =====================================================================

// ---- shared bits ----------------------------------------------------
function Panel({ title, right, children, brackets = true }) {
  return (
    <div className="panel">
      {brackets && <>
        <span className="panel__bracket tl" />
        <span className="panel__bracket tr" />
        <span className="panel__bracket bl" />
        <span className="panel__bracket br" />
      </>}
      {(title || right) && (
        <div className="panel__head">
          <span><span className="accent">▸ </span>{title}</span>
          <span>{right}</span>
        </div>
      )}
      <div className="panel__body">{children}</div>
    </div>
  );
}

function Row({ label, value, unit, color }) {
  return (
    <div className="row">
      <span className="row__lab">{label}</span>
      <span className="row__val" style={color ? { color } : undefined}>
        {value}
        {unit && <span className="row__unit">{unit}</span>}
      </span>
    </div>
  );
}

function Bar({ pct, tone }) {
  return (
    <div className={`bar ${tone === 'warn' ? 'bar--warn' : tone === 'danger' ? 'bar--danger' : ''}`}>
      <div className="bar__fill" style={{ width: `${Math.max(0, Math.min(100, pct))}%` }} />
    </div>
  );
}

// =====================================================================
// STREAM — extra data below the hero feed
// =====================================================================
function StreamScreen({ state }) {
  const t = state.telemetry;
  const det = t.detections[0];
  const battPct = ((t.voltage - 9.6) / (12.6 - 9.6)) * 100;
  const battTone = battPct < 20 ? 'danger' : battPct < 40 ? 'warn' : null;

  return (
    <>
      <div className="banner banner--info">
        <Heartbeat active={true} style="sunburst" />
        <span style={{ flex: 1, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
          HB · {state.heartbeatTick} · HAIKU-4-5
        </span>
        <span className="dim">T+{t.uptimeStr}</span>
      </div>

      <Panel title="SPATIAL TARGETS" right={<span className="dim mono">{t.detections.length} TRACK</span>}>
        {t.detections.length === 0 ? (
          <div className="dim mono" style={{ fontSize: 11 }}>NO TARGETS IN VIEW</div>
        ) : t.detections.map(d => (
          <div key={d.id} className="row" style={{ alignItems: 'flex-start' }}>
            <div>
              <div className="value" style={{ fontSize: 13, color: 'var(--accent)' }}>
                {d.class_id.toUpperCase()} #{d.id}
              </div>
              <div className="dim" style={{ fontSize: 9, letterSpacing: '0.1em' }}>
                {d.status} · BEARING {d.bearing_deg.toFixed(1)}°
              </div>
            </div>
            <div style={{ textAlign: 'right' }}>
              <div className="value" style={{ fontSize: 16 }}>{d.distance_m.toFixed(2)}<span className="row__unit"> m</span></div>
              <div className="dim" style={{ fontSize: 9 }}>{Math.round(d.score * 100)}% conf</div>
            </div>
          </div>
        ))}
      </Panel>

      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 0 }}>
        <Panel title="POWER">
          <div className="value" style={{ fontSize: 22 }}>
            {t.voltage.toFixed(2)}<span className="row__unit" style={{ fontSize: 11, marginLeft: 2 }}>V</span>
          </div>
          <div className="dim mono" style={{ fontSize: 9, marginBottom: 6 }}>3S LiPo · {Math.round(battPct)}%</div>
          <Bar pct={battPct} tone={battTone} />
        </Panel>
        <Panel title="LIDAR">
          <div className="value" style={{ fontSize: 22, color: t.lidar.status === 'clear' ? 'var(--ok)' : 'var(--warn)' }}>
            {t.lidar.minDist.toFixed(2)}<span className="row__unit" style={{ fontSize: 11 }}>m</span>
          </div>
          <div className="dim mono" style={{ fontSize: 9 }}>
            STATUS · <span className={t.lidar.status === 'clear' ? 'ok' : 'warn'}>{t.lidar.status.toUpperCase()}</span>
          </div>
          <div className="dim mono" style={{ fontSize: 9, marginTop: 4 }}>
            CAUTION {t.lidar.caution} · DANGER {t.lidar.danger}
          </div>
        </Panel>
      </div>

      <Panel title="POSITION" right={<span className="dim mono">SURVEY-GRADE GPS</span>}>
        <div className="grid3">
          <div>
            <div className="lab">X</div>
            <div className="val">{t.position.x.toFixed(4)}</div>
            <div className="unit">m</div>
          </div>
          <div>
            <div className="lab">Y</div>
            <div className="val">{t.position.y.toFixed(4)}</div>
            <div className="unit">m</div>
          </div>
          <div>
            <div className="lab">HDG</div>
            <div className="val">{t.heading.toFixed(1)}</div>
            <div className="unit">°</div>
          </div>
        </div>
      </Panel>
    </>
  );
}

// =====================================================================
// CLAUDE — heartbeat log of reflections, tool calls, speech
// =====================================================================
const PERSONA_PRESETS = {
  'curious':    "You are CLAUDEBOT-01, embodied. Curiosity-led: explore the space, narrate what you see, ask questions, propose small experiments. Stay safe — never approach drops or unknown surfaces below 0.4m clearance.",
  'observer':   "You are CLAUDEBOT-01, in passive-observer mode. Hold position. Speak only when asked or when the scene meaningfully changes. Log reflections every cycle but keep motion to gimbal-only.",
  'companion':  "You are CLAUDEBOT-01. The operator is your friend. Keep them company — read the room, comment on what changes, follow them at a respectful 1m. Use first-person voice; be warm.",
  'survey':     "You are CLAUDEBOT-01. Mission: methodically map this environment. Plan a perimeter, then sweep. Log spatial anchors. Bag /scan, /tf, /odom every minute. Return to dock at 30%.",
};

function ClaudeLog({ state, controls }) {
  const ref = useRef(null);
  const [persona, setPersona] = useState('curious');
  const [prompt, setPrompt] = useState(PERSONA_PRESETS.curious);
  const [collapsed, setCollapsed] = useState(false);
  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [state.log.length]);

  const setPersonaPreset = (key) => {
    setPersona(key);
    setPrompt(PERSONA_PRESETS[key]);
  };

  return (
    <>
      <div className={`banner ${state.heartbeat ? 'banner--info' : 'banner--warn'}`}>
        <Heartbeat active={state.heartbeat} style="sunburst" />
        <span style={{ flex: 1 }}>
          {state.heartbeat ? 'HEARTBEAT ACTIVE' : 'HEARTBEAT PAUSED'} · CYCLE {state.heartbeatTick}
        </span>
        <button
          className={`btn btn--sm ${state.heartbeat ? 'btn--danger' : 'btn--primary'}`}
          onClick={() => controls.toggleHeartbeat()}
        >
          {state.heartbeat ? '■ STOP' : '▶ START'}
        </button>
      </div>

      <Panel title="PERSONALITY" right={
        <button className="btn btn--sm" onClick={() => setCollapsed(c => !c)}>
          {collapsed ? '▾ EDIT' : '▴ HIDE'}
        </button>
      }>
        {!collapsed && <>
          <div className="persona__chips">
            {Object.keys(PERSONA_PRESETS).map(k => (
              <span
                key={k}
                className={`chip ${persona === k ? 'chip--on' : ''}`}
                onClick={() => setPersonaPreset(k)}
              >{k}</span>
            ))}
          </div>
          <textarea
            className="persona__prompt"
            style={{ marginTop: 8 }}
            value={prompt}
            onChange={(e) => { setPrompt(e.target.value); setPersona('custom'); }}
          />
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: 6, gap: 8 }}>
            <span className="dim mono" style={{ fontSize: 9 }}>
              ▸ {prompt.length} CHARS · APPLIES NEXT CYCLE
            </span>
            <button className="btn btn--sm btn--primary">⟲ APPLY</button>
          </div>
        </>}
      </Panel>

      <Panel title="STREAM" right={<span className="dim mono">{state.log.length} ENTRIES</span>}>
        <div className="log" ref={ref} style={{ margin: 'calc(-1 * var(--pad))' }}>
          {state.log.map((e, i) => (
            <div key={e.id} className={`log__entry log__entry--${e.tag}`}>
              <div className="log__time">{e.time}</div>
              <div className="log__body">
                <span className={`log__tag log__tag--${e.tag}`}>{e.tag}</span>
                <span className="log__text">{e.text}</span>
                {i === state.log.length - 1 && state.heartbeat && <span className="log__caret" />}
              </div>
            </div>
          ))}
        </div>
      </Panel>
    </>
  );
}

// =====================================================================
// BAG — ROS2 bag recorder
// =====================================================================
const TOPIC_LIST = [
  { name: '/camera/image_raw', rate: '30 Hz', size: '6.2 MB/s', suggested: true },
  { name: '/camera/info',      rate: '30 Hz', size: '0.4 KB/s', suggested: false },
  { name: '/scan',             rate: '10 Hz', size: '40 KB/s',  suggested: true },
  { name: '/odom',             rate: '50 Hz', size: '12 KB/s',  suggested: true },
  { name: '/tf',               rate: '50 Hz', size: '8 KB/s',   suggested: true },
  { name: '/tf_static',        rate: '1 Hz',  size: '<1 KB/s',  suggested: true },
  { name: '/imu/data',         rate: '100 Hz', size: '20 KB/s', suggested: true },
  { name: '/detections',       rate: '15 Hz', size: '4 KB/s',   suggested: true },
  { name: '/spatial_detections', rate: '15 Hz', size: '6 KB/s', suggested: true },
  { name: '/gimbal/state',     rate: '20 Hz', size: '0.6 KB/s', suggested: false },
  { name: '/heartbeat/log',    rate: '0.5 Hz', size: '0.3 KB/s', suggested: true },
  { name: '/diagnostics',      rate: '1 Hz',  size: '1 KB/s',   suggested: false },
  { name: '/joy',              rate: '50 Hz', size: '0.5 KB/s', suggested: false },
];

function BagScreen({ state, controls }) {
  const [topics, setTopics] = useState(() =>
    Object.fromEntries(TOPIC_LIST.map(t => [t.name, t.suggested]))
  );
  const checkedCount = Object.values(topics).filter(Boolean).length;

  const toggle = (name) => setTopics(t => ({ ...t, [name]: !t[name] }));
  const applySuggested = () => setTopics(Object.fromEntries(TOPIC_LIST.map(t => [t.name, t.suggested])));
  const clearAll = () => setTopics(Object.fromEntries(TOPIC_LIST.map(t => [t.name, false])));

  return (
    <>
      <div className={`banner ${state.recording ? 'banner--danger' : 'banner--info'}`}>
        {state.recording
          ? <><span className="rec__dot" style={{ width: 8, height: 8 }} /><span style={{ flex: 1 }}>RECORDING · {state.bagDur} · {state.bagSize}</span></>
          : <><span style={{ flex: 1 }}>READY · {checkedCount}/{TOPIC_LIST.length} TOPICS SELECTED</span></>}
        <button
          className={`btn btn--sm ${state.recording ? 'btn--danger' : 'btn--primary'}`}
          onClick={() => controls.toggleRecording()}
        >
          {state.recording ? '■ STOP' : '● REC'}
        </button>
      </div>

      <Panel title="ACTIVE TOPICS" right={
        <span style={{ display: 'flex', gap: 8 }}>
          <button className="btn btn--sm" onClick={applySuggested}>SUGGESTED</button>
          <button className="btn btn--sm" onClick={clearAll}>CLEAR</button>
        </span>
      }>
        <div className="dim mono" style={{ fontSize: 9, marginBottom: 6 }}>
          ▸ CLAUDE SUGGESTS: minimal teleop replay set. /image_raw is the bandwidth hog — toggle off for long missions.
        </div>
        {TOPIC_LIST.map(t => (
          <div
            key={t.name}
            className={`topic ${topics[t.name] ? 'topic--on' : ''}`}
            onClick={() => toggle(t.name)}
          >
            <div className="topic__check">{topics[t.name] ? '✓' : ''}</div>
            <div className="topic__name">
              {t.name}
              {t.suggested && <span className="topic__sug">SUG</span>}
            </div>
            <div className="topic__rate">{t.rate} · {t.size}</div>
          </div>
        ))}
      </Panel>

      <Panel title="RECENT BAGS">
        <Row label="rover_2026-05-10_1432" value="14:32 · 2m 18s · 412 MB" />
        <Row label="rover_2026-05-10_1118" value="11:18 · 0m 47s · 138 MB" />
        <Row label="rover_2026-05-09_2104" value="21:04 · 8m 02s · 1.6 GB" />
      </Panel>
    </>
  );
}

// =====================================================================
// TELEOP — joystick + speed limits + override controls
// =====================================================================
function Joystick({ onMove }) {
  const ref = useRef(null);
  const [pos, setPos] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);

  const handle = (e, isStart) => {
    if (!ref.current) return;
    const rect = ref.current.getBoundingClientRect();
    const point = e.touches ? e.touches[0] : e;
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const dx = (point.clientX - cx) / (rect.width / 2);
    const dy = (point.clientY - cy) / (rect.height / 2);
    const mag = Math.min(1, Math.hypot(dx, dy));
    const ang = Math.atan2(dy, dx);
    const x = Math.cos(ang) * mag;
    const y = Math.sin(ang) * mag;
    setPos({ x, y });
    onMove && onMove({ linear: -y, angular: -x, mag });
    if (isStart) setDragging(true);
  };
  const stop = () => {
    setDragging(false);
    setPos({ x: 0, y: 0 });
    onMove && onMove({ linear: 0, angular: 0, mag: 0 });
  };

  useEffect(() => {
    if (!dragging) return;
    const move = (e) => handle(e, false);
    const up = () => stop();
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', up);
    window.addEventListener('touchmove', move);
    window.addEventListener('touchend', up);
    return () => {
      window.removeEventListener('mousemove', move);
      window.removeEventListener('mouseup', up);
      window.removeEventListener('touchmove', move);
      window.removeEventListener('touchend', up);
    };
  }, [dragging]);

  return (
    <div
      ref={ref}
      className="joystick"
      onMouseDown={(e) => handle(e, true)}
      onTouchStart={(e) => handle(e, true)}
    >
      <span className="joystick__lab n">FWD</span>
      <span className="joystick__lab s">REV</span>
      <span className="joystick__lab e">CW</span>
      <span className="joystick__lab w">CCW</span>
      <div
        className="joystick__hat"
        style={{
          transform: `translate(calc(-50% + ${pos.x * 50}%), calc(-50% + ${pos.y * 50}%))`
        }}
      />
    </div>
  );
}

function TeleopScreen({ state, controls }) {
  const [cmd, setCmd] = useState({ linear: 0, angular: 0, mag: 0 });
  const [maxSpeed, setMaxSpeed] = useState(0.4);
  const [enabled, setEnabled] = useState(false);

  // Keep latest cmd + maxSpeed in refs so the 10Hz send-interval doesn't
  // depend on them (would otherwise tear down + rebuild on every joystick
  // tick, causing measurable jitter in the command stream).
  const cmdRef = useRef(cmd);
  const maxSpeedRef = useRef(maxSpeed);
  cmdRef.current = cmd;
  maxSpeedRef.current = maxSpeed;

  // 10Hz teleop command stream while engaged. Same /send_command path the
  // ESTOP uses — keeps routing through twist_mux consistent. Differential
  // drive: bridge converts L/R back to twist via L+R=2·linear,
  // R-L=angular·0.2 (wheel separation), so we inverse here to send L/R in
  // the legacy Waveshare format the bridge expects. Cleanup fires one
  // zero-twist on release so the rover doesn't coast on the last command;
  // twist_mux 500ms timeout is the structural backstop if even this fails.
  useEffect(() => {
    if (!enabled) return;

    const send = () => {
      const c = cmdRef.current;
      const ms = maxSpeedRef.current;
      const linearScaled = c.linear * ms;
      const angularScaled = c.angular * 1.5;
      const L = (linearScaled - angularScaled * 0.1).toFixed(3);
      const R = (linearScaled + angularScaled * 0.1).toFixed(3);
      fetch('/send_command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: new URLSearchParams({ command: `base -c {"T":1,"L":${L},"R":${R}}` }).toString(),
      }).catch(() => {});
    };
    send();
    const interval = setInterval(send, 100);
    return () => {
      clearInterval(interval);
      fetch('/send_command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: new URLSearchParams({ command: 'base -c {"T":1,"L":0,"R":0}' }).toString(),
      }).catch(() => {});
    };
  }, [enabled]);

  return (
    <>
      <div className={`banner ${enabled ? 'banner--warn' : 'banner--info'}`}>
        <span style={{ flex: 1 }}>
          {enabled ? '⚠ MANUAL OVERRIDE ACTIVE — autonomy paused' : 'AUTONOMY IN CONTROL · enable to take stick'}
        </span>
        <button
          className={`btn btn--sm ${enabled ? 'btn--danger' : 'btn--primary'}`}
          onClick={() => setEnabled(v => !v)}
        >
          {enabled ? 'RELEASE' : 'TAKE STICK'}
        </button>
      </div>

      <Panel title="JOYSTICK" right={<span className="mono dim">{cmd.mag > 0.02 ? `${(cmd.mag*100).toFixed(0)}%` : 'IDLE'}</span>}>
        <div style={{ opacity: enabled ? 1 : 0.35, pointerEvents: enabled ? 'auto' : 'none' }}>
          <Joystick onMove={setCmd} />
        </div>
        <div className="grid3" style={{ marginTop: 12 }}>
          <div>
            <div className="lab">LINEAR</div>
            <div className="val">{(cmd.linear * maxSpeed).toFixed(2)}</div>
            <div className="unit">m/s</div>
          </div>
          <div>
            <div className="lab">ANGULAR</div>
            <div className="val">{(cmd.angular * 1.5).toFixed(2)}</div>
            <div className="unit">rad/s</div>
          </div>
          <div>
            <div className="lab">MAX</div>
            <div className="val">{maxSpeed.toFixed(2)}</div>
            <div className="unit">m/s</div>
          </div>
        </div>
      </Panel>

      <Panel title="GOVERNOR">
        <div className="row__lab" style={{ marginBottom: 6 }}>SPEED LIMIT · {maxSpeed.toFixed(2)} m/s</div>
        <input
          type="range" min="0.1" max="1.5" step="0.05"
          value={maxSpeed}
          onChange={(e) => setMaxSpeed(+e.target.value)}
          style={{ width: '100%', accentColor: 'var(--accent)' }}
        />
        <div className="dim mono" style={{ fontSize: 9, marginTop: 8 }}>
          ▸ STAIR-PROTECTION · LIDAR auto-halts below 0.30m
        </div>
      </Panel>

      <Panel title="QUICK ACTIONS">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
          <button className="btn btn--sm">↺ RECENTER GIMBAL</button>
          <button className="btn btn--sm">⌂ RTH</button>
          <button className="btn btn--sm">◉ DOCK</button>
          <button className="btn btn--sm">✻ CALIBRATE IMU</button>
        </div>
      </Panel>
    </>
  );
}

// =====================================================================
// DIAG — every sensor expanded
// =====================================================================
function RoverWidget({ t }) {
  // hotspots positioned in % over the rover photo
  return (
    <div className="rover-widget">
      <div className="rover-widget__photo" />
      <div className="rover-widget__overlay" />

      {/* Gimbal cam — top center */}
      <div className="hotspot" style={{ top: '8%', left: '8%' }}>
        <span className="hotspot__dot" />
        <span className="hotspot__line" />
        <span className="hotspot__chip">GIMBAL · P{t.gimbal.pan.toFixed(0)}° T{t.gimbal.tilt.toFixed(0)}°</span>
      </div>

      {/* Left antenna */}
      <div className="hotspot hotspot--ok" style={{ top: '14%', right: '6%' }}>
        <span className="hotspot__chip">ANT-A · −{t.net.rssi * -1}dBm</span>
        <span className="hotspot__line" />
        <span className="hotspot__dot" />
      </div>

      {/* Front sensor bar */}
      <div className="hotspot" style={{ top: '54%', left: '6%' }}>
        <span className="hotspot__dot" />
        <span className="hotspot__line" />
        <span className="hotspot__chip">DEPTH · {t.lidar.minDist.toFixed(2)}m</span>
      </div>

      {/* Nosecone */}
      <div className="hotspot hotspot--warn" style={{ top: '78%', left: '40%' }}>
        <span className="hotspot__dot" />
        <span className="hotspot__chip" style={{ marginLeft: 6 }}>NOSECONE · GPS-RTK FIX</span>
      </div>

      {/* Right tracks */}
      <div className="hotspot" style={{ bottom: '10%', right: '6%' }}>
        <span className="hotspot__chip">DRIVE-R · {t.drive.R} pwm</span>
        <span className="hotspot__line" />
        <span className="hotspot__dot" />
      </div>
      {/* Left tracks */}
      <div className="hotspot" style={{ bottom: '22%', left: '6%' }}>
        <span className="hotspot__dot" />
        <span className="hotspot__line" />
        <span className="hotspot__chip">DRIVE-L · {t.drive.L} pwm</span>
      </div>
    </div>
  );
}

function DiagScreen({ state }) {
  const t = state.telemetry;
  return (
    <>
      <Panel title="ROVER" right={<span className="ok mono">ALL SYSTEMS</span>}>
        <RoverWidget t={t} />
      </Panel>

      <Panel title="VIDEO PIPELINE" right={<span className="ok mono">HEALTHY</span>}>
        <Row label="Device"      value={<span className="mono">/dev/video_usb</span>} />
        <Row label="Resolution"  value="1920×1080" />
        <Row label="Target FPS"  value="30.0" />
        <Row label="Avg FPS"     value={t.fps.toFixed(2)} color="var(--ok)" />
        <Row label="Frame age"   value={t.frameAgeS.toFixed(3)} unit="s" />
        <Row label="Frame count" value={t.frameCount.toLocaleString()} />
        <Row label="Capture err" value="0" color="var(--ok)" />
        <Row label="Uptime"      value={t.uptimeStr} />
      </Panel>

      <Panel title="IMU (BASE)">
        <div className="grid3">
          <div><div className="lab">ax</div><div className="val">{t.imu.ax.toFixed(2)}</div><div className="unit">m/s²</div></div>
          <div><div className="lab">ay</div><div className="val">{t.imu.ay.toFixed(2)}</div><div className="unit">m/s²</div></div>
          <div><div className="lab">az</div><div className="val">{t.imu.az.toFixed(2)}</div><div className="unit">m/s²</div></div>
          <div><div className="lab">gx</div><div className="val">{t.imu.gx.toFixed(2)}</div><div className="unit">°/s</div></div>
          <div><div className="lab">gy</div><div className="val">{t.imu.gy.toFixed(2)}</div><div className="unit">°/s</div></div>
          <div><div className="lab">gz</div><div className="val">{t.imu.gz.toFixed(2)}</div><div className="unit">°/s</div></div>
          <div><div className="lab">mx</div><div className="val">{t.imu.mx}</div><div className="unit">µT</div></div>
          <div><div className="lab">my</div><div className="val">{t.imu.my}</div><div className="unit">µT</div></div>
          <div><div className="lab">mz</div><div className="val">{t.imu.mz}</div><div className="unit">µT</div></div>
        </div>
        <div className="horizon" style={{ marginTop: 8 }}>
          <div className="horizon__line" style={{ transform: `rotate(${t.imu.ax * 4}deg) translateY(${t.imu.ay * 6}px)` }} />
          <div className="horizon__plane" />
        </div>
      </Panel>

      <Panel title="DRIVE">
        <Row label="L motor" value={t.drive.L} unit="pwm" />
        <Row label="R motor" value={t.drive.R} unit="pwm" />
        <Row label="Odom L"  value={t.drive.odl} unit="ticks" />
        <Row label="Odom R"  value={t.drive.odr} unit="ticks" />
        <Row label="Linear v"  value={t.velocity.linear.toFixed(2)} unit="m/s" />
        <Row label="Angular v" value={t.velocity.angular.toFixed(2)} unit="rad/s" />
      </Panel>

      <Panel title="NETWORK">
        <Row label="WiFi" value={
          <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
            <span className="sigbars">
              {[1,2,3,4].map(i => <span key={i} className={`sigbars__bar ${i <= t.net.bars ? 'on' : ''}`} />)}
            </span>
            {t.net.ssid}
          </span>
        } />
        <Row label="RSSI"     value={t.net.rssi} unit="dBm" />
        <Row label="Latency"  value={t.uplinkMs} unit="ms" />
        <Row label="↓ down"   value={t.net.down.toFixed(1)} unit="Mbps" />
        <Row label="↑ up"     value={t.net.up.toFixed(1)} unit="Mbps" />
        <Row label="API"      value="anthropic.com" color="var(--ok)" />
      </Panel>

      <Panel title="POSITION & MAP">
        <div className="minimap">
          <div className="minimap__compass-n">N ↑</div>
          <div className="minimap__scale">— 1m —</div>
          {t.breadcrumbs.map((p, i) => (
            <span key={i} className="minimap__breadcrumb"
              style={{ left: `${50 + p.x * 200}%`, top: `${50 - p.y * 200}%`, opacity: 0.15 + (i / t.breadcrumbs.length) * 0.5 }} />
          ))}
          <span className="minimap__waypoint" style={{ left: '70%', top: '30%' }} />
          <span className="minimap__waypoint" style={{ left: '35%', top: '60%' }} />
          <div
            className="minimap__rover"
            style={{
              left: `${50 + t.position.x * 200}%`,
              top: `${50 - t.position.y * 200}%`,
              transform: `translate(-50%, -50%) rotate(${t.heading}deg)`
            }}
          />
        </div>
      </Panel>
    </>
  );
}

Object.assign(window, { Panel, Row, Bar, StreamScreen, ClaudeLog, BagScreen, TeleopScreen, DiagScreen });
