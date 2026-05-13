/* global React */
const { useState, useEffect, useRef, useMemo } = React;

// =====================================================================
// HUD overlays drawn on top of the live video feed.
// =====================================================================

function CompassTape({ heading }) {
  // heading is degrees 0..360 (yaw). We render a virtual tape from -45..+45 around heading.
  const ticks = [];
  const tickStep = 5;
  const visibleHalf = 50;
  const start = Math.floor((heading - visibleHalf) / tickStep) * tickStep;
  for (let d = start; d <= heading + visibleHalf; d += tickStep) {
    const norm = ((d % 360) + 360) % 360;
    const isMajor = norm % 30 === 0;
    const cardinal = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[norm];
    ticks.push({ deg: d, norm, isMajor, cardinal });
  }
  // each tick = 36px wide; offset so heading sits in the middle
  const tickWidth = 36;
  const offsetPx = -(heading - start) * (tickWidth / tickStep);
  return (
    <div className="compass">
      <div className="compass__tape" style={{ transform: `translateX(${offsetPx}px)` }}>
        {ticks.map(t => (
          <span key={t.deg} className={`compass__tick ${t.isMajor ? 'major' : ''}`}>
            {t.cardinal || (t.isMajor ? String(Math.round(t.norm)).padStart(3, '0') : '·')}
          </span>
        ))}
      </div>
      <div className="compass__needle" />
    </div>
  );
}

function DetectionBox({ det, feedW, feedH, srcW, srcH }) {
  // det.bbox_px is in source-camera pixel space — whatever resolution
  // camera_owner is currently capturing at. Scale to the displayed feed
  // dimensions using live srcW/srcH from telemetry. Fall back to 1920×1080
  // if telemetry hasn't surfaced cam dims yet, so first-frame draw doesn't
  // collapse to a point. Note: when objectFit:cover is used on the underlying
  // <img>, the actual visible mapping is letterboxed/cropped depending on
  // aspect-ratio mismatch — boxes may drift slightly until we align
  // object-fit + scale math.
  const sourceW = srcW || 1920;
  const sourceH = srcH || 1080;
  const sx = feedW / sourceW, sy = feedH / sourceH;
  const x = (det.bbox_px.cx - det.bbox_px.w / 2) * sx;
  const y = (det.bbox_px.cy - det.bbox_px.h / 2) * sy;
  const w = det.bbox_px.w * sx;
  const h = det.bbox_px.h * sy;
  const isTracked = det.status === 'TRACKED';
  return (
    <div
      className={`bbox ${isTracked ? 'bbox--track' : ''} ${det.warn ? 'bbox--warn' : ''}`}
      style={{ left: x, top: y, width: w, height: h }}
    >
      <span className="bbox__corner tl" />
      <span className="bbox__corner tr" />
      <span className="bbox__corner bl" />
      <span className="bbox__corner br" />
      <span className="bbox__label">
        {det.class_id} · {det.distance_m?.toFixed(2)}m · {Math.round(det.score * 100)}%
      </span>
    </div>
  );
}

function Radar({ minDist, points }) {
  return (
    <div className="radar">
      <span className="radar__label">LIDAR</span>
      <span className="radar__center" />
      {points && points.map((p, i) => (
        <span
          key={i}
          style={{
            position: 'absolute',
            top: `${50 + p.dy}%`,
            left: `${50 + p.dx}%`,
            width: 3, height: 3,
            background: p.warn ? 'var(--warn)' : 'var(--accent)',
            borderRadius: '50%',
            opacity: 0.8,
            transform: 'translate(-50%, -50%)',
          }}
        />
      ))}
      <span className="radar__dist">{minDist.toFixed(2)}m</span>
    </div>
  );
}

// Polling live-feed component. We can't use multipart MJPEG via <img> —
// iOS Safari (and Chrome-on-iOS, which is also WKWebView) shows only the
// first frame and stops. Long-standing browser bug. Android handles it
// fine, but the operator console runs on Justin's iPhone, so we poll.
//
// Approach: pre-load the next frame off-screen via `new Image()`, swap
// the visible <img> src only after onload fires. Avoids flicker that the
// naive set-src-on-interval pattern produces (visible frame blanks
// during fetch). Backs off on error so a temporarily unhealthy /snapshot
// (camera_owner returning 503 stale) doesn't hammer the bridge.
function LiveFeedPoll({ rate = 1000, width = 640 }) {
  // 1 Hz target at 640-wide — matches what Haiku's heartbeat sees, costs
  // negligible Jetson CPU. The operator-console live feed is a glance-at-
  // it nice-to-have; we don't want to compete with yolo or with the
  // 30fps bag-record disk path that's the actual training-data value.
  // Real-time monitoring grade video belongs on the NVENC HLS / MSE path
  // (see task #60), not on cv2-encoded MJPEG polling.
  const buildUrl = () => `/snapshot?out_w=${width}&t=${Date.now()}`;
  const [displayedSrc, setDisplayedSrc] = useState(buildUrl);
  useEffect(() => {
    let cancelled = false;
    let timer = null;
    const loadNext = () => {
      if (cancelled) return;
      const next = buildUrl();
      const pre = new Image();
      pre.onload = () => {
        if (cancelled) return;
        setDisplayedSrc(next);
        timer = setTimeout(loadNext, rate);
      };
      pre.onerror = () => {
        if (cancelled) return;
        // 503 (stale frame) or network blip — back off so we're not
        // spamming during failure.
        timer = setTimeout(loadNext, rate * 5);
      };
      pre.src = next;
    };
    timer = setTimeout(loadNext, rate);
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, [rate, width]);
  return (
    <img
      className="hero__video-img"
      src={displayedSrc}
      alt="rover live feed"
      style={{
        position: 'absolute',
        inset: 0,
        width: '100%',
        height: '100%',
        objectFit: 'cover',
        display: 'block',
      }}
    />
  );
}


function VideoFeed({ src, mini, telemetry, showDetections, showHud, recording }) {
  const ref = useRef(null);
  const [feedSize, setFeedSize] = useState({ w: 400, h: 250 });
  useEffect(() => {
    if (!ref.current) return;
    const ro = new ResizeObserver(([e]) => {
      setFeedSize({ w: e.contentRect.width, h: e.contentRect.height });
    });
    ro.observe(ref.current);
    return () => ro.disconnect();
  }, []);

  return (
    <div ref={ref} className={`hero ${mini ? 'hero--mini' : ''}`}>
      {src === 'live' && (<>
        {/* 1 Hz at 640-wide — same view Haiku gets, negligible CPU cost,
            doesn't fight yolo or the 30fps bag-record disk path that's
            the actual training-data value. Smooth real-time video belongs
            on NVENC H.264 → HLS / MSE (see task #60), not here. */}
        <LiveFeedPoll />
        <div className="hero__noise" />
      </>)}
      {src === 'test' && <div className="hero__test-pattern" />}
      {src === 'black' && <div style={{ position: 'absolute', inset: 0, background: '#000' }} />}

      {showHud && <>
        <span className="hud-layer__bracket tl" />
        <span className="hud-layer__bracket tr" />
        <span className="hud-layer__bracket bl" />
        <span className="hud-layer__bracket br" />

        <div className="hud-layer__corner-text tl">
          CAM-01 · {telemetry.cameraW || 1920}×{telemetry.cameraH || 1080}<br/>
          <span style={{ color: 'var(--text-dim)' }}>{telemetry.fps.toFixed(1)} FPS</span>
        </div>
        <div className="hud-layer__corner-text tr">
          GIMBAL P{telemetry.gimbal.pan.toFixed(0)}° T{telemetry.gimbal.tilt.toFixed(0)}°<br/>
          <span style={{ color: 'var(--text-dim)' }}>UPLINK {telemetry.uplinkMs}ms</span>
        </div>
        <div className="hud-layer__corner-text bl">
          POS X{telemetry.position.x.toFixed(2)} Y{telemetry.position.y.toFixed(2)}<br/>
          <span style={{ color: 'var(--text-dim)' }}>HDG {telemetry.heading.toFixed(1)}°</span>
        </div>

        <CompassTape heading={telemetry.heading} />

        <div className="crosshair">
          <span className="crosshair__dot" />
        </div>

        {showDetections && telemetry.detections.map(d => (
          <DetectionBox key={d.id} det={d} feedW={feedSize.w} feedH={feedSize.h} srcW={telemetry.cameraW} srcH={telemetry.cameraH} />
        ))}

        <Radar minDist={telemetry.lidar.minDist} points={telemetry.lidar.points} />
      </>}

      {recording && (
        <div className="rec">
          <span className="rec__dot" />
          <span>REC · BAG</span>
        </div>
      )}
    </div>
  );
}

Object.assign(window, { CompassTape, DetectionBox, Radar, VideoFeed });
