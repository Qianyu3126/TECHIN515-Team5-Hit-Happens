/* Tennis Motion Analyzer — interactive prototype
   Flow: select motion → calibrate → start → swing → stop → analyze → result */

const { useState, useEffect, useRef, useCallback } = React;

/* ───────────────────────── icons (simple line glyphs) ───────────────────────── */
const I = {
  ball: (c, s = 22) => (
    <svg width={s} height={s} viewBox="0 0 24 24" fill="none">
      <circle cx="12" cy="12" r="9.2" fill={c} />
      <path d="M4.4 6.6c3.2 1.7 4.9 4.6 4.9 8.2 0 1.4-.3 2.7-.8 3.9" stroke="#0E1311" strokeWidth="1.4" strokeLinecap="round" opacity=".85"/>
      <path d="M19.6 6.6c-3.2 1.7-4.9 4.6-4.9 8.2 0 1.4.3 2.7.8 3.9" stroke="#0E1311" strokeWidth="1.4" strokeLinecap="round" opacity=".85"/>
    </svg>
  ),
  serve: (c) => (
    <svg width="26" height="26" viewBox="0 0 26 26" fill="none">
      <circle cx="18.5" cy="6.5" r="2.6" fill={c}/>
      <path d="M6 21l5.5-6.5 3 2.2L18 9" stroke={c} strokeWidth="2.1" strokeLinecap="round" strokeLinejoin="round"/>
      <path d="M14.5 8.8l3.5.2.2 3.4" stroke={c} strokeWidth="2.1" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  ),
  forehand: (c) => (
    <svg width="26" height="26" viewBox="0 0 26 26" fill="none">
      <path d="M5 18c2.5-7 6.5-10.5 11-10.5 2.2 0 3.8.9 4.7 2.3" stroke={c} strokeWidth="2.1" strokeLinecap="round"/>
      <path d="M21 9.8l-.2-3.6M21 9.8l-3.5.4" stroke={c} strokeWidth="2.1" strokeLinecap="round" strokeLinejoin="round"/>
      <circle cx="6" cy="19.5" r="1.8" fill={c}/>
    </svg>
  ),
  backhand: (c) => (
    <svg width="26" height="26" viewBox="0 0 26 26" fill="none">
      <path d="M21 18c-2.5-7-6.5-10.5-11-10.5-2.2 0-3.8.9-4.7 2.3" stroke={c} strokeWidth="2.1" strokeLinecap="round"/>
      <path d="M5 9.8l.2-3.6M5 9.8l3.5.4" stroke={c} strokeWidth="2.1" strokeLinecap="round" strokeLinejoin="round"/>
      <circle cx="20" cy="19.5" r="1.8" fill={c}/>
    </svg>
  ),
  target: (c) => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
      <circle cx="12" cy="12" r="8.2" stroke={c} strokeWidth="2"/>
      <circle cx="12" cy="12" r="3.4" stroke={c} strokeWidth="2"/>
      <path d="M12 1.5v3M12 19.5v3M1.5 12h3M19.5 12h3" stroke={c} strokeWidth="2" strokeLinecap="round"/>
    </svg>
  ),
  play: (c) => (
    <svg width="18" height="18" viewBox="0 0 24 24"><path d="M7 4.5l13 7.5-13 7.5z" fill={c}/></svg>
  ),
  stop: (c) => (
    <svg width="16" height="16" viewBox="0 0 24 24"><rect x="5" y="5" width="14" height="14" rx="3" fill={c}/></svg>
  ),
  reset: (c) => (
    <svg width="19" height="19" viewBox="0 0 24 24" fill="none">
      <path d="M20 12a8 8 0 1 1-2.3-5.6" stroke={c} strokeWidth="2" strokeLinecap="round"/>
      <path d="M20 4v4h-4" stroke={c} strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  ),
  node: (c, s = 16) => (
    <svg width={s} height={s} viewBox="0 0 24 24" fill="none">
      <circle cx="12" cy="12" r="4.4" fill={c}/>
      <circle cx="12" cy="12" r="8.4" stroke={c} strokeWidth="1.6" opacity=".45"/>
    </svg>
  ),
  bolt: (c) => (
    <svg width="13" height="13" viewBox="0 0 24 24"><path d="M13 2L4 14h6l-1 8 9-12h-6z" fill={c}/></svg>
  ),
  check: (c) => (
    <svg width="15" height="15" viewBox="0 0 24 24" fill="none"><path d="M4 12.5l5 5 11-11" stroke={c} strokeWidth="2.6" strokeLinecap="round" strokeLinejoin="round"/></svg>
  ),
  warn: (c) => (
    <svg width="15" height="15" viewBox="0 0 24 24" fill="none"><path d="M12 3l9.5 16.5h-19z" stroke={c} strokeWidth="2.2" strokeLinejoin="round"/><path d="M12 10v4.2" stroke={c} strokeWidth="2.2" strokeLinecap="round"/><circle cx="12" cy="17.4" r="1.2" fill={c}/></svg>
  ),
};

/* ───────────────────────── data ───────────────────────── */
const MOTIONS = [
  { id: 'serve', label: 'Serve', icon: I.serve },
  { id: 'forehand', label: 'Forehand', icon: I.forehand },
  { id: 'backhand', label: 'Backhand', icon: I.backhand },
];

const SENSORS = [
  { id: 'waist', label: 'Waist', sub: 'Core' },
  { id: 'rua', label: 'R. Arm', sub: 'Upper' },
  { id: 'rfa', label: 'R. Fore', sub: 'Forearm' },
  { id: 'lua', label: 'L. Arm', sub: 'Upper' },
  { id: 'lfa', label: 'L. Fore', sub: 'Forearm' },
];

const ANALYSIS = {
  serve: [
    { name: 'Flat Serve', conf: 93, tone: 'positive',
      fb: 'Clean kinetic chain — legs drove into a full overhead extension. Toss and contact point stayed well aligned.' },
    { name: 'Flat Serve', conf: 76, tone: 'warning',
      fb: 'Elbow dropped just before contact, bleeding racket-head speed. Lead with the elbow and snap up at full reach.' },
    { name: 'Kick Serve', conf: 68, tone: 'warning',
      fb: 'Trunk rotation lagged behind the arm. Coil the hips earlier and let the shoulders follow to add power.' },
  ],
  forehand: [
    { name: 'Topspin Forehand', conf: 95, tone: 'positive',
      fb: 'Excellent low-to-high path with strong hip rotation. Wrist stayed relaxed through contact for clean spin.' },
    { name: 'Forehand', conf: 79, tone: 'warning',
      fb: 'Contact point was slightly late, behind your hip. Meet the ball further in front for more depth and control.' },
    { name: 'Flat Forehand', conf: 71, tone: 'warning',
      fb: 'Limited shoulder turn on the backswing. Rotate further to load the stroke and build racket-head speed.' },
  ],
  backhand: [
    { name: 'Two-Hand Backhand', conf: 91, tone: 'positive',
      fb: 'Solid unit turn and balanced base. Both arms drove evenly through the line of the shot.' },
    { name: 'One-Hand Backhand', conf: 77, tone: 'warning',
      fb: 'Off arm opened too early, pulling you off balance. Keep the non-dominant arm back through contact.' },
    { name: 'Backhand', conf: 69, tone: 'warning',
      fb: 'Weight stayed on the back foot. Step into the shot and transfer forward through contact for depth.' },
  ],
};

const SEED_RECENT = [
  { shot: 'Forehand', name: 'Topspin Forehand', conf: 95, tone: 'positive', time: '2:41 PM' },
  { shot: 'Serve', name: 'Flat Serve', conf: 74, tone: 'warning', time: '2:36 PM' },
  { shot: 'Backhand', name: 'Two-Hand Backhand', conf: 88, tone: 'positive', time: '2:30 PM' },
];

const confColor = (c) => (c >= 85 ? 'var(--lime)' : c >= 72 ? 'var(--amber)' : 'var(--red)');
const nowLabel = () => {
  const d = new Date();
  let h = d.getHours(); const m = d.getMinutes();
  const ap = h >= 12 ? 'PM' : 'AM'; h = h % 12 || 12;
  return `${h}:${String(m).padStart(2, '0')} ${ap}`;
};

/* ───────────────────────── small pieces ───────────────────────── */
function SectionLabel({ children, right }) {
  return (
    <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', margin: '0 0 11px' }}>
      <span style={{ fontSize: 11.5, fontWeight: 700, letterSpacing: 1.4, textTransform: 'uppercase', color: 'var(--muted)' }}>{children}</span>
      {right}
    </div>
  );
}

function MotionTab({ m, active, onClick }) {
  return (
    <button className="tap" onClick={onClick} style={{
      flex: 1, border: 'none', cursor: 'pointer', borderRadius: 14, padding: '9px 6px 8px',
      display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 4,
      background: active ? 'var(--lime)' : 'var(--surface2)',
      boxShadow: active ? '0 6px 16px -8px rgba(207,242,62,.5)' : 'inset 0 0 0 1px var(--line)',
      color: active ? '#0E1311' : 'var(--text)',
    }}>
      <div style={{ height: 20, display: 'flex', alignItems: 'center', transform: 'scale(.78)' }}>{m.icon(active ? '#0E1311' : 'var(--lime)')}</div>
      <span style={{ fontSize: 12.5, fontWeight: 700, letterSpacing: .2 }}>{m.label}</span>
    </button>
  );
}

function Ctrl({ kind, label, disabled, hidden, onClick, flex = 1 }) {
  const styles = {
    calibrate: { bg: 'var(--surface3)', fg: 'var(--text)', ic: 'var(--lime)', ring: 'inset 0 0 0 1px var(--line2)' },
    start: { bg: 'var(--lime)', fg: '#0E1311', ic: '#0E1311', ring: '0 8px 20px -10px rgba(207,242,62,.7)' },
    stop: { bg: 'var(--red)', fg: '#190B08', ic: '#190B08', ring: '0 8px 20px -10px rgba(242,106,75,.7)' },
    reset: { bg: 'var(--surface2)', fg: 'var(--muted)', ic: 'var(--muted)', ring: 'inset 0 0 0 1px var(--line)' },
  }[kind];
  const icon = { calibrate: I.target, start: I.play, stop: I.stop, reset: I.reset }[kind];
  return (
    <button className="tap" disabled={disabled} onClick={onClick} style={{
      visibility: hidden ? 'hidden' : 'visible', flex,
      border: 'none', cursor: disabled ? 'default' : 'pointer', borderRadius: 14,
      padding: '0 8px', height: 52, minWidth: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 7,
      background: disabled ? 'var(--surface2)' : styles.bg,
      boxShadow: disabled ? 'inset 0 0 0 1px var(--line)' : styles.ring,
      color: disabled ? 'var(--muted2)' : styles.fg,
      opacity: disabled ? .5 : 1,
      animation: kind === 'stop' ? 'recRing 1.4s ease-out infinite' : 'none',
    }}>
      <span style={{ display: 'flex', flexShrink: 0 }}>{icon(disabled ? 'var(--muted2)' : styles.ic)}</span>
      <span style={{ fontSize: 13.5, fontWeight: 700, letterSpacing: .1, whiteSpace: 'nowrap' }}>{label}</span>
    </button>
  );
}

function SensorChip({ s, online }) {
  return (
    <div style={{ flex: 1, minWidth: 0, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 4, padding: '2px 0' }}>
      <span style={{ position: 'relative', display: 'flex' }}>
        {I.node(online ? 'var(--lime)' : 'var(--muted2)', 14)}
        <span style={{
          position: 'absolute', top: -2, right: -3, width: 5, height: 5, borderRadius: 99,
          background: online ? 'var(--lime)' : 'var(--muted2)',
          boxShadow: online ? '0 0 4px rgba(207,242,62,.8)' : 'none',
        }} />
      </span>
      <span style={{ fontSize: 8.5, fontWeight: 700, color: online ? 'var(--muted)' : 'var(--muted2)', whiteSpace: 'nowrap', letterSpacing: .2 }}>{s.label}</span>
    </div>
  );
}

function ResultCard({ r }) {
  const pos = r.tone === 'positive';
  const accent = pos ? 'var(--lime)' : 'var(--amber)';
  const dim = pos ? 'var(--lime-dim)' : 'var(--amber-dim)';
  return (
    <div style={{
      borderRadius: 22, padding: 18, position: 'relative', overflow: 'hidden',
      background: 'linear-gradient(165deg, var(--surface3), var(--surface))',
      boxShadow: `inset 0 0 0 1px ${pos ? 'rgba(207,242,62,.30)' : 'rgba(242,178,62,.30)'}, 0 18px 36px -22px rgba(0,0,0,.9)`,
      opacity: 1,
    }}>
      {/* glow */}
      <div style={{ position: 'absolute', top: -60, right: -40, width: 160, height: 160, borderRadius: '50%', background: dim, filter: 'blur(20px)', pointerEvents: 'none' }} />
      <div style={{ position: 'relative' }}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12 }}>
          <span style={{
            display: 'inline-flex', alignItems: 'center', gap: 6, padding: '5px 10px 5px 8px', borderRadius: 99,
            background: dim, color: accent, fontSize: 11, fontWeight: 800, letterSpacing: .5, textTransform: 'uppercase',
          }}>
            {pos ? I.check(accent) : I.warn(accent)}
            {pos ? 'Good Form' : 'Needs Work'}
          </span>
          <span style={{ fontSize: 10.5, fontWeight: 700, color: 'var(--muted)', letterSpacing: .4, textTransform: 'uppercase' }}>{r.shot}</span>
        </div>

        <div style={{ display: 'flex', alignItems: 'flex-end', justifyContent: 'space-between', gap: 12 }}>
          <div style={{ minWidth: 0 }}>
            <div style={{ fontSize: 10.5, fontWeight: 700, color: 'var(--muted)', letterSpacing: 1, textTransform: 'uppercase', marginBottom: 3 }}>Detected</div>
            <div className="mono" style={{ fontSize: 25, fontWeight: 600, color: 'var(--text)', lineHeight: 1.05, letterSpacing: -.5 }}>{r.name}</div>
          </div>
          <div style={{ textAlign: 'right', flexShrink: 0 }}>
            <div className="mono" style={{ fontSize: 30, fontWeight: 600, color: confColor(r.conf), lineHeight: 1 }}>{r.conf}<span style={{ fontSize: 16 }}>%</span></div>
            <div style={{ fontSize: 9.5, fontWeight: 700, color: 'var(--muted)', letterSpacing: 1, textTransform: 'uppercase', marginTop: 1 }}>Confidence</div>
          </div>
        </div>

        {/* confidence meter */}
        <div style={{ height: 5, borderRadius: 99, background: 'rgba(255,255,255,.06)', margin: '13px 0 14px', overflow: 'hidden' }}>
          <div style={{ height: '100%', width: r.conf + '%', borderRadius: 99, background: confColor(r.conf), transition: 'width .7s cubic-bezier(.2,.8,.25,1)' }} />
        </div>

        <div style={{ display: 'flex', gap: 9 }}>
          <div style={{ width: 3, borderRadius: 3, background: accent, flexShrink: 0 }} />
          <p style={{ margin: 0, fontSize: 13.5, lineHeight: 1.5, color: '#C8D2C5', textWrap: 'pretty' }}>{r.fb}</p>
        </div>
      </div>
    </div>
  );
}

function AnalysisStage({ phase, elapsed, result, calibrated, motionLabel, calProgress }) {
  // result is the focus → big, distinct hero stage
  if (phase === 'done' && result) return <ResultCard r={result} />;

  if (phase === 'recording') {
    return (
      <div style={{
        minHeight: 172, borderRadius: 22, padding: 22, position: 'relative', overflow: 'hidden',
        background: 'linear-gradient(160deg, rgba(242,106,75,.16), var(--surface))',
        boxShadow: 'inset 0 0 0 1px rgba(242,106,75,.32)',
        display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 8,
      }}>
        <span style={{ display: 'inline-flex', alignItems: 'center', gap: 7, color: 'var(--red)', fontSize: 11.5, fontWeight: 800, letterSpacing: 1.2, textTransform: 'uppercase' }}>
          <span style={{ width: 8, height: 8, borderRadius: 99, background: 'var(--red)', animation: 'dotBlink 1s infinite' }} />
          Recording {motionLabel}
        </span>
        <div className="mono" style={{ fontSize: 56, fontWeight: 600, color: 'var(--text)', lineHeight: 1, letterSpacing: -1.5 }}>
          {elapsed.toFixed(1)}<span style={{ fontSize: 24, color: 'var(--red)' }}>s</span>
        </div>
        <div style={{ fontSize: 12.5, color: 'var(--muted)' }}>Press Stop after your follow-through</div>
        <div style={{ position: 'absolute', left: 0, right: 0, bottom: 0, height: 4, background: 'rgba(242,106,75,.15)', overflow: 'hidden' }}>
          <div style={{ position: 'absolute', top: 0, bottom: 0, width: '40%', background: 'linear-gradient(90deg, transparent, var(--red), transparent)', animation: 'barShine 1.1s linear infinite' }} />
        </div>
      </div>
    );
  }

  if (phase === 'analyzing') {
    return (
      <div style={{
        minHeight: 172, borderRadius: 22, padding: 22,
        background: 'linear-gradient(160deg, var(--amber-dim), var(--surface))',
        boxShadow: 'inset 0 0 0 1px rgba(242,178,62,.3)',
        display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 16,
      }}>
        <span style={{ width: 36, height: 36, borderRadius: 99, border: '3px solid rgba(242,178,62,.25)', borderTopColor: 'var(--amber)', animation: 'spin .8s linear infinite' }} />
        <div style={{ textAlign: 'center' }}>
          <div style={{ fontSize: 15.5, fontWeight: 700, color: 'var(--amber)' }}>Analyzing swing…</div>
          <div style={{ fontSize: 12.5, color: 'var(--muted)', marginTop: 4 }}>Running motion model on captured IMU data</div>
        </div>
      </div>
    );
  }

  if (phase === 'calibrating') {
    return (
      <div style={{
        minHeight: 172, borderRadius: 22, padding: 22,
        background: 'linear-gradient(160deg, var(--amber-dim), var(--surface))',
        boxShadow: 'inset 0 0 0 1px rgba(242,178,62,.3)',
        display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 16,
      }}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
          <span style={{ display: 'inline-flex', alignItems: 'center', gap: 7, color: 'var(--amber)', fontSize: 11.5, fontWeight: 800, letterSpacing: 1.2, textTransform: 'uppercase' }}>
            <span style={{ width: 8, height: 8, borderRadius: 99, background: 'var(--amber)', animation: 'dotBlink 1s infinite' }} />
            Calibrating
          </span>
          <span className="mono" style={{ fontSize: 30, fontWeight: 600, color: 'var(--amber)', lineHeight: 1, letterSpacing: -.5 }}>{Math.round(calProgress)}<span style={{ fontSize: 16 }}>%</span></span>
        </div>
        <div style={{ fontSize: 13, color: 'var(--muted)', lineHeight: 1.4 }}>Hold still — aligning sensor orientation…</div>
        <div style={{ height: 8, borderRadius: 99, background: 'rgba(255,255,255,.06)', overflow: 'hidden' }}>
          <div style={{ height: '100%', width: calProgress + '%', borderRadius: 99, background: 'linear-gradient(90deg, var(--amber), #F2C96A)', transition: 'width .12s linear', boxShadow: '0 0 10px rgba(242,178,62,.5)' }} />
        </div>
      </div>
    );
  }

  // idle / ready → inviting placeholder
  const ready = calibrated;
  return (
    <div style={{
      minHeight: 172, borderRadius: 22, padding: 22,
      background: 'var(--surface)', boxShadow: 'inset 0 0 0 1px var(--line)',
      display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 13, textAlign: 'center',
    }}>
      <div style={{ opacity: .45, filter: 'grayscale(.3)' }}>{I.ball('var(--muted)', 42)}</div>
      <div>
        <div style={{ fontSize: 14.5, fontWeight: 700, color: 'var(--text)' }}>{ready ? 'Ready when you are' : 'No swing recorded yet'}</div>
        <div style={{ fontSize: 12.5, lineHeight: 1.5, color: 'var(--muted)', marginTop: 5, maxWidth: 240, textWrap: 'pretty' }}>
          {ready ? 'Press Start, swing, then Stop — your form breakdown lands here instantly.' : 'Calibrate the sensors, then record a swing to get instant AI form feedback.'}
        </div>
      </div>
    </div>
  );
}

function RecentRow({ r }) {
  const accent = r.tone === 'positive' ? 'var(--lime)' : 'var(--amber)';
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '11px 0' }}>
      <span style={{ width: 8, height: 8, borderRadius: 99, background: accent, flexShrink: 0, boxShadow: `0 0 6px ${r.tone === 'positive' ? 'rgba(207,242,62,.6)' : 'rgba(242,178,62,.5)'}` }} />
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontSize: 14, fontWeight: 700, color: 'var(--text)', whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{r.name}</div>
        <div style={{ fontSize: 11, fontWeight: 600, color: 'var(--muted)', marginTop: 1 }}>{r.shot} · {r.time}</div>
      </div>
      <span className="mono" style={{ fontSize: 15, fontWeight: 600, color: confColor(r.conf) }}>{r.conf}%</span>
    </div>
  );
}

/* ───────────────────────── main ───────────────────────── */
function App() {
  const [motion, setMotion] = useState('serve');
  // phases: idle | calibrating | ready | recording | analyzing | done
  const [phase, setPhase] = useState('idle');
  const [calibrated, setCalibrated] = useState(false);
  const [calProgress, setCalProgress] = useState(0);
  const [elapsed, setElapsed] = useState(0);
  const [result, setResult] = useState(null);
  const [recent, setRecent] = useState(SEED_RECENT);
  // L. Forearm starts offline → comes online after calibration
  const [offline, setOffline] = useState({ lfa: true });

  const timers = useRef([]);
  const addTimer = (t) => { timers.current.push(t); return t; };
  const clearTimers = () => { timers.current.forEach(clearInterval); timers.current.forEach(clearTimeout); timers.current = []; };

  const allOnline = SENSORS.every((s) => !offline[s.id]);

  /* calibration */
  const calibrate = () => {
    if (phase === 'calibrating' || phase === 'recording' || phase === 'analyzing') return;
    setPhase('calibrating'); setCalProgress(0); setResult(null);
    const start = Date.now(); const dur = 2400;
    const iv = addTimer(setInterval(() => {
      const p = Math.min(100, ((Date.now() - start) / dur) * 100);
      setCalProgress(p);
      if (p >= 100) clearInterval(iv);
    }, 40));
    addTimer(setTimeout(() => { setOffline({}); }, 1500)); // last node connects
    addTimer(setTimeout(() => { setCalibrated(true); setPhase('ready'); }, dur));
  };

  /* recording */
  const startRec = () => {
    if (!calibrated || phase === 'recording') return;
    setPhase('recording'); setElapsed(0); setResult(null);
    const start = Date.now();
    addTimer(setInterval(() => setElapsed((Date.now() - start) / 1000), 100));
  };

  const stopRec = () => {
    if (phase !== 'recording') return;
    clearTimers();
    setPhase('analyzing');
    addTimer(setTimeout(() => {
      const pool = ANALYSIS[motion];
      const pick = pool[Math.floor(Math.random() * pool.length)];
      const shotLabel = MOTIONS.find((m) => m.id === motion).label;
      const res = { ...pick, shot: shotLabel };
      setResult(res); setPhase('done');
      setRecent((prev) => [{ shot: shotLabel, name: pick.name, conf: pick.conf, tone: pick.tone, time: nowLabel() }, ...prev].slice(0, 3));
    }, 1600));
  };

  const reset = () => {
    clearTimers();
    setPhase('idle'); setCalibrated(false); setCalProgress(0); setElapsed(0); setResult(null);
  };

  useEffect(() => () => clearTimers(), []);
  useEffect(() => { window.__fitStage && window.__fitStage(); });

  /* status text */
  const statusMap = {
    idle: { tag: 'Idle', tone: 'muted', msg: 'Select a shot, then calibrate the sensors to begin.' },
    calibrating: { tag: 'Calibrating', tone: 'amber', msg: 'Hold still — aligning sensor orientation…' },
    ready: { tag: 'Ready', tone: 'lime', msg: 'Calibrated. Press Start, then swing when ready.' },
    recording: { tag: 'Recording', tone: 'red', msg: 'Capturing swing — press Stop after your follow-through.' },
    analyzing: { tag: 'Analyzing', tone: 'amber', msg: 'Running motion model on the captured swing…' },
    done: { tag: 'Done', tone: 'lime', msg: 'Analysis complete. Review feedback below or reset.' },
  };
  const st = statusMap[phase];
  const tagColors = {
    muted: ['var(--surface3)', 'var(--muted)'],
    lime: ['var(--lime-dim)', 'var(--lime)'],
    amber: ['var(--amber-dim)', 'var(--amber)'],
    red: ['var(--red-dim)', 'var(--red)'],
  }[st.tone];

  const recording = phase === 'recording';
  const analyzing = phase === 'analyzing';

  return (
    <div className="noscroll" style={{
        height: '100%', overflowY: 'auto', background: 'var(--bg)',
        padding: '60px 16px 40px', display: 'flex', flexDirection: 'column', gap: 18,
        color: 'var(--text)',
      }}>

        {/* ── header ── */}
        <header style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 11 }}>
            <div style={{ width: 38, height: 38, borderRadius: 12, background: 'var(--surface2)', display: 'flex', alignItems: 'center', justifyContent: 'center', boxShadow: 'inset 0 0 0 1px var(--line)' }}>
              {I.ball('var(--lime)', 23)}
            </div>
            <div style={{ lineHeight: 1.1 }}>
              <div style={{ fontSize: 16.5, fontWeight: 800, letterSpacing: -.2 }}>Tennis Motion</div>
              <div style={{ fontSize: 11, fontWeight: 600, color: 'var(--muted)', letterSpacing: .3 }}>IMU Swing Analyzer</div>
            </div>
          </div>
          <div style={{
            display: 'flex', alignItems: 'center', gap: 6, padding: '6px 10px 6px 9px', borderRadius: 99,
            background: allOnline ? 'var(--lime-dim)' : 'var(--amber-dim)', boxShadow: `inset 0 0 0 1px ${allOnline ? 'rgba(207,242,62,.25)' : 'rgba(242,178,62,.25)'}`,
          }}>
            <span style={{ width: 7, height: 7, borderRadius: 99, background: allOnline ? 'var(--lime)' : 'var(--amber)', animation: allOnline ? 'livePulse 1.8s ease-out infinite' : 'dotBlink 1s infinite' }} />
            <span style={{ fontSize: 10.5, fontWeight: 800, letterSpacing: .8, color: allOnline ? 'var(--lime)' : 'var(--amber)' }}>{allOnline ? 'LIVE' : 'SYNC'}</span>
          </div>
        </header>

        {/* ══ OPERATION CONSOLE — grouped, lower visual priority ══ */}
        <div style={{ borderRadius: 24, background: 'var(--surface)', boxShadow: 'inset 0 0 0 1px var(--line)', padding: 14, display: 'flex', flexDirection: 'column', gap: 13 }}>
          {/* shot type */}
          <div>
            <SectionLabel>Shot Type</SectionLabel>
            <div style={{ display: 'flex', gap: 8 }}>
              {MOTIONS.map((m) => (
                <MotionTab key={m.id} m={m} active={motion === m.id} onClick={() => { if (!recording && !analyzing) setMotion(m.id); }} />
              ))}
            </div>
          </div>

          {/* controls — compact single row */}
          <div>
            <SectionLabel>Controls</SectionLabel>
            <div style={{ display: 'flex', gap: 8 }}>
              <Ctrl kind="calibrate" label={phase === 'calibrating' ? Math.round(calProgress) + '%' : 'Calibrate'} onClick={calibrate} disabled={recording || analyzing} flex={1.05} />
              {recording
                ? <Ctrl kind="stop" label="Stop" onClick={stopRec} flex={1.3} />
                : <Ctrl kind="start" label="Start" onClick={startRec} disabled={!calibrated || analyzing} flex={1.3} />}
              <Ctrl kind="reset" label="Reset" onClick={reset} flex={0.9} />
            </div>
          </div>

          {/* system status: sensor strip + state line */}
          <div style={{ borderTop: '1px solid var(--line)', paddingTop: 12, display: 'flex', flexDirection: 'column', gap: 11 }}>
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
              <span style={{ fontSize: 10, fontWeight: 700, letterSpacing: 1.2, textTransform: 'uppercase', color: 'var(--muted2)' }}>Sensor Nodes</span>
              <span style={{ fontSize: 10, fontWeight: 700, color: allOnline ? 'var(--lime)' : 'var(--amber)' }}>{SENSORS.filter((s) => !offline[s.id]).length}/5 online</span>
            </div>
            <div style={{ display: 'flex', gap: 4 }}>
              {SENSORS.map((s) => (<SensorChip key={s.id} s={s} online={!offline[s.id]} />))}
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: 9, paddingTop: 3 }}>
              <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6, padding: '4px 9px', borderRadius: 8, background: tagColors[0], color: tagColors[1], fontSize: 11, fontWeight: 800, letterSpacing: .3, flexShrink: 0 }}>
                <span style={{ width: 6, height: 6, borderRadius: 99, background: tagColors[1], animation: (recording || analyzing || phase === 'calibrating') ? 'dotBlink 1s infinite' : 'none' }} />
                {st.tag}
              </span>
              <span style={{ fontSize: 11.5, color: 'var(--muted)', lineHeight: 1.35, textWrap: 'pretty' }}>{st.msg}</span>
            </div>
          </div>
        </div>

        {/* ══ ANALYSIS — the hero / focus zone ══ */}
        <section>
          <div style={{ display: 'flex', alignItems: 'center', gap: 9, margin: '2px 2px 12px' }}>
            <span style={{ width: 4, height: 17, borderRadius: 2, background: 'var(--lime)' }} />
            <span style={{ fontSize: 15, fontWeight: 800, letterSpacing: .2, color: 'var(--text)' }}>Analysis</span>
            <span style={{ flex: 1 }} />
            {result && phase === 'done' && (
              <span style={{ fontSize: 11, fontWeight: 700, color: 'var(--muted)', letterSpacing: .3 }}>Latest swing</span>
            )}
          </div>
          <AnalysisStage phase={phase} elapsed={elapsed} result={result} calibrated={calibrated} calProgress={calProgress} motionLabel={MOTIONS.find((m) => m.id === motion).label} />
        </section>

        {/* ── recent ── */}
        <section>
          <SectionLabel>Recent Results</SectionLabel>
          <div style={{ borderRadius: 20, padding: '4px 16px', background: 'var(--surface)', boxShadow: 'inset 0 0 0 1px var(--line)' }}>
            {recent.map((r, i) => (
              <div key={i} style={{ borderBottom: i < recent.length - 1 ? '1px solid var(--line)' : 'none' }}>
                <RecentRow r={r} />
              </div>
            ))}
          </div>
        </section>

        <div style={{ height: 6 }} />
      </div>
  );
}

function Root() {
  const [splashDone, setSplashDone] = useState(false);
  const Splash = window.Splash;
  return (
    <IOSDevice width={390} height={844} dark>
      <div style={{ position: 'relative', height: '100%' }}>
        <App />
        {!splashDone && Splash && <Splash onDone={() => setSplashDone(true)} />}
      </div>
    </IOSDevice>
  );
}

const __root = ReactDOM.createRoot(document.getElementById('stage'));
__root.render(<Root />);
window.__replaySplash = () => __root.render(<Root key={Date.now()} />);
setTimeout(() => window.__fitStage && window.__fitStage(), 100);
