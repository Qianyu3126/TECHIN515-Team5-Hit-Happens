/* Splash — JS/rAF-driven motion intro for Tennis Motion Analyzer.
   A tennis ball arcs across, tracing its path, impacts center, then the
   wordmark rises in and the whole screen slides away to reveal the app.
   Driven by requestAnimationFrame (not CSS keyframes) so it always plays. */

(function () {
  const { useState, useEffect, useRef } = React;

  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const easeOut = (p) => 1 - Math.pow(1 - p, 3);
  const easeInOut = (p) => (p < 0.5 ? 4 * p * p * p : 1 - Math.pow(-2 * p + 2, 3) / 2);

  // arc geometry (device 390×844)
  const S = [-44, 760], C = [44, 150], E = [195, 360];
  const bez = (p) => {
    const u = 1 - p;
    return [u * u * S[0] + 2 * u * p * C[0] + p * p * E[0],
            u * u * S[1] + 2 * u * p * C[1] + p * p * E[1]];
  };

  function Ball({ cx, cy, r, sx = 1, sy = 1, opacity = 1 }) {
    const seamL = `M ${-0.55 * r} ${-0.8 * r} C ${0.2 * r} ${-0.45 * r}, ${0.2 * r} ${0.45 * r}, ${-0.55 * r} ${0.8 * r}`;
    const seamR = `M ${0.55 * r} ${-0.8 * r} C ${-0.2 * r} ${-0.45 * r}, ${-0.2 * r} ${0.45 * r}, ${0.55 * r} ${0.8 * r}`;
    return (
      <g transform={`translate(${cx} ${cy}) scale(${sx} ${sy})`} opacity={opacity}>
        <circle r={r} fill="var(--lime)" />
        <path d={seamL} fill="none" stroke="#0E1311" strokeWidth={0.13 * r} strokeLinecap="round" opacity="0.85" />
        <path d={seamR} fill="none" stroke="#0E1311" strokeWidth={0.13 * r} strokeLinecap="round" opacity="0.85" />
      </g>
    );
  }

  function Splash({ onDone }) {
    const [t, setT] = useState(0);
    const start = useRef(null);
    const raf = useRef(null);
    const done = useRef(false);

    useEffect(() => {
      const startMs = Date.now();
      const id = setInterval(() => {
        const e = Date.now() - startMs;
        setT(e);
        if (e >= 2050) clearInterval(id);
      }, 16);
      return () => clearInterval(id);
    }, []);

    useEffect(() => {
      if (t >= 1980 && !done.current) { done.current = true; onDone && onDone(); }
    }, [t]);

    const skip = () => { if (!done.current) { done.current = true; onDone && onDone(); } };

    // ── derive frame ──
    const bp = easeOut(clamp(t / 720, 0, 1));         // arc progress
    const [bx, by] = bez(bp);
    const landP = clamp((t - 660) / 240, 0, 1);        // settle
    const squash = Math.sin(landP * Math.PI);          // 0→1→0
    const r = 15 + landP * 9;                          // grow into logo
    const sx = 1 + squash * 0.16, sy = 1 - squash * 0.16;

    const trailFade = 1 - easeOut(clamp((t - 820) / 380, 0, 1));
    const ringP = clamp((t - 700) / 380, 0, 1);
    const ringScale = 0.35 + easeOut(ringP) * 1.5;
    const ringOp = ringP > 0 && ringP < 1 ? (1 - ringP) * 0.6 : 0;

    const l1 = easeOut(clamp((t - 820) / 360, 0, 1));
    const l2 = easeOut(clamp((t - 950) / 360, 0, 1));
    const tg = clamp((t - 1160) / 320, 0, 1);
    const flightOn = t < 760;

    const exP = easeInOut(clamp((t - 1560) / 420, 0, 1));
    const exitY = -exP * 46;
    const exitOp = 1 - exP;

    const W = 390, H = 844;

    return (
      <div onClick={skip} style={{
        position: 'absolute', inset: 0, zIndex: 40, overflow: 'hidden', cursor: 'pointer',
        backgroundColor: '#0B0F0C',
        backgroundImage: 'radial-gradient(125% 95% at 50% 40%, #141C16 0%, #0E1311 58%, #080B09 100%)',
        transform: `translateY(${exitY}px)`, opacity: exitOp,
      }}>
        {/* faint court baseline accents */}
        <div style={{ position: 'absolute', left: 28, right: 28, top: 360, height: 1, background: 'rgba(207,242,62,0.07)' }} />

        <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`} style={{ position: 'absolute', inset: 0 }}>
          <defs>
            <linearGradient id="trailG" x1={S[0]} y1={S[1]} x2={E[0]} y2={E[1]} gradientUnits="userSpaceOnUse">
              <stop offset="0" stopColor="var(--lime)" stopOpacity="0" />
              <stop offset="0.75" stopColor="var(--lime)" stopOpacity="0.85" />
              <stop offset="1" stopColor="var(--lime)" stopOpacity="1" />
            </linearGradient>
            <filter id="ballGlow" x="-60%" y="-60%" width="220%" height="220%">
              <feGaussianBlur stdDeviation="7" />
            </filter>
          </defs>

          {/* traced arc */}
          <path d={`M ${S[0]} ${S[1]} Q ${C[0]} ${C[1]} ${E[0]} ${E[1]}`}
            fill="none" stroke="url(#trailG)" strokeWidth="3.5" strokeLinecap="round"
            pathLength="1" strokeDasharray="1" strokeDashoffset={1 - bp} opacity={trailFade * 0.95} />

          {/* impact ring */}
          {ringOp > 0 && (
            <circle cx={E[0]} cy={E[1]} r="22" fill="none" stroke="var(--lime)" strokeWidth="2"
              style={{ transformOrigin: `${E[0]}px ${E[1]}px`, transform: `scale(${ringScale})`, opacity: ringOp }} />
          )}

          {/* motion-blur ghosts during flight */}
          {flightOn && [0.06, 0.12, 0.19].map((d, i) => {
            const gp = bp - d;
            if (gp <= 0) return null;
            const [gx, gy] = bez(gp);
            return <Ball key={i} cx={gx} cy={gy} r={r} opacity={0.14 * (1 - i * 0.28)} />;
          })}

          {/* glow + ball */}
          <circle cx={bx} cy={by} r={r * 0.95} fill="var(--lime)" opacity={0.45} filter="url(#ballGlow)" />
          <Ball cx={bx} cy={by} r={r} sx={sx} sy={sy} />
        </svg>

        {/* wordmark */}
        <div style={{ position: 'absolute', left: 0, right: 0, top: E[1] + 48, textAlign: 'center', fontFamily: "'Hanken Grotesk', system-ui, sans-serif" }}>
          <div style={{ overflow: 'hidden', padding: '0 2px' }}>
            <div style={{ fontSize: 31, fontWeight: 800, color: 'var(--text)', letterSpacing: -0.6, lineHeight: 1.05, transform: `translateY(${(1 - l1) * 110}%)`, opacity: l1 }}>Tennis Motion</div>
          </div>
          <div style={{ overflow: 'hidden', padding: '0 2px', marginTop: 1 }}>
            <div style={{ fontSize: 31, fontWeight: 800, color: 'var(--lime)', letterSpacing: -0.6, lineHeight: 1.05, transform: `translateY(${(1 - l2) * 110}%)`, opacity: l2 }}>Analyzer</div>
          </div>
          <div style={{ marginTop: 16, fontSize: 11, fontWeight: 700, letterSpacing: 3.4, textTransform: 'uppercase', color: 'var(--muted)', opacity: tg, transform: `translateY(${(1 - tg) * 6}px)` }}>Powered by IMU&nbsp;+&nbsp;AI</div>
        </div>
      </div>
    );
  }

  window.Splash = Splash;
})();
