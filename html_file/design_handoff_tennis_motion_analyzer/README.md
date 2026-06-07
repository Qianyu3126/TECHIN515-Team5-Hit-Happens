# Handoff: Tennis Motion Analyzer

## Overview
A single-screen mobile web app (iPhone-sized, 390 × 844) for a tennis motion-analysis
system that uses wearable IMU sensors. The athlete/coach selects a shot type, calibrates
the sensors, records a swing, and instantly sees an AI analysis of their form. The screen
opens with a ~2s motion-graphic **splash** that then reveals the main app.

The product flow is always: **select motion → calibrate → start → swing → stop → view result.**

## About the Design Files
The files in this bundle are **design references created in HTML/React (via in-browser Babel)** —
prototypes that demonstrate the intended look, motion, and behavior. They are **not** meant to
be shipped as-is. The task is to **recreate these designs in your target codebase** using its
established environment and patterns (React Native, SwiftUI, Flutter, a production React/Vite app,
etc.). If no environment exists yet, pick the most appropriate framework and implement there.

The prototype is a single React tree transpiled at runtime by Babel — fine for a prototype, but in
production you should use a normal build pipeline, real components, and proper state management.

> **Important environment note:** the prototype drives all of its animation with `setInterval`
> (the splash, the recording timer, the calibration bar) **because the preview sandbox throttles
> `requestAnimationFrame` and CSS keyframe animations.** That is a sandbox workaround, **not** a
> recommendation. In a real app, use `requestAnimationFrame` / CSS transitions / a motion library
> (Framer Motion, React Native Reanimated, Core Animation, etc.) for the splash and micro-interactions.

## Fidelity
**High-fidelity (hifi).** Final colors, typography, spacing, motion, and interaction states are all
specified below. Recreate the UI faithfully using your codebase's component library, then wire the
state machine and timings exactly as documented.

---

## Design Tokens

### Color
| Token | Value | Use |
|---|---|---|
| `--bg` | `#0E1311` | App background (near-black green) |
| `--bg2` | `#090C0A` | Deepest background |
| `--surface` | `#161D19` | Cards / console panel |
| `--surface2` | `#1B241F` | Inactive tabs, secondary buttons |
| `--surface3` | `#212C26` | Calibrate button, result card top |
| `--line` | `rgba(196,216,186,0.10)` | Hairline borders / dividers |
| `--line2` | `rgba(196,216,186,0.16)` | Slightly stronger border |
| `--text` | `#ECF2E8` | Primary text (warm off-white) |
| `--muted` | `#8C988D` | Secondary text |
| `--muted2` | `#5C6960` | Tertiary / disabled text |
| `--lime` | `#CFF23E` | **Primary accent** (tennis-ball lime) — active states, primary CTA, online, positive |
| `--lime-deep` | `#A6C82C` | Deeper lime |
| `--lime-dim` | `rgba(207,242,62,0.14)` | Lime tint fills (badges, pills) |
| `--amber` | `#F2B23E` | Calibrating / analyzing / "needs work" warning |
| `--amber-dim` | `rgba(242,178,62,0.14)` | Amber tint fill |
| `--red` | `#F26A4B` | Recording / Stop |
| `--red-dim` | `rgba(242,106,75,0.16)` | Red tint fill |
| `--ok` | `#79E39A` | (reserved positive green) |

Confidence color mapping: **≥85 → lime**, **72–84 → amber**, **<72 → red**.

### Typography
- **UI font:** `Hanken Grotesk` (weights 400/500/600/700/800).
- **Numeric / data font:** `Space Grotesk` (`.mono` class, `font-variant-numeric: tabular-nums`) — used
  for timers, percentages, the detected-motion name, and confidence figures.
- Both loaded from Google Fonts.

Representative sizes (px): section labels 10–11.5 (uppercase, letter-spacing 1.2–1.4); body/status 11.5–13.5;
button labels 13.5; tab labels 12.5; result detected-name 25; confidence 30; recording timer 56.

### Spacing / Radius / Shadow
- Outer screen padding `60px 16px 40px`; vertical gap between major blocks `18px`.
- Console panel radius `24`, padding `14`, inner gap `13`.
- Cards/hero radius `22`; buttons/tabs radius `14`; small pills/tags radius `8–10`; dots/rings `99`.
- Elevations are subtle: `inset 0 0 0 1px var(--line)` for grouping; primary buttons get a soft colored
  glow e.g. `0 8px 20px -10px rgba(207,242,62,.7)`; result card `0 18px 36px -22px rgba(0,0,0,.9)`.
- Tap feedback: `transform: scale(.965)` on `:active` (≈120ms). **Do not transition `background` between
  CSS variables** — in this engine that strands the element at its old color; switch background instantly.

---

## Screens / Views

There is **one screen** with two stacked zones inside an iPhone frame. Hit targets are ≥44px for on-court use.

### A. Splash (intro overlay)
- **Purpose:** branded motion-graphic intro on launch.
- **Layout:** full-bleed overlay (absolute inset 0) above the app, opaque dark radial-gradient background
  (`#0B0F0C` base, `radial-gradient(125% 95% at 50% 40%, #141C16, #0E1311 58%, #080B09)`).
- **Content:** a tennis ball (lime circle with two dark seam arcs) + wordmark "**Tennis Motion**" (white 800)
  / "**Analyzer**" (lime 800) + tagline "POWERED BY IMU + AI" (muted, letter-spacing 3.4, uppercase).
- **Motion timeline (total ≈2.0s, see `splash.jsx`):**
  - `0–720ms`: ball arcs from bottom-left `(-44,760)` through control `(44,150)` to center `(195,360)`
    along a quadratic bezier, eased `easeOutCubic`. A glowing lime trail (gradient stroke) traces the path
    via `stroke-dashoffset`. 3 faint motion-blur "ghost" balls trail during flight.
  - `660–900ms`: ball **settles** with a squash/stretch (`sin` curve) and grows from r15 → r24.
  - `700–1080ms`: an **impact ring** expands and fades at center.
  - `820–1300ms`: wordmark lines rise in from a clip mask (translateY 110%→0, staggered ~130ms).
  - `1160–1480ms`: tagline fades up.
  - `1560–1980ms`: whole splash slides up (−46px) and fades out → reveals app; unmounts at ~1980ms.
- **Skip:** tapping anywhere fires `onDone` immediately.
- **Production note:** implement with your platform's animation system; the math (bezier, easings, timings)
  is all in `splash.jsx` and is the source of truth.

### B. Main app (revealed under the splash)
Top to bottom:

**Header** (`<header>`)
- Left: 38×38 rounded-square (radius 12, `--surface2`) holding a small tennis-ball glyph (lime), then
  two-line title "**Tennis Motion**" (16.5/800) over "IMU Swing Analyzer" (11/600 muted).
- Right: connection pill — a dot + label. **LIVE** (lime, with a soft pulsing ring) when all 5 sensors are
  online; **SYNC** (amber, blinking dot) when any sensor is offline.

**Operation Console** (grouped, lower visual priority — one `--surface` panel, radius 24)
1. **Shot Type** — 3 equal tap-tabs in a row: **Serve / Forehand / Backhand**. Each = icon (scaled .78) +
   label (12.5/700). Exactly one active; active tab = lime fill, dark text, soft lime glow; inactive =
   `--surface2` with hairline border. Disabled while recording/analyzing.
2. **Controls** — a single compact row (height 52, radius 14), `flex` weights ~1.05 / 1.3 / 0.9:
   - **Calibrate** (`--surface3`, lime icon) — label shows live `NN%` while calibrating.
   - **Start** (lime, primary) — **disabled/greyed until calibration completes**. While recording this slot
     is replaced **in place** by **Stop** (red, with a pulsing `recRing` halo).
   - **Reset** (`--surface2`, muted) — clears the session back to Idle.
3. **System status strip** (divided by a top hairline):
   - Row: "Sensor Nodes" label + "`N/5 online`" count (lime if all online, else amber).
   - **5 sensor chips** in a row, downsized/low-priority: small node glyph + status dot (top-right) +
     tiny label. Nodes: **Waist, R. Arm, R. Fore, L. Arm, L. Fore** (full names: Waist, R. Upper Arm,
     R. Forearm, L. Upper Arm, L. Forearm). Online = lime glyph+dot; offline = muted.
   - **Status line:** a state tag pill (Idle / Calibrating / Ready / Recording / Analyzing / Done, color-coded)
     + a short guidance message.

**Analysis (hero / focus zone)** — the visual climax; distinct heading with a lime tick + "Analysis".
A single stage card (min-height 172, radius 22) that swaps by phase:
   - **idle / ready:** placeholder — faded ball glyph + "No swing recorded yet" / "Ready when you are" + hint.
   - **calibrating:** amber-tinted card — "CALIBRATING" tag, big `NN%`, "Hold still — aligning sensor
     orientation…", and a **glowing amber progress bar** filling to 100%.
   - **recording:** red-tinted card — "RECORDING {shot}" tag, **large live timer** `N.N` s (mono, 56px),
     "Press Stop after your follow-through", and a sweeping shimmer bar at the bottom.
   - **analyzing:** amber-tinted card — spinner + "Analyzing swing…" + sub-line.
   - **done:** the **Result Card** (see below).

**Result Card** (shown in the Analysis stage when done)
- Tone-driven: **positive** uses lime accent + "GOOD FORM" check badge; **warning** uses amber + "NEEDS WORK"
  triangle badge (tone = positive when confidence ≥85, else warning).
- Top row: tone badge (left) + shot type label (right, muted uppercase).
- Middle: "DETECTED" label + large detected-motion name (mono 25) on the left; large color-coded
  confidence `NN%` (mono 30) + "CONFIDENCE" on the right.
- Confidence **meter** bar (animates width to `conf%`, color = confidence mapping).
- Diagnostic feedback: 2–3 lines of plain-English coaching, with a left accent rule in the tone color.
- A soft blurred radial glow (tone color) in the top-right corner.

**Recent Results** (bottom, low priority)
- Last 3 sessions in a `--surface` list. Each row: tone dot + detected-motion name (700) over
  "`{shot} · {time}`" (muted) + color-coded `conf%` (mono). New results prepend; list capped at 3.

---

## Interactions & Behavior

### State machine (phases)
`idle → calibrating → ready → recording → analyzing → done` (+ `reset` returns to `idle`).

| Action | Guard | Effect |
|---|---|---|
| Tap a shot tab | not recording/analyzing | sets `motion` (serve/forehand/backhand) |
| **Calibrate** | not calibrating/recording/analyzing | `phase=calibrating`; progress 0→100 over **2400ms**; at ~1500ms the offline sensor (L. Forearm) comes online; at 2400ms `calibrated=true`, `phase=ready` |
| **Start** | `calibrated` && not recording | `phase=recording`; elapsed timer counts up (0.1s resolution) |
| **Stop** | recording | `phase=analyzing`; after **1600ms** pick an analysis result, `phase=done`, prepend to Recent |
| **Reset** | always | clears timers; `phase=idle`, `calibrated=false`, progress/elapsed/result reset (Recent persists) |
| Tap splash | splash visible | skip to app |

### Sensors
- 5 nodes. **L. Forearm starts offline** so the LIVE/SYNC indicator and "N/5 online" are meaningful;
  it connects ~1.5s into calibration, after which the header shows **LIVE** and "5/5 online".

### Analysis content (mock model output)
Each shot type has a small pool of plausible results `{ name, conf, tone, feedback }`; one is chosen at
random on Stop. Examples (see `ANALYSIS` in `app.jsx` for the full set):
- Serve → "Flat Serve" 93% positive; "Flat Serve" 76% warning; "Kick Serve" 68% warning.
- Forehand → "Topspin Forehand" 95% positive; "Forehand" 79% warning; "Flat Forehand" 71% warning.
- Backhand → "Two-Hand Backhand" 91% positive; "One-Hand Backhand" 77% warning; "Backhand" 69% warning.
- In production, replace this pool with the real classifier output (label + confidence + generated coaching text).

### Motion / micro-interactions
- Tap: `scale(.965)` ≈120ms. Live dot pulses (LIVE ring, recording/analyzing/calibrating blink).
- Confidence meter animates its width on appear. Recording shimmer + Stop halo loop while recording.
- Timers/bars are **setInterval-driven in the prototype** (sandbox constraint) — use native timers/animation
  in production; the durations above are the contract.

---

## State Management
State variables (currently `useState` in `app.jsx`):
- `motion`: `'serve' | 'forehand' | 'backhand'` (default `serve`)
- `phase`: `'idle' | 'calibrating' | 'ready' | 'recording' | 'analyzing' | 'done'`
- `calibrated`: boolean
- `calProgress`: 0–100 (calibration %)
- `elapsed`: seconds (recording timer)
- `result`: `{ shot, name, conf, tone, fb } | null`
- `recent`: array of last ≤3 `{ shot, name, conf, tone, time }`
- `offline`: map of sensorId → true (starts `{ lfa: true }`)
- Timers tracked in a ref and cleared on reset/stop/unmount.

Data needs in production: a sensor-connection service (online/offline per node), a calibration routine,
a swing-capture + IMU stream, and the classifier that returns `{ label, confidence, feedback }`.

---

## Assets
No external image/icon assets — all glyphs (tennis ball, shot icons, sensor node, target, play/stop/reset,
check/warn/bolt) are **inline SVG** defined in the `I` object in `app.jsx` and in `splash.jsx`. Recreate as
your icon components or reuse the SVG paths. Fonts: Hanken Grotesk + Space Grotesk (Google Fonts).

## Files
- `index.html` — page shell: design tokens (`:root`), `@keyframes`, font links, fit-to-viewport scaling,
  script load order (React 18.3.1 + Babel, then `ios-frame.jsx`, `splash.jsx`, `app.jsx`).
- `app.jsx` — the full app: icon set, data (`MOTIONS`, `SENSORS`, `ANALYSIS`, `SEED_RECENT`), components
  (`MotionTab`, `Ctrl`, `SensorChip`, `ResultCard`, `AnalysisStage`, `RecentRow`), the `App` screen, and
  the `Root` that overlays the splash on the app.
- `splash.jsx` — the `Splash` motion-graphic component (bezier arc, easings, full timeline).
- `ios-frame.jsx` — the iPhone bezel/status-bar wrapper (prototype chrome only; replace with the real device).

Open `index.html` to see everything running; the splash replays on each load
(`window.__replaySplash()` triggers it manually).
