# Tennis Motion Analyzer — UI Specification

## Overview

A single-page web app that opens in iPhone Safari.  
The UI receives structured inference results from an ESP32 via Supabase Realtime, then renders them as a designed result card.  
No App Store required. No serial/USB connection to the phone.

---

## Data Flow

```
ESP32 (inference done)
    │
    │  HTTP POST over WiFi
    ▼
Supabase (cloud database)
    │
    │  Realtime WebSocket subscription
    ▼
iPhone Safari (this UI)
    │
    │  JS receives: { predicted: "serve_elbow", confidence: 0.76 }
    ▼
Renders result card with typography, color, and feedback copy
```

The Serial Monitor (Arduino IDE black box) is for hardware debugging only — the phone cannot access it.

---

## Design System

| Token       | Value     | Usage                              |
|-------------|-----------|------------------------------------|
| `--bg`      | `#0d1117` | Page background                    |
| `--surface` | `#161b22` | Card background                    |
| `--border`  | `#21262d` | Dividers, chip borders             |
| `--primary` | `#c5e642` | Tennis yellow-green — selected, active states |
| `--good`    | `#4ade80` | Correct form results               |
| `--warn`    | `#fb923c` | Minor form errors                  |
| `--danger`  | `#ef4444` | Significant form errors            |
| `--text`    | `#f0f6fc` | Primary text                       |
| `--muted`   | `#8b949e` | Secondary / label text             |

**Font:** `-apple-system` (SF Pro on iPhone, system default elsewhere)  
**Max width:** 430px, centered, 16px side padding

---

## Layout (top to bottom, single scroll)

```
┌─────────────────────────────────┐
│  🎾 Tennis Motion Analyzer  ●  │  header + connection dot
├─────────────────────────────────┤
│  [ Serve ]  [ Forehand ]  [ BH ]│  motion selector — 3 tab pills
├─────────────────────────────────┤
│  [Calibrate]  [▶ Start  ]       │
│  [⏹ Stop   ]  [↺ Reset  ]       │  controls (Stop hidden until recording)
├─────────────────────────────────┤
│  Waist · R.UA · R.FA · L.UA · L.FA  │  node status chips
├─────────────────────────────────┤
│  STATUS: Recording   3.2s       │  state label + live timer
├─────────────────────────────────┤
│  ┌──────────────────────────┐   │
│  │  ⚠️  Elbow Tuck Detected │   │
│  │  Confidence: 76%         │   │  result card (appears after Stop)
│  │  ──────────────────────  │   │
│  │  Your elbow drops…       │   │
│  └──────────────────────────┘   │
├─────────────────────────────────┤
│  Recent Results                 │
│  ○  Serve · Elbow Tuck    76%  14:32  │
│  ○  Forehand · Correct    82%  14:28  │  last 3 sessions
│  ○  Backhand · No Rotate  65%  14:21  │
└─────────────────────────────────┘
```

---

## State Machine

```
idle
 ├─[Calibrate]─▶ calibrating ─[done]─▶ idle (ready)
 │                                          │
 │                                    [▶ Start]
 │                                          │
 │                                      recording
 │                                          │
 │                                     [⏹ Stop]
 │                                          │
 │                                      analyzing
 │                                          │
 │                                        done ─[↺ Reset]─▶ idle
 │
 └─[↺ Reset (any state)]─▶ idle
```

### Button visibility per state

| State        | Calibrate | Start    | Stop    | Reset |
|--------------|-----------|----------|---------|-------|
| idle         | ✓         | ✓ (gray if not calibrated) | — | ✓ |
| calibrating  | disabled  | disabled | —       | disabled |
| recording    | disabled  | hidden   | ✓       | disabled |
| analyzing    | disabled  | disabled | —       | disabled |
| done         | ✓         | ✓        | —       | ✓ |

---

## Result Card

Shown only after Stop → analyzing completes.

- **Motion label** — human-readable English (e.g. "Elbow Tuck Detected", "Correct Form")
- **Confidence badge** — percentage, color-coded:
  - ≥ 75% → `--good` green
  - 50–74% → `--warn` orange
  - < 50% → `--danger` red
- **Diagnostic copy** — 2–3 sentences of English feedback per label (hardcoded in UI)
- No score bars, no swing amplitude (keep card concise)

---

## Motion Labels & Feedback Copy

### Serve
| Key | Display | Feedback |
|-----|---------|----------|
| `serve_correct` | Correct Form | Great serve. Full arm extension and complete follow-through. Keep it up. |
| `serve_elbow` | Elbow Tuck Detected | Your elbow drops below shoulder level at contact. Focus on keeping the elbow high and fully extending through the ball. |
| `serve_incomplete_followthrough` | Incomplete Follow-Through | The racket arm doesn't complete its downward swing. Let the momentum carry through to finish across your body. |

### Forehand
| Key | Display | Feedback |
|-----|---------|----------|
| `forehand_correct` | Correct Form | Solid forehand. Full takeback, good contact point, and clean follow-through. |
| `forehand_incomplete_backswing` | Short Takeback | The racket doesn't travel far enough back before contact. Rotate the shoulder fully to load up power. |
| `forehand_incomplete_followthrough` | Incomplete Follow-Through | The follow-through stops short of the shoulder. Let the racket swing over and finish high on the opposite side. |

### Backhand
| Key | Display | Feedback |
|-----|---------|----------|
| `backhand_correct` | Correct Form | Clean backhand. Good body rotation and complete follow-through. |
| `backhand_incomplete_followthrough` | No Shoulder Finish | The racket stops before crossing the shoulder. Commit to the follow-through and let the racket pass over your lead shoulder. |
| `backhand_no_body_rotation` | Insufficient Body Rotation | The swing is arm-only — the hips and torso aren't driving the shot. Initiate from the waist and let the upper body follow. |

---

## Supabase Schema (for hardware team)

```sql
-- UI writes to this table (commands for ESP32)
create table commands (
  id         uuid primary key default gen_random_uuid(),
  model      text not null,   -- 'serve' | 'forehand' | 'backhand'
  type       text not null,   -- 'calibrate' | 'start' | 'stop'
  ts         timestamptz default now()
);

-- ESP32 writes to this table (inference results)
create table results (
  id         uuid primary key default gen_random_uuid(),
  model      text not null,
  predicted  text not null,   -- label key, e.g. 'serve_elbow'
  confidence float not null,  -- 0.0 – 1.0
  ts         timestamptz default now()
);
```

UI subscribes to `results` INSERT events via Supabase Realtime.  
Each ESP32 polls `commands` and only acts on rows where `model` matches its own model type.

---

## Handoff Notes for Hardware Team

1. Create a Supabase project, run the SQL above.
2. Share `SUPABASE_URL` and `SUPABASE_ANON_KEY` with UI developer.
3. In ESP32 firmware: after inference, POST to `results` table via HTTP.
4. ESP32 polls `commands` table (or uses Supabase Realtime) for start/stop/calibrate triggers.
5. UI stub code is in `index.html` inside the commented `initSupabase()` block — just fill in URL + key.

---

## Files

| File | Purpose |
|------|---------|
| `index.html` | Complete single-file web app (HTML + CSS + JS) |
| `SPEC.md` | This document |
