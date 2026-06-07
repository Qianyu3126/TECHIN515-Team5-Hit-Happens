# Receiver firmware — three stroke models

The receiver ESP32 (MAC `58:8C:81:AC:02:E8`) runs **one** Edge Impulse model at a
time. There are three near-identical sketches, one per stroke. Flash whichever
stroke you want to practice; the UI routes commands and results by model so you
can switch tabs freely.

| Folder | Sketch | Model library to install | Result tag (`model`) |
|--------|--------|--------------------------|----------------------|
| [forehand/](forehand) | `forehand.ino` | `models/ei-0605-techin515-group-arduino-*.zip` → `a0605_TECHIN515_Group_inferencing` | `forehand` |
| [backhand/](backhand) | `backhand.ino` | `models/ei-backhand-arduino-*.zip` → `backhand_inferencing` | `backhand` |
| [serve/](serve)       | `serve.ino`    | `models/ei-serve-arduino-*.zip` → `murphywei2000-project-1_inferencing` | `serve` |

## What differs between the three (and nothing else)

Only three clearly-marked `PER-MODEL` sections differ per folder:

1. **`#include`** — the model's Edge Impulse library header.
2. **`#define MY_MODEL`** — `"forehand"` / `"backhand"` / `"serve"`. This both tags
   every uploaded result row and filters the `commands` table (`model=eq.MY_MODEL`)
   so the board only reacts to its own tab in the app.
3. **Label normalization + feedback copy** in `postToSupabase()` / `printResult()` —
   each model emits different raw labels, mapped here to the app's keys + feedback.

The ESP-NOW receive, calibration, recording, and inference engine are byte-for-byte
identical. The engine is model-agnostic: it records 10 features/frame (5 nodes ×
roll+pitch) at 100 Hz and derives the window length from the model's own
`EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE` (forehand = 4 s, backhand/serve = 2 s).

## Per-model labels

| Model | Raw labels (from the .zip) | Normalized to |
|-------|----------------------------|---------------|
| forehand | `standard`, `incomplete backswing`, `incomplete followthough` | `forehand_correct`, `forehand_incomplete_backswing`, `forehand_incomplete_followthrough` |
| backhand | `backhand_correct`, `backhand_incomplete_followthrough`, `backhand_no_body_rotation` | same keys |
| serve | `correct`, `incomplete_backswing`, `incomplete_followthrough` | `serve_correct`, `serve_incomplete_backswing`, `serve_incomplete_followthrough` |

## How to flash

1. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** and add the
   matching model zip from `models/` (do this once per model; all three can be
   installed side by side).
2. Install **ArduinoJson** (v6+) from the Library Manager.
3. Open the folder for the stroke you want, set `WIFI_SSID` / `WIFI_PASSWORD`,
   select the board's port, and **Upload**.

## Important: running all three at once

Today the 5 sensor nodes **unicast** to this single receiver MAC, so only one
receiver board can ever get IMU data — meaning one stroke model runs at a time
(reflash to switch). To run three receiver boards **simultaneously**, the sensor
nodes must be changed to **broadcast** (`FF:FF:FF:FF:FF:FF`) on the shared WiFi
channel so every receiver hears the same stream. That node-side change is not
done yet.
