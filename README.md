# TECHIN515-Team5-Hit-Happens

## Overview

A wearable tennis stroke monitoring system using 4× XIAO ESP32S3 microcontrollers and LSM6DSOX IMU sensors. Three sensor nodes transmit IMU data wirelessly via ESP-NOW to one central hub node, which computes joint angles in real time.

---

## Hardware

| Component | Quantity | Role |
|-----------|----------|------|
| Seeed Studio XIAO ESP32S3 | 4 | 3× sensor nodes + 1× hub |
| LSM6DSOX IMU (I²C) | 4 | Motion sensing |
| LiPo Battery 3.7V | 3 | Wireless power |

### Sensor Placement So Far

| Node ID | Placement | Role |
|---------|-----------|------|
| Hub (center) | Connected to PC | Receives & computes |
| Node 1 | Upper arm | Upper arm orientation |
| Node 2 | Forearm | Forearm orientation |
| Node 3 | Waist / other arm | Additional context |

---

## How to Run the Code

### 1. Prerequisites

Install the following before starting:

- [Arduino IDE](https://www.arduino.cc/en/software) (v2.0 or later)
- **ESP32 board support** for Arduino IDE
  - Open `Preferences` → paste this URL into *Additional Boards Manager URLs*:
    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```
  - Go to `Tools` → `Board` → `Boards Manager`, search for **esp32**, and install the package by Espressif Systems
- **Required libraries** (install via `Sketch` → `Include Library` → `Manage Libraries`):
  - `Adafruit LSM6DSOX`
  - `Adafruit Unified Sensor`
  - `Wire` (built-in)
  - `esp_now` and `WiFi` (included with the ESP32 board package)

### 2. Hardware Setup

Wire each **LSM6DSOX IMU** to its **XIAO ESP32S3** over I²C:

| IMU Pin | XIAO ESP32S3 Pin |
| ------- | ---------------- |
| VIN     | 3V3              |
| GND     | GND              |
| SDA     | D4 (GPIO5)       |
| SCL     | D5 (GPIO6)       |

Do this for all 4 boards (4 sensor nodes + 1 hub). Connect LiPo batteries to the 4 sensor nodes; the hub stays connected to your PC via USB.

### 3. Find the Hub's MAC Address

The sensor nodes need to know the hub's MAC address to send data via ESP-NOW.

1. Connect the **hub** board to your computer via USB.
2. In Arduino IDE, select the correct board: `Tools` → `Board` → `XIAO_ESP32S3`.
3. Select the correct port under `Tools` → `Port`.
4. Upload a simple MAC-printing sketch, or use this snippet in the Serial Monitor:
   ```cpp
   #include <WiFi.h>
   void setup() {
     Serial.begin(115200);
     WiFi.mode(WIFI_STA);
     Serial.println(WiFi.macAddress());
   }
   void loop() {}
   ```
5. Open the Serial Monitor at **115200 baud** and record the printed MAC address (e.g., `AA:BB:CC:DD:EE:FF`).

### 4. Flash the Sensor Nodes

For **each of the 4 sensor nodes**:

1. Open `Wireless.ino` in Arduino IDE.
2. Update the `hubAddress[]` array at the top of the file with the MAC address you recorded in Step 4:
   ```cpp
   uint8_t hubAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
   ```
3. Set a unique `nodeID` for each sensor node (1, 2, or 3) — this lets the hub tell them apart.
4. Connect the node to your computer via USB, select the correct port, and click **Upload**.
5. Once uploaded, disconnect and power the node from its LiPo battery.
6. Repeat for the remaining sensor nodes.

### 5. Flash the Hub

1. Open `receiver.ino` in Arduino IDE.
2. Connect the hub board to your computer via USB, select the correct port.
3. Click **Upload**.
4. Leave the hub plugged into your PC — this is how data reaches your computer.

### 6. View the Data Stream

1. With the hub connected to your PC, open the Arduino IDE **Serial Monitor** (`Tools` → `Serial Monitor`).
2. Set the baud rate to **115200**.
3. Power on the 4 sensor nodes.
4. You should see incoming IMU readings tagged by node ID, along with computed joint angles, streaming in real time.

---

## Troubleshooting

- **No data on Serial Monitor** — Check that the baud rate is set to 115200, and that the hub's MAC address in `Wireless.ino` matches exactly what the hub printed.
- **Sensor nodes can't connect** — Make sure all 4 boards are on the same Wi-Fi channel (ESP-NOW defaults to channel 1). If you change it on one, change it on all.
- **IMU reads zeros or fails to initialize** — Double-check I²C wiring, and confirm the IMU's I²C address (default `0x6A` for LSM6DSOX).
- **Upload fails** — Hold the BOOT button on the XIAO ESP32S3 while plugging in USB to force bootloader mode.

