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
