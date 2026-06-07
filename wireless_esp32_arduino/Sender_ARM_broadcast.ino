// =============================================================
// Sender_ARM_broadcast.ino
// Arm-node sender for 5-node tennis IMU system (nodes 2/3/4/5).
//
// BROADCAST version: sends to FF:FF:FF:FF:FF:FF so ALL receiver
// ESP32s on the same WiFi channel receive the same IMU stream at
// once (forehand + backhand + serve boards simultaneously). No
// need to know each receiver's MAC.
//
// Edit NODE_ID before flashing:
//   NODE_ID = 2 -> Right Upper Arm
//   NODE_ID = 3 -> Right Forearm
//   NODE_ID = 4 -> Left  Upper Arm
//   NODE_ID = 5 -> Left  Forearm
//
// Channel note: receivers connect to AP "robolab_5" and lock to its
// channel; this node defaults to channel 1. They must match. If
// receivers show this node offline, the AP is not on channel 1 —
// uncomment the esp_wifi_set_channel() line below and set it to the
// AP's channel.
// =============================================================

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
// #include "esp_wifi.h"   // only needed if you pin the channel below

#define NODE_ID  2     // ← EDIT THIS: 2, 3, 4, or 5
#define SDA_PIN  D4
#define SCL_PIN  D5
#define IMU_ADDR 0x6A
#define CALIB_SAMPLES 300

// BROADCAST address — reaches every receiver on the channel at once.
uint8_t receiverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct {
  uint8_t node_id;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  uint32_t timestamp;
} IMUPacket;

IMUPacket packet;
esp_now_peer_info_t peerInfo;

float gx_offset=0, gy_offset=0, gz_offset=0;
float ax_offset=0, ay_offset=0, az_offset=0;

void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg){
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 1);
  return Wire.read();
}

int16_t readReg16(uint8_t regL){
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(regL);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (int16_t)(hi<<8 | lo);
}

void calibrateBias(){
  Serial.println("Bias calibration... KEEP STILL");

  float sum_gx=0, sum_gy=0, sum_gz=0;

  for(int i=0; i<CALIB_SAMPLES; i++){
    sum_gx += readReg16(0x22) * 0.00875f;
    sum_gy += readReg16(0x24) * 0.00875f;
    sum_gz += readReg16(0x26) * 0.00875f;
    if(i % 50 == 0) Serial.printf("Bias calib %d/%d\n", i, CALIB_SAMPLES);
    delay(5);
  }

  gx_offset = sum_gx / CALIB_SAMPLES;
  gy_offset = sum_gy / CALIB_SAMPLES;
  gz_offset = sum_gz / CALIB_SAMPLES;

  ax_offset = 0.0f;
  ay_offset = 0.0f;
  az_offset = 0.0f;

  Serial.println("Bias done.");
  Serial.printf("  gyro bias: %.4f %.4f %.4f dps\n",
                gx_offset, gy_offset, gz_offset);
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status){}

void setup(){
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  uint8_t who = readReg(0x0F);
  Serial.printf("WHO_AM_I: 0x%02X\n", who);
  if(!(who==0x69 || who==0x6A || who==0x6C)){
    Serial.println("IMU not detected!");
    while(1) delay(1000);
  }

  writeReg(0x10, 0x40);   // accel ODR 104Hz
  writeReg(0x11, 0x40);   // gyro  ODR 104Hz
  delay(200);

  calibrateBias();

  WiFi.mode(WIFI_STA);
  delay(500);

  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed");
    while(1) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // pin to AP channel if needed

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;        // 0 = use current channel
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  packet.node_id = NODE_ID;

  const char* name = "?";
  switch(NODE_ID){
    case 2: name = "R_UPPER"; break;
    case 3: name = "R_FORE";  break;
    case 4: name = "L_UPPER"; break;
    case 5: name = "L_FORE";  break;
  }
  Serial.printf("Node %d (%s) ready. [BROADCAST]\n", NODE_ID, name);
}

void loop(){
  float ax = readReg16(0x28) * 0.000061f;
  float ay = readReg16(0x2A) * 0.000061f;
  float az = readReg16(0x2C) * 0.000061f;
  float gx = readReg16(0x22) * 0.00875f;
  float gy = readReg16(0x24) * 0.00875f;
  float gz = readReg16(0x26) * 0.00875f;

  ax -= ax_offset; ay -= ay_offset; az -= az_offset;
  gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

  packet.ax = ax; packet.ay = ay; packet.az = az;
  packet.gx = gx; packet.gy = gy; packet.gz = gz;
  packet.roll = 0; packet.pitch = 0; packet.yaw = 0;
  packet.q0 = 1; packet.q1 = 0; packet.q2 = 0; packet.q3 = 0;
  packet.timestamp = millis();

  esp_now_send(receiverAddress, (uint8_t*)&packet, sizeof(packet));

  delay(10);  // ~100 Hz
}
