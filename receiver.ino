#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <math.h>

// -------- CONFIG --------
#define UPPER_NODE 3
#define FOREARM_NODE 4

typedef struct {
  uint8_t node_id;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  uint32_t timestamp;
} IMUPacket;

// -------- STORAGE --------
float q_upper[4];
float q_forearm[4];

bool got_upper = false;
bool got_forearm = false;

// -------- RECEIVE --------
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  IMUPacket p;
  memcpy(&p, data, sizeof(p));

  // ---- map nodes ----
  if (p.node_id == UPPER_NODE) {
    q_upper[0] = p.q0;
    q_upper[1] = p.q1;
    q_upper[2] = p.q2;
    q_upper[3] = p.q3;
    got_upper = true;
  }

  if (p.node_id == FOREARM_NODE) {
    q_forearm[0] = p.q0;
    q_forearm[1] = p.q1;
    q_forearm[2] = p.q2;
    q_forearm[3] = p.q3;
    got_forearm = true;
  }

  // ---- debug print ----
  Serial.printf("Node%d | Roll:%.1f Pitch:%.1f Yaw:%.1f\n",
                p.node_id, p.roll, p.pitch, p.yaw);

  // -------- compute elbow --------
  if (got_upper && got_forearm) {

    // inverse upper
    float w1 = q_upper[0];
    float x1 = -q_upper[1];
    float y1 = -q_upper[2];
    float z1 = -q_upper[3];

    // multiply q_upper^-1 * q_forearm
    float w = w1*q_forearm[0] - x1*q_forearm[1] - y1*q_forearm[2] - z1*q_forearm[3];
    float x = w1*q_forearm[1] + x1*q_forearm[0] + y1*q_forearm[3] - z1*q_forearm[2];
    float y = w1*q_forearm[2] - x1*q_forearm[3] + y1*q_forearm[0] + z1*q_forearm[1];
    float z = w1*q_forearm[3] + x1*q_forearm[2] - y1*q_forearm[1] + z1*q_forearm[0];

    // clamp
    if (w > 1) w = 1;
    if (w < -1) w = -1;

    float angle = 2 * acos(w) * 57.3;

    Serial.printf("🔥 Elbow Angle: %.1f°\n\n", angle);

    got_upper = false;
    got_forearm = false;
  }
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  delay(500);
  WiFi.begin();
  delay(500);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  Serial.println("中心节点启动，等待数据...");
}

// -------- LOOP --------
void loop() {}
