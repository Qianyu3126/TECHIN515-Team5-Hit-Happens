// =============================================================
// Receiver_ComplementaryFilter.ino (v3)
// Manual toggle: s 开始，s 停止
// =============================================================

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <math.h>

#define WAIST_NODE   1
#define R_UPPER_NODE 2
#define R_FORE_NODE  3
#define L_UPPER_NODE 4
#define L_FORE_NODE  5

typedef struct {
  uint8_t node_id;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  uint32_t timestamp;
} IMUPacket;

struct NodeFilter {
  float roll, pitch;
  float gx_bias, gy_bias, gz_bias;
  uint32_t lastTs;
};

NodeFilter f_waist = {};
NodeFilter f_rua   = {};
NodeFilter f_rfa   = {};
NodeFilter f_lua   = {};
NodeFilter f_lfa   = {};

struct RawIMU {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t ts;
};

RawIMU raw_waist = {0,0,0,0,0,0,0};
RawIMU raw_rua   = {0,0,0,0,0,0,0};
RawIMU raw_rfa   = {0,0,0,0,0,0,0};
RawIMU raw_lua   = {0,0,0,0,0,0,0};
RawIMU raw_lfa   = {0,0,0,0,0,0,0};

bool got_waist=false, got_rua=false, got_rfa=false;
bool got_lua=false,   got_lfa=false;

enum SysState { S_IDLE, S_CALIB_GYRO, S_READY };
SysState sysState = S_IDLE;

bool streamOn = false;
bool headerPrinted = false;

const int GYRO_CALIB_SAMPLES = 300;
int calibSamples = 0;
float sum_gx_waist=0, sum_gy_waist=0, sum_gz_waist=0;
float sum_gx_rua=0,   sum_gy_rua=0,   sum_gz_rua=0;
float sum_gx_rfa=0,   sum_gy_rfa=0,   sum_gz_rfa=0;
float sum_gx_lua=0,   sum_gy_lua=0,   sum_gz_lua=0;
float sum_gx_lfa=0,   sum_gy_lfa=0,   sum_gz_lfa=0;

const float beta = 0.1f;

void updateFilter(
  NodeFilter &f,
  float ax, float ay, float az,
  float gx, float gy, float gz,
  float dt)
{
  float accel_roll = atan2f(ay, az) * 57.29578f;
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;

  float roll_pred = f.roll + gx * dt;
  float pitch_pred = f.pitch + gy * dt;

  f.roll = beta * accel_roll + (1.0f - beta) * roll_pred;
  f.pitch = beta * accel_pitch + (1.0f - beta) * pitch_pred;
}

void clearGotFlags(){
  got_waist = got_rua = got_rfa = got_lua = got_lfa = false;
}

void onReceive(const esp_now_recv_info_t *info,
               const uint8_t *data, int len){
  if(len < (int)sizeof(IMUPacket)) return;

  IMUPacket p;
  memcpy(&p, data, sizeof(p));

  switch(p.node_id){
    case WAIST_NODE:
      raw_waist.ax=p.ax; raw_waist.ay=p.ay; raw_waist.az=p.az;
      raw_waist.gx=p.gx; raw_waist.gy=p.gy; raw_waist.gz=p.gz;
      raw_waist.ts = p.timestamp;
      got_waist = true; break;
    case R_UPPER_NODE:
      raw_rua.ax=p.ax; raw_rua.ay=p.ay; raw_rua.az=p.az;
      raw_rua.gx=p.gx; raw_rua.gy=p.gy; raw_rua.gz=p.gz;
      raw_rua.ts = p.timestamp;
      got_rua = true; break;
    case R_FORE_NODE:
      raw_rfa.ax=p.ax; raw_rfa.ay=p.ay; raw_rfa.az=p.az;
      raw_rfa.gx=p.gx; raw_rfa.gy=p.gy; raw_rfa.gz=p.gz;
      raw_rfa.ts = p.timestamp;
      got_rfa = true; break;
    case L_UPPER_NODE:
      raw_lua.ax=p.ax; raw_lua.ay=p.ay; raw_lua.az=p.az;
      raw_lua.gx=p.gx; raw_lua.gy=p.gy; raw_lua.gz=p.gz;
      raw_lua.ts = p.timestamp;
      got_lua = true; break;
    case L_FORE_NODE:
      raw_lfa.ax=p.ax; raw_lfa.ay=p.ay; raw_lfa.az=p.az;
      raw_lfa.gx=p.gx; raw_lfa.gy=p.gy; raw_lfa.gz=p.gz;
      raw_lfa.ts = p.timestamp;
      got_lfa = true; break;
    default: return;
  }

  if(!(got_waist && got_rua && got_rfa && got_lua && got_lfa))
    return;

  if(sysState == S_IDLE){
    clearGotFlags();
    return;
  }

  if(sysState == S_CALIB_GYRO){
    sum_gx_waist += raw_waist.gx; sum_gy_waist += raw_waist.gy; sum_gz_waist += raw_waist.gz;
    sum_gx_rua   += raw_rua.gx;   sum_gy_rua   += raw_rua.gy;   sum_gz_rua   += raw_rua.gz;
    sum_gx_rfa   += raw_rfa.gx;   sum_gy_rfa   += raw_rfa.gy;   sum_gz_rfa   += raw_rfa.gz;
    sum_gx_lua   += raw_lua.gx;   sum_gy_lua   += raw_lua.gy;   sum_gz_lua   += raw_lua.gz;
    sum_gx_lfa   += raw_lfa.gx;   sum_gy_lfa   += raw_lfa.gy;   sum_gz_lfa   += raw_lfa.gz;
    calibSamples++;

    if(calibSamples % 50 == 0){
      Serial.printf("# Calib %d/%d\n", calibSamples, GYRO_CALIB_SAMPLES);
    }

    if(calibSamples >= GYRO_CALIB_SAMPLES){
      f_waist.gx_bias = sum_gx_waist / calibSamples;
      f_waist.gy_bias = sum_gy_waist / calibSamples;
      f_waist.gz_bias = sum_gz_waist / calibSamples;

      f_rua.gx_bias = sum_gx_rua / calibSamples;
      f_rua.gy_bias = sum_gy_rua / calibSamples;
      f_rua.gz_bias = sum_gz_rua / calibSamples;

      f_rfa.gx_bias = sum_gx_rfa / calibSamples;
      f_rfa.gy_bias = sum_gy_rfa / calibSamples;
      f_rfa.gz_bias = sum_gz_rfa / calibSamples;

      f_lua.gx_bias = sum_gx_lua / calibSamples;
      f_lua.gy_bias = sum_gy_lua / calibSamples;
      f_lua.gz_bias = sum_gz_lua / calibSamples;

      f_lfa.gx_bias = sum_gx_lfa / calibSamples;
      f_lfa.gy_bias = sum_gy_lfa / calibSamples;
      f_lfa.gz_bias = sum_gz_lfa / calibSamples;

      sysState = S_READY;
      Serial.println("# Gyro calib done.");
      headerPrinted = false;
    }

    clearGotFlags();
    return;
  }

  // ---- Calculate dt ----
  float dt_waist = (raw_waist.ts > f_waist.lastTs && f_waist.lastTs > 0)
                   ? (raw_waist.ts - f_waist.lastTs) * 0.001f : 0.01f;
  float dt_rua   = (raw_rua.ts > f_rua.lastTs && f_rua.lastTs > 0)
                   ? (raw_rua.ts - f_rua.lastTs) * 0.001f : 0.01f;
  float dt_rfa   = (raw_rfa.ts > f_rfa.lastTs && f_rfa.lastTs > 0)
                   ? (raw_rfa.ts - f_rfa.lastTs) * 0.001f : 0.01f;
  float dt_lua   = (raw_lua.ts > f_lua.lastTs && f_lua.lastTs > 0)
                   ? (raw_lua.ts - f_lua.lastTs) * 0.001f : 0.01f;
  float dt_lfa   = (raw_lfa.ts > f_lfa.lastTs && f_lfa.lastTs > 0)
                   ? (raw_lfa.ts - f_lfa.lastTs) * 0.001f : 0.01f;

  // ---- Update filters ----
  updateFilter(f_waist, raw_waist.ax, raw_waist.ay, raw_waist.az,
               raw_waist.gx, raw_waist.gy, raw_waist.gz, dt_waist);
  updateFilter(f_rua, raw_rua.ax, raw_rua.ay, raw_rua.az,
               raw_rua.gx, raw_rua.gy, raw_rua.gz, dt_rua);
  updateFilter(f_rfa, raw_rfa.ax, raw_rfa.ay, raw_rfa.az,
               raw_rfa.gx, raw_rfa.gy, raw_rfa.gz, dt_rfa);
  updateFilter(f_lua, raw_lua.ax, raw_lua.ay, raw_lua.az,
               raw_lua.gx, raw_lua.gy, raw_lua.gz, dt_lua);
  updateFilter(f_lfa, raw_lfa.ax, raw_lfa.ay, raw_lfa.az,
               raw_lfa.gx, raw_lfa.gy, raw_lfa.gz, dt_lfa);

  f_waist.lastTs = raw_waist.ts;
  f_rua.lastTs = raw_rua.ts;
  f_rfa.lastTs = raw_rfa.ts;
  f_lua.lastTs = raw_lua.ts;
  f_lfa.lastTs = raw_lfa.ts;

  // ---- Stream output ----
  if(streamOn){
    if(!headerPrinted){
      Serial.println("roll_waist,pitch_waist,roll_rua,pitch_rua,roll_rfa,pitch_rfa,roll_lua,pitch_lua,roll_lfa,pitch_lfa");
      headerPrinted = true;
    }

    Serial.printf(
      "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      f_waist.roll, f_waist.pitch,
      f_rua.roll, f_rua.pitch,
      f_rfa.roll, f_rfa.pitch,
      f_lua.roll, f_lua.pitch,
      f_lfa.roll, f_lfa.pitch);
  }

  clearGotFlags();
}

void setup(){
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.begin();
  delay(300);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("# MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  if(esp_now_init() != ESP_OK){
    Serial.println("# ESP-NOW init failed");
    while(1) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.println("# Complementary Filter (beta=0.1, manual toggle)");
  Serial.println("# Commands: c=calib, s=toggle stream, r=reset");
}

void loop(){
  if(Serial.available()){
    char cmd = Serial.read();

    if(cmd == 'c' || cmd == 'C'){
      if(sysState != S_CALIB_GYRO){
        calibSamples = 0;
        sum_gx_waist=0; sum_gy_waist=0; sum_gz_waist=0;
        sum_gx_rua=0;   sum_gy_rua=0;   sum_gz_rua=0;
        sum_gx_rfa=0;   sum_gy_rfa=0;   sum_gz_rfa=0;
        sum_gx_lua=0;   sum_gy_lua=0;   sum_gz_lua=0;
        sum_gx_lfa=0;   sum_gy_lfa=0;   sum_gz_lfa=0;
        sysState = S_CALIB_GYRO;
        Serial.println("# Calib started. Keep still...");
      }
    }
    else if(cmd == 's' || cmd == 'S'){
      if(sysState == S_READY){
        streamOn = !streamOn;
        if(streamOn){
          Serial.println("# Stream ON");
          headerPrinted = false;
        } else {
          Serial.println("# Stream OFF");
        }
      } else {
        Serial.println("# Not ready. Press 'c' first.");
      }
    }
    else if(cmd == 'r' || cmd == 'R'){
      sysState = S_IDLE;
      streamOn = false;
      headerPrinted = false;
      Serial.println("# Reset. Press 'c' to recalibrate.");
    }
  }
}