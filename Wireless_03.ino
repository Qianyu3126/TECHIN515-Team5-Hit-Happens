#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// ⚠️ 每块板子改这里
#define NODE_ID 3       
#define SDA_PIN D4       
#define SCL_PIN D5

#define IMU_ADDR 0x6A

// -------- ESP-NOW --------
uint8_t centerMAC[] = {0x1C, 0xDB, 0xD4, 0x5C, 0x8D, 0x90};

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

// -------- Quaternion --------
float q0_ = 1, q1_ = 0, q2_ = 0, q3_ = 0;
float beta = 0.15f;

// -------- Gyro offsets --------
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// -------- Timing --------
unsigned long lastTime;

// -------- I2C helpers --------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 1);
  return Wire.read();
}

int16_t readReg16(uint8_t regL) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(regL);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (int16_t)(hi << 8 | lo);
}

// -------- Gyro calibration --------
void calibrateGyro() {
  Serial.println("Calibrating gyro... KEEP STILL");
  int samples = 300;
  for (int i = 0; i < samples; i++) {
    gx_offset += readReg16(0x22) * 0.00875f;
    gy_offset += readReg16(0x24) * 0.00875f;
    gz_offset += readReg16(0x26) * 0.00875f;
    delay(5);
  }
  gx_offset /= samples;
  gy_offset /= samples;
  gz_offset /= samples;
  Serial.println("Calibration done!");
}

// -------- Madgwick --------
void MadgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az, float dt) {
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0) return;
  ax /= norm; ay /= norm; az /= norm;

  float vx = 2*(q1_*q3_ - q0_*q2_);
  float vy = 2*(q0_*q1_ + q2_*q3_);
  float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  gx += beta*ex; gy += beta*ey; gz += beta*ez;

  float qDot0 = 0.5f*(-q1_*gx - q2_*gy - q3_*gz);
  float qDot1 = 0.5f*( q0_*gx + q2_*gz - q3_*gy);
  float qDot2 = 0.5f*( q0_*gy - q1_*gz + q3_*gx);
  float qDot3 = 0.5f*( q0_*gz + q1_*gy - q2_*gx);

  q0_ += qDot0*dt; q1_ += qDot1*dt;
  q2_ += qDot2*dt; q3_ += qDot3*dt;

  norm = sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
  q0_ /= norm; q1_ /= norm; q2_ /= norm; q3_ /= norm;
}

// -------- ESP-NOW 回调 --------
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// -------- Setup --------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  uint8_t who = readReg(0x0F);
  Serial.printf("WHO_AM_I: 0x%02X\n", who);
  if (!(who == 0x69 || who == 0x6A || who == 0x6C)) {
    Serial.println("IMU not detected!");
    while (1);
  }

  writeReg(0x10, 0x40);
  writeReg(0x11, 0x40);
  delay(200);
  calibrateGyro();

  WiFi.mode(WIFI_STA);
  delay(500);
  esp_now_init();
  esp_now_register_send_cb(onSent);

  memcpy(peerInfo.peer_addr, centerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  packet.node_id = NODE_ID;
  lastTime = micros();
  Serial.printf("节点 %d 启动完成\n", NODE_ID);
}

// -------- Loop --------
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.1) dt = 0.01;

  float ax = readReg16(0x28) * 0.000061f;
  float ay = readReg16(0x2A) * 0.000061f;
  float az = readReg16(0x2C) * 0.000061f;
  float gx = readReg16(0x22) * 0.00875f - gx_offset;
  float gy = readReg16(0x24) * 0.00875f - gy_offset;
  float gz = readReg16(0x26) * 0.00875f - gz_offset;

  // 翻转修正
  ax=-ax; ay=-ay; az=-az;
  gx=-gx; gy=-gy; gz=-gz;

  MadgwickUpdate(gx, gy, gz, ax, ay, az, dt);

  float roll  = atan2(2*(q0_*q1_ + q2_*q3_), 1 - 2*(q1_*q1_ + q2_*q2_)) * 57.3f;
  float pitch = asin(2*(q0_*q2_ - q3_*q1_)) * 57.3f;
  float yaw   = atan2(2*(q0_*q3_ + q1_*q2_), 1 - 2*(q2_*q2_ + q3_*q3_)) * 57.3f;

  // 填包
  packet.ax = ax; packet.ay = ay; packet.az = az;
  packet.gx = gx; packet.gy = gy; packet.gz = gz;
  packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
  packet.q0 = q0_; packet.q1 = q1_; packet.q2 = q2_; packet.q3 = q3_;
  packet.timestamp = millis();

  esp_now_send(centerMAC, (uint8_t*)&packet, sizeof(packet));

  delay(10); // 100Hz
}