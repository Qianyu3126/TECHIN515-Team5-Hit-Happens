#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// ---------- NODE SETTINGS ----------
#define NODE_ID  1
#define SDA_PIN  D4
#define SCL_PIN  D5
#define IMU_ADDR 0x6A
#define CALIB_SAMPLES 300

uint8_t receiverAddress[] = {0x58,0x8C,0x81,0xAC,0x02,0xE8};

// ---------- Packet ----------
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

// ---------- Madgwick ----------
float q0_=1, q1_=0, q2_=0, q3_=0;
float beta = 0.15f;

// ---------- Bias ----------
float gx_offset=0, gy_offset=0, gz_offset=0;
float ax_offset=0, ay_offset=0, az_offset=0;

// ---------- Timing ----------
unsigned long lastTime;

// ---------- I2C helpers ----------
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

// ---------- Bias calibration ----------
void calibrateBias(){
  Serial.println("Calibrating... KEEP STILL");

  float sum_gx=0,sum_gy=0,sum_gz=0;
  float sum_ax=0,sum_ay=0,sum_az=0;

  for(int i=0; i<CALIB_SAMPLES; i++){
    sum_ax += readReg16(0x28) * 0.000061f;
    sum_ay += readReg16(0x2A) * 0.000061f;
    sum_az += readReg16(0x2C) * 0.000061f;
    sum_gx += readReg16(0x22) * 0.00875f;
    sum_gy += readReg16(0x24) * 0.00875f;
    sum_gz += readReg16(0x26) * 0.00875f;

    if(i%50==0) Serial.printf("Calibrating %d/%d\n", i, CALIB_SAMPLES);
    delay(5);
  }

  gx_offset = sum_gx / CALIB_SAMPLES;
  gy_offset = sum_gy / CALIB_SAMPLES;
  gz_offset = sum_gz / CALIB_SAMPLES;
  ax_offset = sum_ax / CALIB_SAMPLES;
  ay_offset = sum_ay / CALIB_SAMPLES;
  az_offset = sum_az / CALIB_SAMPLES - 1.0f; // 减去重力

  Serial.println("Bias calibration done.");
  Serial.printf("  gyro  bias: %.4f %.4f %.4f dps\n",
                gx_offset, gy_offset, gz_offset);
  Serial.printf("  accel bias: %.4f %.4f %.4f g\n",
                ax_offset, ay_offset, az_offset);
}

// ---------- Madgwick update ----------
void MadgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az, float dt){
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if(norm < 1e-10f) goto integrate;
  ax /= norm; ay /= norm; az /= norm;

  {
    float vx = 2*(q1_*q3_ - q0_*q2_);
    float vy = 2*(q0_*q1_ + q2_*q3_);
    float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    gx += beta*ex; gy += beta*ey; gz += beta*ez;
  }

  integrate:
  float qDot0 = 0.5f*(-q1_*gx - q2_*gy - q3_*gz);
  float qDot1 = 0.5f*( q0_*gx + q2_*gz - q3_*gy);
  float qDot2 = 0.5f*( q0_*gy - q1_*gz + q3_*gx);
  float qDot3 = 0.5f*( q0_*gz + q1_*gy - q2_*gx);

  q0_ += qDot0*dt; q1_ += qDot1*dt;
  q2_ += qDot2*dt; q3_ += qDot3*dt;

  norm = sqrtf(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
  q0_ /= norm; q1_ /= norm; q2_ /= norm; q3_ /= norm;
}

// ---------- Send Callback ----------
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status){}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  uint8_t who = readReg(0x0F);
  Serial.printf("WHO_AM_I: 0x%02X\n", who);
  if(!(who==0x69 || who==0x6A || who==0x6C)){
    Serial.println("IMU not detected!");
    while(1);
  }

  // ODR 104Hz
  writeReg(0x10, 0x40);  // accel
  writeReg(0x11, 0x40);  // gyro
  delay(200);

  calibrateBias();

  WiFi.mode(WIFI_STA);
  delay(500);
  esp_now_init();
  esp_now_register_send_cb(onSent);

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  packet.node_id = NODE_ID;
  lastTime = micros();
  Serial.printf("Node %d ready\n", NODE_ID);
}

// ---------- Loop ----------
void loop(){
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if(dt<=0 || dt>0.1f) dt = 0.01f;

  // 读原始数据
  float ax = readReg16(0x28) * 0.000061f;
  float ay = readReg16(0x2A) * 0.000061f;
  float az = readReg16(0x2C) * 0.000061f;
  float gx = readReg16(0x22) * 0.00875f;
  float gy = readReg16(0x24) * 0.00875f;
  float gz = readReg16(0x26) * 0.00875f;

  // 去偏置
  ax -= ax_offset; ay -= ay_offset; az -= az_offset;
  gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

  MadgwickUpdate(gx, gy, gz, ax, ay, az, dt);

  float roll  = atan2f(2*(q0_*q1_+q2_*q3_), 1-2*(q1_*q1_+q2_*q2_)) * 57.3f;
  float pitch = asinf (2*(q0_*q2_-q3_*q1_)) * 57.3f;
  float yaw   = atan2f(2*(q0_*q3_+q1_*q2_), 1-2*(q2_*q2_+q3_*q3_)) * 57.3f;

  packet.ax=ax; packet.ay=ay; packet.az=az;
  packet.gx=gx; packet.gy=gy; packet.gz=gz;
  packet.roll=roll; packet.pitch=pitch; packet.yaw=yaw;
  packet.q0=q0_; packet.q1=q1_; packet.q2=q2_; packet.q3=q3_;
  packet.timestamp = millis();

  esp_now_send(receiverAddress, (uint8_t*)&packet, sizeof(packet));

  delay(10);
}