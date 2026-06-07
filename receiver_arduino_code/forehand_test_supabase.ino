/*
 * forehand_test.ino  — Supabase integration version
 * Added on top of the original:
 *   - WiFi password connection
 *   - postToSupabase() function
 *   - printResult() auto-POSTs after inference
 *
 * Library to install (Arduino Library Manager):
 *   ArduinoJson  by Benoit Blanchon  (v6+)
 *
 * Edit two lines to enable upload:
 *   WIFI_SSID / WIFI_PASSWORD  ->  your network
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>     // added (needed for HTTPS)
#include <esp_now.h>
#include "esp_wifi.h"
#include <math.h>
#include <HTTPClient.h>          // added
#include <ArduinoJson.h>         // added (library required)
#include <a0605_TECHIN515_Group_inferencing.h>

// ── WiFi credentials (your network) ───────────
#define WIFI_SSID     "robolab_5"
#define WIFI_PASSWORD "robot1234"

// ── Supabase config ───────────────────────────
#define SUPABASE_URL   "https://lljsklwsqtwkzbngkjtf.supabase.co"
#define SUPABASE_ANON  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImxsanNrbHdzcXR3a3pibmdranRmIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODA3NzE4MTUsImV4cCI6MjA5NjM0NzgxNX0.3_8A0oLARbU7zqa3hmPiTCyZfkfma2cCF7UqoH3k5S0"
#define SUPABASE_TABLE "results"

// ── Node IDs ──────────────────────────────────
#define WAIST_NODE   1
#define R_UPPER_NODE 2
#define R_FORE_NODE  3
#define L_UPPER_NODE 4
#define L_FORE_NODE  5

// ── ESP-NOW packet struct (must match node firmware) ────
typedef struct {
  uint8_t node_id;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  uint32_t timestamp;
} IMUPacket;

// ── Complementary filter ──────────────────────
const float BETA = 0.1f;

struct NodeFilter {
  float roll, pitch;
  float gx_bias, gy_bias, gz_bias;
  uint32_t lastTs;
};

NodeFilter f_waist={}, f_rua={}, f_rfa={}, f_lua={}, f_lfa={};
bool got_waist=false, got_rua=false, got_rfa=false, got_lua=false, got_lfa=false;

// ── Node connection monitor ───────────────────
uint32_t nodeLastSeen[6] = {0};
const char* nodeNames[6] = { "", "Waist", "R.UpperArm", "R.Forearm", "L.UpperArm", "L.Forearm" };
const uint32_t NODE_TIMEOUT_MS = 3000;
uint32_t lastNodeCheck = 0;

// ── System state ──────────────────────────────
enum SysState { S_IDLE, S_CALIB_GYRO, S_READY };
SysState sysState = S_IDLE;

enum Mode { MODE_IDLE, MODE_RECORDING };
Mode currentMode = MODE_IDLE;

// ── Calibration accumulators ──────────────────
const int GYRO_CALIB_SAMPLES = 300;
int calibSamples = 0;
float sum_gx_waist=0, sum_gy_waist=0, sum_gz_waist=0;
float sum_gx_rua=0,   sum_gy_rua=0,   sum_gz_rua=0;
float sum_gx_rfa=0,   sum_gy_rfa=0,   sum_gz_rfa=0;
float sum_gx_lua=0,   sum_gy_lua=0,   sum_gz_lua=0;
float sum_gx_lfa=0,   sum_gy_lfa=0,   sum_gz_lfa=0;

// ── Timed frame recording (100Hz, doesn't wait for all 5 nodes) ────
uint32_t lastFrameTime = 0;
const uint32_t FRAME_INTERVAL_MS = 10;
bool allNodesSeen = false;

// ── Inference buffer ──────────────────────────
#define N_AXES           10
// 1200 frames = 12 s @100Hz, plenty for a forehand swing (~6 s).
// The old 3000-frame static array used ~117 KB of heap, leaving only ~38 KB
// at upload time — not enough for a TLS (HTTPS) connection (~40-50 KB) -> connection refused.
// 1200 frames frees ~70 KB, raising pre-upload free heap to ~108 KB, enough for TLS.
#define MAX_FRAMES       1200
#define MOTION_THRESHOLD 20.0f

float ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
float rec_buffer[MAX_FRAMES][N_AXES];
int   rec_len = 0;
volatile bool pendingInference = false;   // flag to run inference (deferred) after recording stops

// forward declaration (setup calls pollCommands)
void pollCommands(bool applyBaseline);

// ─────────────────────────────────────────────
// POST inference result to Supabase
// ─────────────────────────────────────────────
void postToSupabase(const char* label, float confidence) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("# [Supabase] WiFi not connected, skipping upload");
    return;
  }

  // Label normalization: model may output without prefix or with spelling
  // variants; map them all to the standard labels
  String labelStr(label);
  String normalized;
  if (labelStr == "forehand_correct"   || labelStr == "correct"   || labelStr == "standard") {
    normalized = "forehand_correct";
  } else if (labelStr == "forehand_incomplete_backswing"  || labelStr == "incomplete backswing"  || labelStr == "incomplete_backswing") {
    normalized = "forehand_incomplete_backswing";
  } else if (labelStr == "forehand_incomplete_followthrough" || labelStr == "incomplete followthough" || labelStr == "incomplete_followthough" || labelStr == "incomplete followthrough" || labelStr == "incomplete_followthrough") {
    normalized = "forehand_incomplete_followthrough";
  } else {
    normalized = labelStr;  // unknown label passed through as-is
  }
  const char* normLabel = normalized.c_str();
  Serial.printf("# [Supabase] label normalized: %s -> %s\n", label, normLabel);

  // build feedback string
  const char* feedback = "";
  if (normalized == "forehand_correct") {
    feedback = "Solid forehand. Full takeback, good contact point, and clean follow-through.";
  } else if (normalized == "forehand_incomplete_backswing") {
    feedback = "Short takeback: the racket doesn't travel far enough back before contact. Rotate the shoulder fully to load up power.";
  } else if (normalized == "forehand_incomplete_followthrough") {
    feedback = "Incomplete follow-through: the swing stops short of the shoulder. Let the racket finish high on the opposite side.";
  } else {
    feedback = "Swing recorded. Keep practicing!";
  }

  Serial.printf("# [Supabase] free heap before upload: %u bytes\n", ESP.getFreeHeap());

  StaticJsonDocument<256> doc;
  doc["model"]      = "forehand";
  doc["predicted"]  = normLabel;
  doc["confidence"] = confidence;
  doc["feedback"]   = feedback;

  String body;
  serializeJson(doc, body);
  Serial.printf("# [Supabase] uploading: %s\n", body.c_str());

  // retry up to 3 times (new connection each time, gives memory a chance to recover)
  for (int attempt = 1; attempt <= 3; attempt++) {
    WiFiClientSecure client;
    client.setInsecure();            // ESP32 skips SSL certificate verification

    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_TABLE;
    http.begin(client, url);
    http.setTimeout(8000);           // POST timeout widened to 8 s
    http.addHeader("Content-Type",  "application/json");
    http.addHeader("apikey",        SUPABASE_ANON);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON);
    http.addHeader("Prefer",        "return=minimal");

    int code = http.POST(body);
    if (code > 0) {
      Serial.printf("# [Supabase] response %d OK\n", code);
      http.end();
      return;
    }
    Serial.printf("# [Supabase] attempt %d failed: %s (retrying...)\n",
                  attempt, HTTPClient::errorToString(code).c_str());
    http.end();
    delay(500);                      // wait half a second for TLS memory to free
  }
  Serial.println("# [Supabase] all three attempts failed, giving up this upload");
}

// ─────────────────────────────────────────────
void updateFilter(NodeFilter &f,
                  float ax, float ay, float az,
                  float gx, float gy, float gz, float dt) {
  float accel_roll  = atan2f(ay, az) * 57.29578f;
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
  f.roll  = BETA * accel_roll  + (1.0f - BETA) * (f.roll  + gx * dt);
  f.pitch = BETA * accel_pitch + (1.0f - BETA) * (f.pitch + gy * dt);
}

void clearGotFlags() {
  got_waist = got_rua = got_rfa = got_lua = got_lfa = false;
}

// ─────────────────────────────────────────────
// Result interpretation (postToSupabase() appended at the end)
// ─────────────────────────────────────────────
void printResult(const char* label, float confidence) {
  Serial.println("# ==================");
  Serial.printf ("# Result: %s (confidence %.1f%%)\n", label, confidence * 100);
  Serial.println("# ------------------");

  String ls(label);
  if (ls == "forehand_correct" || ls == "correct" || ls == "standard") {
    Serial.println("# [OK] Correct form — solid forehand, keep it up!");
  } else if (ls == "forehand_incomplete_backswing" || ls == "incomplete backswing" || ls == "incomplete_backswing") {
    Serial.println("# [X] Issue: short takeback");
    Serial.println("# -> Rotate the shoulder fully and take the racket farther back to load up power");
  } else if (ls == "forehand_incomplete_followthrough" || ls == "incomplete followthough" || ls == "incomplete_followthough" || ls == "incomplete followthrough" || ls == "incomplete_followthrough") {
    Serial.println("# [X] Issue: incomplete follow-through");
    Serial.println("# -> Let the racket swing over and finish high on the opposite shoulder");
  } else {
    Serial.printf("# Label: %s\n", label);
  }

  Serial.println("# ==================");
  Serial.println("# Press 's' to start the next recording");

  // auto-upload to Supabase after inference
  postToSupabase(label, confidence);
}

// ─────────────────────────────────────────────
// Inference main function (unchanged)
// ─────────────────────────────────────────────
void runInference() {
  int window_size = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / N_AXES;

  Serial.printf("# Analyzing %d frames...\n", rec_len);
  Serial.printf("# [memory] free heap: %u bytes\n", ESP.getFreeHeap());

  if (rec_len < window_size) {
    Serial.printf("# Recording too short! Need at least %d frames, currently %d\n", window_size, rec_len);
    return;
  }

  float max_val = -9999, min_val = 9999;
  for (int i = 0; i < rec_len; i++)
    for (int j = 0; j < N_AXES; j++) {
      if (rec_buffer[i][j] > max_val) max_val = rec_buffer[i][j];
      if (rec_buffer[i][j] < min_val) min_val = rec_buffer[i][j];
    }

  float motion_range = max_val - min_val;
  Serial.printf("# Motion range: %.1f deg\n", motion_range);

  if (motion_range < MOTION_THRESHOLD) {
    Serial.println("# No forehand motion detected (range too small), please retry");
    return;
  }

  int step = window_size / 2;
  int window_count = 0;
  float score_sum[EI_CLASSIFIER_LABEL_COUNT] = {0};

  for (int start = 0; start + window_size <= rec_len; start += step) {
    for (int i = 0; i < window_size; i++)
      for (int j = 0; j < N_AXES; j++)
        ei_buffer[i * N_AXES + j] = rec_buffer[start + i][j];

    signal_t signal;
    numpy::signal_from_buffer(ei_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result;
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) continue;

    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
      score_sum[i] += result.classification[i].value;
    window_count++;

    Serial.printf("# Window %d: ", window_count);
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
      Serial.printf("%s=%.0f%% ", result.classification[i].label,
                    result.classification[i].value * 100);
    Serial.println();
  }

  if (window_count == 0) { Serial.println("# Inference failed, please retry"); return; }

  float best_val = 0;
  int   best_idx = 0;

  Serial.printf("# %d windows total, combined scores:\n", window_count);
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float avg = score_sum[i] / window_count;
    Serial.printf("#   %-42s %.1f%%\n", ei_classifier_inferencing_categories[i], avg * 100);
    if (avg > best_val) { best_val = avg; best_idx = i; }
  }

  printResult(ei_classifier_inferencing_categories[best_idx], best_val);
}

// ─────────────────────────────────────────────
// ESP-NOW receive callback (unchanged)
// ─────────────────────────────────────────────
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(IMUPacket)) return;

  IMUPacket p;
  memcpy(&p, data, sizeof(p));

  if (p.node_id < 1 || p.node_id > 5) return;
  nodeLastSeen[p.node_id] = millis();

  if (sysState == S_IDLE) return;

  if (sysState == S_CALIB_GYRO) {
    switch (p.node_id) {
      case WAIST_NODE:   sum_gx_waist+=p.gx; sum_gy_waist+=p.gy; sum_gz_waist+=p.gz; got_waist=true; break;
      case R_UPPER_NODE: sum_gx_rua  +=p.gx; sum_gy_rua  +=p.gy; sum_gz_rua  +=p.gz; got_rua  =true; break;
      case R_FORE_NODE:  sum_gx_rfa  +=p.gx; sum_gy_rfa  +=p.gy; sum_gz_rfa  +=p.gz; got_rfa  =true; break;
      case L_UPPER_NODE: sum_gx_lua  +=p.gx; sum_gy_lua  +=p.gy; sum_gz_lua  +=p.gz; got_lua  =true; break;
      case L_FORE_NODE:  sum_gx_lfa  +=p.gx; sum_gy_lfa  +=p.gy; sum_gz_lfa  +=p.gz; got_lfa  =true; break;
    }
    if (!(got_waist && got_rua && got_rfa && got_lua && got_lfa)) return;
    calibSamples++;
    if (calibSamples % 50 == 0)
      Serial.printf("# Calibrating... %d/%d\n", calibSamples, GYRO_CALIB_SAMPLES);
    if (calibSamples >= GYRO_CALIB_SAMPLES) {
      f_waist.gx_bias=sum_gx_waist/calibSamples; f_waist.gy_bias=sum_gy_waist/calibSamples; f_waist.gz_bias=sum_gz_waist/calibSamples;
      f_rua.gx_bias  =sum_gx_rua/calibSamples;   f_rua.gy_bias  =sum_gy_rua/calibSamples;   f_rua.gz_bias  =sum_gz_rua/calibSamples;
      f_rfa.gx_bias  =sum_gx_rfa/calibSamples;   f_rfa.gy_bias  =sum_gy_rfa/calibSamples;   f_rfa.gz_bias  =sum_gz_rfa/calibSamples;
      f_lua.gx_bias  =sum_gx_lua/calibSamples;   f_lua.gy_bias  =sum_gy_lua/calibSamples;   f_lua.gz_bias  =sum_gz_lua/calibSamples;
      f_lfa.gx_bias  =sum_gx_lfa/calibSamples;   f_lfa.gy_bias  =sum_gy_lfa/calibSamples;   f_lfa.gz_bias  =sum_gz_lfa/calibSamples;
      sysState = S_READY;
      currentMode = MODE_IDLE;
      allNodesSeen = false;
      Serial.println("# [OK] Calibration done! Press 's' to start recording");
    }
    clearGotFlags();
    return;
  }

  NodeFilter* f = nullptr;
  switch (p.node_id) {
    case WAIST_NODE:   f = &f_waist; break;
    case R_UPPER_NODE: f = &f_rua;   break;
    case R_FORE_NODE:  f = &f_rfa;   break;
    case L_UPPER_NODE: f = &f_lua;   break;
    case L_FORE_NODE:  f = &f_lfa;   break;
  }
  if (!f) return;

  float dt = (p.timestamp > f->lastTs && f->lastTs > 0)
             ? (p.timestamp - f->lastTs) * 0.001f : 0.01f;
  updateFilter(*f, p.ax, p.ay, p.az,
               p.gx - f->gx_bias, p.gy - f->gy_bias, p.gz - f->gz_bias, dt);
  f->lastTs = p.timestamp;

  switch (p.node_id) {
    case WAIST_NODE:   got_waist=true; break;
    case R_UPPER_NODE: got_rua  =true; break;
    case R_FORE_NODE:  got_rfa  =true; break;
    case L_UPPER_NODE: got_lua  =true; break;
    case L_FORE_NODE:  got_lfa  =true; break;
  }
}

// ─────────────────────────────────────────────
// Timed frame recording (catch-up version: recovers frames lost during blocking)
// ─────────────────────────────────────────────
void recordFrameTick() {
  if (currentMode != MODE_RECORDING) return;

  if (!allNodesSeen) {
    if (got_waist && got_rua && got_rfa && got_lua && got_lfa) {
      allNodesSeen = true;
      lastFrameTime = millis();   // start timing from the moment all nodes are present
    } else {
      return;
    }
  }

  // compute how many frames to backfill since the last record (one per FRAME_INTERVAL_MS)
  uint32_t now = millis();
  int framesDue = (now - lastFrameTime) / FRAME_INTERVAL_MS;
  if (framesDue <= 0) return;

  for (int n = 0; n < framesDue; n++) {
    if (rec_len >= MAX_FRAMES) {
      Serial.println("# Buffer full (auto-stop)");
      currentMode = MODE_IDLE;
      pendingInference = true;
      return;
    }
    // fill with the latest filtered values (no new IMU data during blocking, reuse most recent)
    rec_buffer[rec_len][0]=f_waist.roll;  rec_buffer[rec_len][1]=f_waist.pitch;
    rec_buffer[rec_len][2]=f_rua.roll;    rec_buffer[rec_len][3]=f_rua.pitch;
    rec_buffer[rec_len][4]=f_rfa.roll;    rec_buffer[rec_len][5]=f_rfa.pitch;
    rec_buffer[rec_len][6]=f_lua.roll;    rec_buffer[rec_len][7]=f_lua.pitch;
    rec_buffer[rec_len][8]=f_lfa.roll;    rec_buffer[rec_len][9]=f_lfa.pitch;
    rec_len++;

    if (rec_len % 100 == 0)
      Serial.printf("# Recording... %d frames (%.1f s)\n", rec_len, rec_len / 100.0f);
  }
  lastFrameTime += (uint32_t)framesDue * FRAME_INTERVAL_MS;
}

// ─────────────────────────────────────────────
// Node connection monitor (unchanged)
// ─────────────────────────────────────────────
void checkNodeConnections() {
  if (millis() - lastNodeCheck < 2000) return;
  lastNodeCheck = millis();

  uint32_t now = millis();
  bool anyMissing = false;
  for (int i = 1; i <= 5; i++) {
    bool connected = (nodeLastSeen[i] > 0 && now - nodeLastSeen[i] < NODE_TIMEOUT_MS);
    if (!connected) {
      Serial.printf("# [!] Node offline: %s (node %d)\n", nodeNames[i], i);
      anyMissing = true;
    }
  }
  if (!anyMissing && nodeLastSeen[1] > 0)
    Serial.println("# [OK] All nodes online");
}

// ─────────────────────────────────────────────
// setup (only the WiFi.begin() line changed)
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // changed: added password
  Serial.print("# Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n# WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n# WiFi connection failed, inference results will not be uploaded");

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("# MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed, restarting...");
    while(1) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.println("#");
  Serial.println("# ==============================");
  Serial.println("#    Forehand Motion Analyzer");
  Serial.println("# ------------------------------");
  Serial.println("#   c -> Calibrate (keep all nodes still)");
  Serial.println("#   s -> Start recording");
  Serial.println("#   s -> Stop recording + auto-analyze");
  Serial.println("#   r -> Reset");
  Serial.println("# ==============================");
  Serial.println("# Press 'c' to calibrate first (or tap Calibrate in the App)...");

  // set command baseline: only respond to UI clicks after boot, ignore historical commands
  pollCommands(true);
}

// ─────────────────────────────────────────────
// Command action functions (shared by serial and UI buttons)
// ─────────────────────────────────────────────
void doCalibrate() {
  if (sysState == S_CALIB_GYRO) return;
  calibSamples = 0;
  sum_gx_waist=sum_gy_waist=sum_gz_waist=0;
  sum_gx_rua  =sum_gy_rua  =sum_gz_rua  =0;
  sum_gx_rfa  =sum_gy_rfa  =sum_gz_rfa  =0;
  sum_gx_lua  =sum_gy_lua  =sum_gz_lua  =0;
  sum_gx_lfa  =sum_gy_lfa  =sum_gz_lfa  =0;
  sysState = S_CALIB_GYRO;
  currentMode = MODE_IDLE;
  Serial.println("# Starting calibration, keep all sensors still...");
}

void doStart() {
  if (sysState != S_READY) {
    Serial.println("# Not calibrated yet, please calibrate first");
    return;
  }
  if (currentMode != MODE_RECORDING) {
    rec_len = 0;
    allNodesSeen = false;
    lastFrameTime = millis();
    currentMode = MODE_RECORDING;
    Serial.println("# Recording started, stop when the motion is complete...");
  }
}

void doStop() {
  if (currentMode == MODE_RECORDING) {
    currentMode = MODE_IDLE;
    Serial.printf("# Recording ended, %d frames (%.1f s)\n", rec_len, rec_len / 100.0f);
    pendingInference = true;   // defer to loop to avoid running inference while the HTTPS connection holds memory
  }
}

// serial 's' is a start/stop toggle, keep original behavior
void doToggleRecord() {
  if (sysState != S_READY) {
    Serial.println("# Not calibrated yet, please press 'c' first");
    return;
  }
  if (currentMode != MODE_RECORDING) doStart();
  else                               doStop();
}

void doReset() {
  sysState = S_IDLE;
  currentMode = MODE_IDLE;
  rec_len = 0;
  Serial.println("# Reset done, press 'c' to recalibrate");
}

// ─────────────────────────────────────────────
// Poll the Supabase commands table
// ─────────────────────────────────────────────
long lastCmdId = -1;                 // last processed command id (-1 = not initialized yet)
uint32_t lastCmdPoll = 0;
const uint32_t CMD_POLL_INTERVAL_MS = 1000;   // query once per second

// Poll the commands table.
// applyBaseline=true only records the baseline, does not execute commands (used at boot).
void pollCommands(bool applyBaseline = false) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url;
  if (applyBaseline) {
    // baseline mode: only take the id of the latest row
    url = String(SUPABASE_URL) +
          "/rest/v1/commands?select=id,type&order=id.desc&limit=1";
  } else {
    // normal mode: take all commands greater than lastCmdId, execute ascending (avoid missing intermediate commands)
    url = String(SUPABASE_URL) +
          "/rest/v1/commands?select=id,type&id=gt." + String(lastCmdId) +
          "&order=id.asc&limit=10";
  }
  http.begin(client, url);
  http.setTimeout(1500);           // single poll blocks at most 1.5 s, to avoid tanking the frame rate
  http.addHeader("apikey",        SUPABASE_ANON);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON);

  int code = http.GET();
  if (code != 200) { http.end(); return; }

  String payload = http.getString();
  http.end();

  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, payload) != DeserializationError::Ok) return;
  if (!doc.is<JsonArray>()) return;

  // baseline mode
  if (applyBaseline) {
    if (doc.size() == 0) { lastCmdId = 0; return; }
    long id = doc[0]["id"] | 0;
    lastCmdId = id;
    Serial.printf("# [command] baseline id=%ld (only later clicks will respond)\n", id);
    return;
  }

  // normal mode: execute all new commands one by one
  for (JsonObject cmd : doc.as<JsonArray>()) {
    long id          = cmd["id"] | -1;
    const char* type = cmd["type"] | "";
    if (id <= lastCmdId) continue;
    lastCmdId = id;
    Serial.printf("# [command] received UI command: %s (id=%ld)\n", type, id);
    if      (strcmp(type, "calibrate") == 0) doCalibrate();
    else if (strcmp(type, "start")     == 0) doStart();
    else if (strcmp(type, "stop")      == 0) doStop();
    else if (strcmp(type, "reset")     == 0) doReset();
  }
}

// called in loop at a throttled interval
void pollCommandsThrottled() {
  // lengthen the poll interval while recording to reduce HTTPS blocking impact on frame rate (still receives Stop within a few seconds)
  uint32_t interval = (currentMode == MODE_RECORDING) ? 2500 : CMD_POLL_INTERVAL_MS;
  if (millis() - lastCmdPoll < interval) return;
  lastCmdPoll = millis();
  pollCommands(false);
}

// ─────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────
void loop() {
  checkNodeConnections();
  recordFrameTick();
  pollCommandsThrottled();         // poll UI button commands

  // deferred inference: by now pollCommands has returned, the HTTPS connection is released, memory is sufficient
  if (pendingInference) {
    pendingInference = false;
    runInference();
  }

  // serial commands (keep original manual operation)
  if (Serial.available()) {
    char cmd = Serial.read();
    if      (cmd == 'c' || cmd == 'C') doCalibrate();
    else if (cmd == 's' || cmd == 'S') doToggleRecord();
    else if (cmd == 'r' || cmd == 'R') doReset();
  }
}
