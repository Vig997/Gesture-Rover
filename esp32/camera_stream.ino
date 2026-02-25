#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include <string.h>

// Keep Wi-Fi credentials and port aligned with esp32/wifi_connect.ino.
static const char* ssid = "Presus";
static const char* password = "password321";
static const uint16_t STREAM_PORT = 12345;

// UDP framing / adaptation settings.
static const uint8_t PROTO_MAGIC_0 = 'U';
static const uint8_t PROTO_MAGIC_1 = 'F';
static const size_t UDP_HEADER_SIZE = 12;
static const size_t UDP_CHUNK_DATA = 1180;  // Keep packet safely under MTU.
static const unsigned long VIEWER_TIMEOUT_MS = 3000;
static const float TARGET_FPS = 15.0f;
static const float FPS_DOWNGRADE_THRESHOLD = 12.0f;
static const float FPS_UPGRADE_THRESHOLD = 20.0f;
static const unsigned long ADAPT_WINDOW_MS = 1500;
static const unsigned long ADAPT_COOLDOWN_MS = 2500;

// AI Thinker ESP32-CAM pin map.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

struct StreamProfile {
  framesize_t frame_size;
  int jpeg_quality;
  const char* label;
};

static const StreamProfile STREAM_PROFILES[] = {
    {FRAMESIZE_VGA, 16, "VGA q16"},
    {FRAMESIZE_CIF, 20, "CIF q20"},
    {FRAMESIZE_QVGA, 24, "QVGA q24"},
    {FRAMESIZE_QQVGA, 30, "QQVGA q30"},
};

static const size_t STREAM_PROFILE_COUNT = sizeof(STREAM_PROFILES) / sizeof(STREAM_PROFILES[0]);
static size_t current_profile_index = 2;
static size_t min_profile_index = 0;
static size_t max_profile_index = STREAM_PROFILE_COUNT - 1;

WiFiUDP udp;
IPAddress viewer_ip(0, 0, 0, 0);
uint16_t viewer_port = 0;
bool viewer_active = false;
uint32_t next_frame_id = 1;
unsigned long last_viewer_ms = 0;
unsigned long last_adapt_ms = 0;
unsigned long window_start_ms = 0;
uint32_t frames_sent_window = 0;
uint32_t bytes_sent_window = 0;
uint32_t dropped_frames_window = 0;

bool apply_stream_profile(size_t index) {
  if (index >= STREAM_PROFILE_COUNT) return false;

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) return false;

  const StreamProfile& profile = STREAM_PROFILES[index];
  int ok_fs = sensor->set_framesize(sensor, profile.frame_size);
  int ok_q = sensor->set_quality(sensor, profile.jpeg_quality);
  if (ok_fs != 0 || ok_q != 0) {
    Serial.printf("Failed to apply profile %s (fs=%d q=%d)\n", profile.label, ok_fs, ok_q);
    return false;
  }

  current_profile_index = index;
  Serial.printf("Stream profile -> %s\n", profile.label);
  return true;
}

bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    min_profile_index = 0;
    current_profile_index = 2;  // Start at QVGA for throughput headroom.
    config.fb_count = 2;
  } else {
    min_profile_index = 2;       // Keep smaller frames without PSRAM.
    current_profile_index = 3;   // Start at QQVGA.
    config.fb_count = 1;
  }

  config.frame_size = STREAM_PROFILES[current_profile_index].frame_size;
  config.jpeg_quality = STREAM_PROFILES[current_profile_index].jpeg_quality;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed (0x%x)\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_brightness(sensor, 0);
    sensor->set_saturation(sensor, 0);
    sensor->set_quality(sensor, STREAM_PROFILES[current_profile_index].jpeg_quality);
    sensor->set_framesize(sensor, STREAM_PROFILES[current_profile_index].frame_size);
  }

  Serial.printf("Initial stream profile: %s\n", STREAM_PROFILES[current_profile_index].label);
  return true;
}

void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

void send_ack() {
  if (!viewer_active) return;
  if (udp.beginPacket(viewer_ip, viewer_port) != 1) return;
  udp.print("ACK");
  udp.endPacket();
}

void handle_viewer_packets() {
  while (true) {
    int packet_size = udp.parsePacket();
    if (packet_size <= 0) break;

    char msg[32];
    int n = udp.read((uint8_t*)msg, sizeof(msg) - 1);
    if (n < 0) n = 0;
    msg[n] = '\0';

    if (strncmp(msg, "HELLO", 5) == 0 || strncmp(msg, "KEEPALIVE", 9) == 0 || strncmp(msg, "PING", 4) == 0) {
      IPAddress new_ip = udp.remoteIP();
      uint16_t new_port = udp.remotePort();
      bool changed = (!viewer_active) || (new_ip != viewer_ip) || (new_port != viewer_port);

      viewer_ip = new_ip;
      viewer_port = new_port;
      viewer_active = true;
      last_viewer_ms = millis();

      if (changed) {
        Serial.printf("Viewer attached: %s:%u\n", viewer_ip.toString().c_str(), viewer_port);
      }
      send_ack();
    }
  }

  if (viewer_active && (millis() - last_viewer_ms > VIEWER_TIMEOUT_MS)) {
    viewer_active = false;
    viewer_port = 0;
    viewer_ip = IPAddress(0, 0, 0, 0);
    Serial.println("Viewer timed out.");
  }
}

bool send_frame_udp(camera_fb_t* fb) {
  if (!viewer_active || viewer_port == 0) return false;
  if (fb == nullptr || fb->len == 0) return false;

  size_t total_chunks_sz = (fb->len + UDP_CHUNK_DATA - 1) / UDP_CHUNK_DATA;
  if (total_chunks_sz == 0 || total_chunks_sz > 65535) return false;

  uint32_t frame_id = next_frame_id++;
  uint16_t total_chunks = (uint16_t)total_chunks_sz;
  size_t offset = 0;

  for (uint16_t chunk_idx = 0; chunk_idx < total_chunks; ++chunk_idx) {
    size_t remaining = fb->len - offset;
    uint16_t payload_len = (uint16_t)(remaining > UDP_CHUNK_DATA ? UDP_CHUNK_DATA : remaining);

    uint8_t header[UDP_HEADER_SIZE];
    header[0] = PROTO_MAGIC_0;
    header[1] = PROTO_MAGIC_1;
    header[2] = (uint8_t)((frame_id >> 24) & 0xFF);
    header[3] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)(frame_id & 0xFF);
    header[6] = (uint8_t)((chunk_idx >> 8) & 0xFF);
    header[7] = (uint8_t)(chunk_idx & 0xFF);
    header[8] = (uint8_t)((total_chunks >> 8) & 0xFF);
    header[9] = (uint8_t)(total_chunks & 0xFF);
    header[10] = (uint8_t)((payload_len >> 8) & 0xFF);
    header[11] = (uint8_t)(payload_len & 0xFF);

    if (udp.beginPacket(viewer_ip, viewer_port) != 1) return false;
    udp.write(header, UDP_HEADER_SIZE);
    udp.write(fb->buf + offset, payload_len);
    if (udp.endPacket() != 1) return false;

    offset += payload_len;
  }

  return true;
}

void maybe_adapt_profile(unsigned long now_ms) {
  unsigned long elapsed = now_ms - window_start_ms;
  if (elapsed < ADAPT_WINDOW_MS) return;

  float fps = (1000.0f * frames_sent_window) / float(elapsed);
  float avg_kb = frames_sent_window > 0 ? (bytes_sent_window / 1024.0f) / frames_sent_window : 0.0f;

  if ((now_ms - last_adapt_ms) >= ADAPT_COOLDOWN_MS) {
    if (fps < FPS_DOWNGRADE_THRESHOLD && current_profile_index < max_profile_index) {
      if (apply_stream_profile(current_profile_index + 1)) {
        last_adapt_ms = now_ms;
      }
    } else if (fps > FPS_UPGRADE_THRESHOLD && current_profile_index > min_profile_index) {
      if (apply_stream_profile(current_profile_index - 1)) {
        last_adapt_ms = now_ms;
      }
    }
  }

  Serial.printf(
      "udp fps=%.1f target=%.1f avg=%.1fKB drop=%u profile=%s viewer=%s:%u\n",
      fps,
      TARGET_FPS,
      avg_kb,
      dropped_frames_window,
      STREAM_PROFILES[current_profile_index].label,
      viewer_active ? viewer_ip.toString().c_str() : "-",
      viewer_active ? viewer_port : 0);

  frames_sent_window = 0;
  bytes_sent_window = 0;
  dropped_frames_window = 0;
  window_start_ms = now_ms;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!init_camera()) {
    Serial.println("Stopping because camera init failed.");
    while (true) {
      delay(1000);
    }
  }

  connect_wifi();
  udp.begin(STREAM_PORT);
  window_start_ms = millis();

  Serial.print("UDP camera stream listening for HELLO on ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(STREAM_PORT);
  Serial.println("Run fpview.py to register as viewer.");
}

void loop() {
  handle_viewer_packets();
  if (!viewer_active) {
    delay(5);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    dropped_frames_window++;
    delay(10);
    return;
  }

  bool sent = send_frame_udp(fb);
  if (sent) {
    frames_sent_window++;
    bytes_sent_window += fb->len;
  } else {
    dropped_frames_window++;
  }
  esp_camera_fb_return(fb);

  maybe_adapt_profile(millis());
  delay(1);
  yield();
}
