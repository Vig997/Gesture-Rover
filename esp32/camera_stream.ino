#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// Keep Wi-Fi credentials and port aligned with esp32/wifi_connect.ino.
static const char* ssid = "Presus";
static const char* password = "password321";
static const uint16_t STREAM_PORT = 12345;
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

WebServer server(STREAM_PORT);

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
    current_profile_index = 2;  // Start aggressively at QVGA for FPS headroom.
    config.frame_size = STREAM_PROFILES[current_profile_index].frame_size;
    config.jpeg_quality = STREAM_PROFILES[current_profile_index].jpeg_quality;
    config.fb_count = 2;
  } else {
    min_profile_index = 2;       // Avoid high resolutions without PSRAM.
    current_profile_index = 3;   // Start at QQVGA for stability.
    config.frame_size = STREAM_PROFILES[current_profile_index].frame_size;
    config.jpeg_quality = STREAM_PROFILES[current_profile_index].jpeg_quality;
    config.fb_count = 1;
  }

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

void handle_root() {
  String msg = "ESP32-CAM stream ready\n";
  msg += "GET /stream for MJPEG\n";
  msg += "http://" + WiFi.localIP().toString() + ":" + String(STREAM_PORT) + "/stream\n";
  server.send(200, "text/plain", msg);
}

void handle_stream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n\r\n");

  Serial.println("Stream client connected.");
  unsigned long window_start_ms = millis();
  unsigned long last_adapt_ms = 0;
  uint32_t frames_sent = 0;
  uint32_t bytes_sent = 0;

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
      Serial.println("Camera capture failed.");
      delay(10);
      continue;
    }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Content-Length: ");
    client.print(fb->len);
    client.print("\r\n\r\n");
    size_t wrote = client.write(fb->buf, fb->len);
    client.print("\r\n");

    if (wrote != fb->len) {
      esp_camera_fb_return(fb);
      break;
    }

    frames_sent++;
    bytes_sent += fb->len;
    esp_camera_fb_return(fb);

    unsigned long now = millis();
    unsigned long elapsed = now - window_start_ms;
    if (elapsed >= ADAPT_WINDOW_MS) {
      float fps = (1000.0f * frames_sent) / float(elapsed);
      float avg_kb = frames_sent > 0 ? (bytes_sent / 1024.0f) / frames_sent : 0.0f;

      if ((now - last_adapt_ms) >= ADAPT_COOLDOWN_MS) {
        if (fps < FPS_DOWNGRADE_THRESHOLD && current_profile_index < max_profile_index) {
          if (apply_stream_profile(current_profile_index + 1)) {
            last_adapt_ms = now;
          }
        } else if (fps > FPS_UPGRADE_THRESHOLD && current_profile_index > min_profile_index) {
          if (apply_stream_profile(current_profile_index - 1)) {
            last_adapt_ms = now;
          }
        }
      }

      Serial.printf(
          "stream fps=%.1f target=%.1f avg=%.1fKB profile=%s\n",
          fps,
          TARGET_FPS,
          avg_kb,
          STREAM_PROFILES[current_profile_index].label);

      window_start_ms = now;
      frames_sent = 0;
      bytes_sent = 0;
    }

    delay(1);
    yield();
  }

  Serial.println("Stream client disconnected.");
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

  server.on("/", HTTP_GET, handle_root);
  server.on("/stream", HTTP_GET, handle_stream);
  server.begin();

  Serial.print("HTTP server listening on port ");
  Serial.println(STREAM_PORT);
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.print(STREAM_PORT);
  Serial.println("/stream");
}

void loop() {
  server.handleClient();
  delay(1);
}
