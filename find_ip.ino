#include <WiFi.h>

// Reuse your existing network credentials.
static const char* ssid = "Presus";
static const char* password = "password321";

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Print once in setup over wired USB serial.
}
