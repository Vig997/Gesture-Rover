#include <WiFi.h>

// Set your Wi-Fi credentials directly here.
static const char* ssid = "Presus"
static const char* password = "password321";
static const uint16_t TCP_PORT = 12345;

// ESP32 UART2 pins (edit if your board wiring is different).
static const int ESP32_UART2_RX = 14;
static const int ESP32_UART2_TX = 15;

static const size_t PACKET_LEN = 8;  // RL + SSS + RR + SSS (ASCII digits)

WiFiServer server(TCP_PORT);
WiFiClient client;

char packet_buf[PACKET_LEN];
size_t packet_pos = 0;

void connect_wifi() {
  WiFi.mode(WIFI_STA);
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

void setup() {
  Serial.begin(115200);  // USB debug
  Serial2.begin(115200, SERIAL_8N1, ESP32_UART2_RX, ESP32_UART2_TX);  // To Arduino

  connect_wifi();
  server.begin();
  Serial.print("WiFiServer listening on port ");
  Serial.println(TCP_PORT);
}

void handle_byte(uint8_t b) {
  // Accept only digits for stream alignment.
  if (b < '0' || b > '9') {
    packet_pos = 0;
    return;
  }

  packet_buf[packet_pos++] = (char)b;

  if (packet_pos == PACKET_LEN) {
    Serial2.write((uint8_t*)packet_buf, PACKET_LEN);
    Serial2.write('\n');

    Serial.print("FWD ");
    for (size_t i = 0; i < PACKET_LEN; ++i) Serial.print(packet_buf[i]);
    Serial.println();

    packet_pos = 0;
  }
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      packet_pos = 0;
      Serial.println("Client connected.");
    } else {
      delay(5);
      return;
    }
  }

  while (client.available() > 0) {
    int r = client.read();
    if (r < 0) break;
    handle_byte((uint8_t)r);
  }

  if (!client.connected()) {
    Serial.println("Client disconnected.");
    packet_pos = 0;
  }

  yield();
}
