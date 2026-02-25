// Arduino motor controller from 8-digit packet stream:
// packet = RL + SSS + RR + SSS
// Example: 11800110

static const unsigned long PACKET_TIMEOUT_MS = 1000;
static const size_t PACKET_LEN = 8;
static const bool LOG_PACKETS = true;
static const unsigned long LOG_PACKET_EVERY_MS = 200;

// Pins from hardware.MD
static const uint8_t LEFT_PWM_PIN = 3;
static const uint8_t LEFT_FWD_PIN = 22;
static const uint8_t LEFT_REV_PIN = 23;

static const uint8_t RIGHT_PWM_PIN = 2;
static const uint8_t RIGHT_FWD_PIN = 24;
static const uint8_t RIGHT_REV_PIN = 25;

unsigned long lastPacketMs = 0;
unsigned long lastPacketLogMs = 0;
char packetBuf[PACKET_LEN];
size_t packetPos = 0;

int parseSpeed3(const char* p) {
  int v = (p[0] - '0') * 100 + (p[1] - '0') * 10 + (p[2] - '0');
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return v;
}

void applyWheel(uint8_t pwmPin, uint8_t fwdPin, uint8_t revPin, bool reverse, int speed) {
  if (speed <= 0) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  if (reverse) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
  } else {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
  }

  analogWrite(pwmPin, speed);
}

void stopAll() {
  applyWheel(LEFT_PWM_PIN, LEFT_FWD_PIN, LEFT_REV_PIN, false, 0);
  applyWheel(RIGHT_PWM_PIN, RIGHT_FWD_PIN, RIGHT_REV_PIN, false, 0);
}

void applyPacket(const char* pkt) {
  bool leftReverse = (pkt[0] == '1');
  int leftSpeed = parseSpeed3(&pkt[1]);

  bool rightReverse = (pkt[4] == '1');
  int rightSpeed = parseSpeed3(&pkt[5]);

  applyWheel(LEFT_PWM_PIN, LEFT_FWD_PIN, LEFT_REV_PIN, leftReverse, leftSpeed);
  applyWheel(RIGHT_PWM_PIN, RIGHT_FWD_PIN, RIGHT_REV_PIN, rightReverse, rightSpeed);

  unsigned long now = millis();
  if (LOG_PACKETS && (now - lastPacketLogMs >= LOG_PACKET_EVERY_MS)) {
    Serial.print("APPLY ");
    for (size_t i = 0; i < PACKET_LEN; ++i) Serial.print(pkt[i]);
    Serial.print(" | L ");
    Serial.print(leftReverse ? "REV " : "FWD ");
    Serial.print(leftSpeed);
    Serial.print(" | R ");
    Serial.print(rightReverse ? "REV " : "FWD ");
    Serial.println(rightSpeed);
    lastPacketLogMs = now;
  }
}

void setup() {
  Serial.begin(115200);   // USB debug
  Serial1.begin(115200);  // ESP32 -> Mega (Mega RX1 pin 19)

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_REV_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_REV_PIN, OUTPUT);

  stopAll();
  lastPacketMs = millis();
  Serial.println("Arduino motor controller ready.");
}

void loop() {
  while (Serial1.available() > 0) {
    int r = Serial1.read();
    if (r < 0) break;
    char c = (char)r;

    if (c == '\n' || c == '\r') {
      if (packetPos == PACKET_LEN) {
        applyPacket(packetBuf);
        lastPacketMs = millis();
      }
      packetPos = 0;
      continue;
    }

    if (c >= '0' && c <= '9') {
      if (packetPos < PACKET_LEN) {
        packetBuf[packetPos++] = c;
      } else {
        packetPos = 0;
      }
    } else {
      packetPos = 0;
    }
  }

  if (millis() - lastPacketMs > PACKET_TIMEOUT_MS) {
    stopAll();
  }
}
