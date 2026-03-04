// Arduino motor controller from 8-digit packet stream:
// packet = RL + SSS + RR + SSS
// Example: 11800110

static const unsigned long PACKET_TIMEOUT_MS = 1000;
static const size_t PACKET_LEN = 8;
static const bool LOG_PACKETS = true;
static const unsigned long LOG_PACKET_EVERY_MS = 200;

struct WheelPins {
  uint8_t pwm;
  uint8_t fwd;
  uint8_t rev;
};

// Pins from WHEELPINS.md
// Right side wheels
static const WheelPins F_RIGHT = {3, 22, 23};
static const WheelPins B_RIGHT = {2, 24, 25};
// Left side wheels
static const WheelPins F_LEFT = {5, 30, 31};
static const WheelPins B_LEFT = {4, 32, 33};

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

void applyWheel(const WheelPins& wheel, bool reverse, int speed) {
  if (speed <= 0) {
    digitalWrite(wheel.fwd, LOW);
    digitalWrite(wheel.rev, LOW);
    analogWrite(wheel.pwm, 0);
    return;
  }

  if (reverse) {
    digitalWrite(wheel.fwd, LOW);
    digitalWrite(wheel.rev, HIGH);
  } else {
    digitalWrite(wheel.fwd, HIGH);
    digitalWrite(wheel.rev, LOW);
  }

  analogWrite(wheel.pwm, speed);
}

void applySide(const WheelPins& frontWheel, const WheelPins& backWheel, bool reverse, int speed) {
  applyWheel(frontWheel, reverse, speed);
  applyWheel(backWheel, reverse, speed);
}

void initWheelPins(const WheelPins& wheel) {
  pinMode(wheel.pwm, OUTPUT);
  pinMode(wheel.fwd, OUTPUT);
  pinMode(wheel.rev, OUTPUT);
}

void stopAll() {
  applySide(F_LEFT, B_LEFT, false, 0);
  applySide(F_RIGHT, B_RIGHT, false, 0);
}

void applyPacket(const char* pkt) {
  bool leftReverse = (pkt[0] == '1');
  int leftSpeed = parseSpeed3(&pkt[1]);

  bool rightReverse = (pkt[4] == '1');
  int rightSpeed = parseSpeed3(&pkt[5]);

  // Packet format stays identical:
  // left channel controls both left wheels, right channel controls both right wheels.
  applySide(F_LEFT, B_LEFT, leftReverse, leftSpeed);
  applySide(F_RIGHT, B_RIGHT, rightReverse, rightSpeed);

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

  initWheelPins(F_LEFT);
  initWheelPins(B_LEFT);
  initWheelPins(F_RIGHT);
  initWheelPins(B_RIGHT);

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
