// =====================================================================
//  MarioKart - ESP32-C6 robot firmware
//  Two DC motors (L298N) + weapon servo, driven from mariokartfinal.html
//  over Bluetooth Low Energy OR the robot's own WiFi network.
// =====================================================================
//
//  WIRING (unchanged from the previous build)
//    L298N ENA -> GPIO 6      L298N ENB -> GPIO 20
//    L298N IN1 -> GPIO 7      L298N IN3 -> GPIO 22
//    L298N IN2 -> GPIO 10     L298N IN4 -> GPIO 21
//    Servo signal -> GPIO 18  (red -> L298N 5V rail, brown -> GND rail)
//    ESP32 GND, L298N GND, servo GND all joined.  ENA/ENB jumper caps OFF.
//
//  TWO WAYS TO CONNECT (both run at the same time)
//    Bluetooth: open mariokartfinal.html on the laptop in Chrome / Edge /
//               Opera, or Brave with brave://flags/#brave-web-bluetooth-api
//               enabled. Click "Connect via Bluetooth".
//    WiFi:      any browser, any device (Brave, Firefox, Safari, phones).
//               Join the WiFi network named DEVICE_NAME (password
//               WIFI_PASSWORD), then open http://192.168.4.1 - the robot
//               serves the controls itself.
//
//  ARDUINO IDE SETUP
//    1. Boards Manager: "esp32" by Espressif Systems, version 3.x.
//       Board: "ESP32C6 Dev Module".
//    2. Tools -> Partition Scheme -> "Huge APP (3MB No OTA/1MB SPIFFS)".
//       (WiFi + Bluetooth together are too big for the default partition.
//        Or set ENABLE_BLE to 0 below for WiFi only - that fits the default.)
//    3. Library Manager: "NimBLE-Arduino" by h2zero, version 2.x
//       (not needed if ENABLE_BLE is 0).
//       WiFi and WebServer come with the esp32 package - nothing else needed.
//    4. Keep page.h in the same folder as this file (it holds the web page).
//    5. Upload. Serial Monitor at 115200 baud shows everything the robot
//       receives and does.
//
//  HARDWARE SELF-TEST (no laptop needed)
//    Press the BOOT button on the ESP32 after it has started, or click
//    "Run hardware test" on the page. The robot runs: left motor forward,
//    left reverse, right forward, right reverse, servo out, servo back.
//    If a step doesn't move, that part's wiring/power is the problem.
//
//  COMMANDS (text, same over Bluetooth and WiFi)
//    M:<left>,<right>  motor speeds -255..255      e.g. M:200,-200
//    J:<dir>,<speed>,<rest>  weapon jog: dir -1 = L, 0 = none, 1 = R;
//                      speed = sweep speed while held (deg/s);
//                      rest = glide-back speed when released (deg/s, 0 = stay)
//                      e.g. J:1,120,60
//    Several commands can share one message, separated by ';'
//    e.g. M:255,255;J:0,120,60  (this is what the page sends)
//    X                 stop everything (motors + abort self-test)
//    T                 run the hardware self-test
//    P                 ping (no action, just returns status)
//  STATUS (robot -> page): ST:<left>,<right>,<servo>,<commandCount>,<testStep>
// =====================================================================

// ----------------------------- settings ------------------------------
// Turn either connection off here. Both on needs Partition Scheme "Huge APP".
// WiFi only (ENABLE_BLE 0) fits the default partition.
#define ENABLE_WIFI 1                          // 0 = Bluetooth only (uses less power)
#define ENABLE_BLE  1                          // 0 = WiFi only (much smaller sketch)

#if !ENABLE_WIFI && !ENABLE_BLE
#error "Enable at least one of ENABLE_WIFI / ENABLE_BLE"
#endif

#if ENABLE_BLE
#include <NimBLEDevice.h>
#endif
#if ENABLE_WIFI
#include <WiFi.h>
#include <WebServer.h>
#include "page.h"
#endif

const char* DEVICE_NAME   = "MarioKart-ESP2";  // Bluetooth name AND WiFi network name - change per robot
const char* WIFI_PASSWORD = "mariokart2";       // at least 8 characters - make it different per car
const int   WIFI_CHANNEL  = 6;                 // spread cars across 1, 6 and 11 to reduce interference

// Bluetooth UUIDs (must match mariokartfinal.html)
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // page -> robot commands
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // robot -> page status

// Pins
#define PIN_ENA   6
#define PIN_IN1   7
#define PIN_IN2   10
#define PIN_ENB   20
#define PIN_IN3   22
#define PIN_IN4   21
#define PIN_SERVO 18
#define PIN_BOOT_BUTTON 9      // the board's BOOT button - used to start the self-test

// Motors
const uint32_t MOTOR_PWM_HZ   = 1000;   // L298N works best at low PWM frequencies
const uint8_t  MOTOR_PWM_BITS = 8;      // duty 0..255
const int      RAMP_STEP      = 15;     // max duty change per ramp tick (0 -> 255 in ~170 ms)
const unsigned long RAMP_MS   = 10;
const unsigned long WATCHDOG_MS = 400;  // stop motors if no command for this long

// Servo
const int SERVO_MIN_US     = 500;       // pulse at 0 degrees
const int SERVO_MAX_US     = 2500;      // pulse at 180 degrees
const int SERVO_MIN_ANGLE  = 0;         // tighten to protect your weapon mount
const int SERVO_MAX_ANGLE  = 180;
const int SERVO_REST_ANGLE = 90;        // rest position the arm glides back to (centre)
const bool SERVO_INVERT    = false;     // true if L sweeps the arm the wrong way on your car
const int SERVO_MAX_SPEED  = 300;       // deg/s cap for the page's Speed sliders (gentle on the 5V regulator)
const int DEFAULT_REST_SPEED = 60;      // deg/s used to bring the arm home on disconnect
const int SERVO_TEST_SWING = 45;        // self-test: rest -45 deg, rest +45 deg, back to rest
const int SERVO_STEP_DEG   = 3;         // self-test slew: 3 deg / 10 ms
const unsigned long SERVO_STEP_MS = 10;
const uint32_t SERVO_PWM_HZ   = 50;
const uint8_t  SERVO_PWM_BITS = 14;

// ------------------------------- state -------------------------------
volatile int targetLeft = 0, targetRight = 0;     // set by commands
int currentLeft = 0, currentRight = 0;            // actually applied (ramped)
volatile int servoTarget = SERVO_REST_ANGLE;     // used by the self-test
int servoCurrent = SERVO_REST_ANGLE;              // angle actually being output
float servoPos = SERVO_REST_ANGLE;                // smooth position for jogging
volatile int jogDir = 0;                          // -1 = L held, 1 = R held, 0 = released
volatile int jogSpeed = 120;                      // deg/s while held
volatile int jogRestSpeed = DEFAULT_REST_SPEED;   // deg/s glide back (0 = stay put)
volatile unsigned long lastJogCmdMs = 0;
volatile unsigned long lastMotorCmdMs = 0;
volatile uint32_t commandCount = 0;
volatile bool stopRequested = false;
volatile bool testRequested = false;
volatile bool bleConnected = false;

bool pwmOk = true;
unsigned long lastRampMs = 0, lastServoMs = 0, lastStatusMs = 0, lastReportMs = 0;
int lastLoggedLeft = 9999, lastLoggedRight = 9999;

// Self-test sequence
struct TestStep { const char* name; int left; int right; int servo; unsigned long ms; };
const TestStep TEST_STEPS[] = {
  { "left motor forward",  200,    0, SERVO_REST_ANGLE, 1200 },
  { "left motor reverse", -200,    0, SERVO_REST_ANGLE, 1200 },
  { "right motor forward",   0,  200, SERVO_REST_ANGLE, 1200 },
  { "right motor reverse",   0, -200, SERVO_REST_ANGLE, 1200 },
  { "servo left",            0,    0, SERVO_REST_ANGLE - SERVO_TEST_SWING, 1000 },
  { "servo right",           0,    0, SERVO_REST_ANGLE + SERVO_TEST_SWING, 1000 },
  { "servo back to rest",    0,    0, SERVO_REST_ANGLE, 1000 },
};
const int TEST_COUNT = sizeof(TEST_STEPS) / sizeof(TEST_STEPS[0]);
int testStep = 0;                 // 0 = not running, 1..TEST_COUNT = running that step
unsigned long testStepStartMs = 0;

#if ENABLE_BLE
NimBLECharacteristic* txChar = nullptr;
#endif
#if ENABLE_WIFI
WebServer web(80);
#endif

// ============================ motor output ===========================
void applyMotor(int pinEn, int pinA, int pinB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  ledcWrite(pinEn, (uint32_t)abs(speed));
}

void stopMotorsNow() {
  targetLeft = 0; targetRight = 0;
  currentLeft = 0; currentRight = 0;
  applyMotor(PIN_ENA, PIN_IN1, PIN_IN2, 0);
  applyMotor(PIN_ENB, PIN_IN3, PIN_IN4, 0);
}

int stepToward(int cur, int target, int step) {
  if (cur < target) return min(cur + step, target);
  if (cur > target) return max(cur - step, target);
  return cur;
}

void rampMotors() {
  unsigned long now = millis();
  if (now - lastRampMs < RAMP_MS) return;
  lastRampMs = now;
  int l = stepToward(currentLeft, targetLeft, RAMP_STEP);
  int r = stepToward(currentRight, targetRight, RAMP_STEP);
  if (l != currentLeft)  { currentLeft = l;  applyMotor(PIN_ENA, PIN_IN1, PIN_IN2, currentLeft); }
  if (r != currentRight) { currentRight = r; applyMotor(PIN_ENB, PIN_IN3, PIN_IN4, currentRight); }
}

// ============================ servo output ===========================
void writeServo(int angle) {
  long us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  const long periodUs = 1000000L / SERVO_PWM_HZ;
  const long maxDuty = (1L << SERVO_PWM_BITS) - 1;
  ledcWrite(PIN_SERVO, (uint32_t)(us * maxDuty / periodUs));
}

// Self-test: step toward servoTarget. Otherwise: sweep while L/R is held at
// jogSpeed, glide back to SERVO_REST_ANGLE at jogRestSpeed when released.
void rampServo() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastServoMs;
  if (dtMs < SERVO_STEP_MS) return;
  lastServoMs = now;
  float dt = min(dtMs, 100UL) / 1000.0f;

  if (testStep > 0) {
    servoPos = stepToward((int)lroundf(servoPos), servoTarget, SERVO_STEP_DEG);
  } else {
    int dir = SERVO_INVERT ? -jogDir : jogDir;
    if (dir != 0) {
      servoPos += dir * jogSpeed * dt;
    } else if (jogRestSpeed > 0) {
      float diff = SERVO_REST_ANGLE - servoPos, step = jogRestSpeed * dt;
      servoPos += (fabsf(diff) <= step) ? diff : (diff > 0 ? step : -step);
    }
    servoPos = constrain(servoPos, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
  }
  int a = (int)lroundf(servoPos);
  if (a != servoCurrent) { servoCurrent = a; writeServo(a); }
}

// ============================== commands =============================
// Called from both the Bluetooth task and the WiFi handler.
void handleOne(const char* msg) {
  if (msg[0] == 'M' && msg[1] == ':') {
    int l = 0, r = 0;
    if (sscanf(msg + 2, "%d,%d", &l, &r) == 2) {
      lastMotorCmdMs = millis();
      if (testStep == 0) {                       // ignore drive while the self-test runs
        targetLeft = constrain(l, -255, 255);
        targetRight = constrain(r, -255, 255);
      }
    }
  } else if (msg[0] == 'J' && msg[1] == ':') {
    int dir = 0, speed = 0, rest = 0;
    if (sscanf(msg + 2, "%d,%d,%d", &dir, &speed, &rest) == 3) {
      jogDir = constrain(dir, -1, 1);
      jogSpeed = constrain(speed, 0, SERVO_MAX_SPEED);
      jogRestSpeed = constrain(rest, 0, SERVO_MAX_SPEED);
      lastJogCmdMs = millis();
    }
  } else if (msg[0] == 'X') {
    stopRequested = true;
    Serial.println("[CMD] STOP");
  } else if (msg[0] == 'T') {
    testRequested = true;
    Serial.println("[CMD] hardware test requested");
  }
  // 'P' (ping) and anything else: no action, status is returned anyway
}

// A message may hold several commands separated by ';'
void handleCommand(const char* msg) {
  commandCount++;
  char buf[96];
  strncpy(buf, msg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* save = nullptr;
  for (char* part = strtok_r(buf, ";", &save); part; part = strtok_r(nullptr, ";", &save)) handleOne(part);
}

void buildStatus(char* buf, size_t len) {
  snprintf(buf, len, "ST:%d,%d,%d,%lu,%d",
           currentLeft, currentRight, servoCurrent, (unsigned long)commandCount, testStep);
}

// ============================= Bluetooth =============================
#if ENABLE_BLE
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    bleConnected = true;
    Serial.printf("[BLE] connected: %s\n", info.getAddress().toString().c_str());
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    bleConnected = false;
    stopRequested = true;                         // safety: stop on link loss
    jogDir = 0;                                   // and bring the weapon home
    if (jogRestSpeed == 0) jogRestSpeed = DEFAULT_REST_SPEED;
    Serial.printf("[BLE] disconnected (reason %d) - motors stopped, advertising again\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    NimBLEAttValue v = c->getValue();
    char buf[96];
    size_t n = v.length();
    if (n > sizeof(buf) - 1) n = sizeof(buf) - 1;
    memcpy(buf, v.data(), n);
    buf[n] = '\0';                                // always a clean, terminated string
    handleCommand(buf);
  }
};

void setupBluetooth() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);
  NimBLECharacteristic* rx = service->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  txChar = service->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  service->start();

  // Service UUID in the advert (the page filters on it), name in the scan
  // response (a 128-bit UUID + name don't both fit in one 31-byte advert).
  NimBLEAdvertisementData adv;
  adv.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  adv.addServiceUUID(SERVICE_UUID);
  NimBLEAdvertisementData scan;
  scan.setName(DEVICE_NAME);

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setAdvertisementData(adv);
  advertising->setScanResponseData(scan);
  advertising->enableScanResponse(true);
  bool ok = advertising->start();
  Serial.printf("[BLE] advertising as \"%s\": %s\n", DEVICE_NAME, ok ? "OK" : "FAILED");
}
#endif

// =============================== WiFi ================================
#if ENABLE_WIFI
void handleRoot() {
  web.send_P(200, "text/html", INDEX_HTML);
}

void handleCmd() {
  if (web.hasArg("c")) {
    String c = web.arg("c");
    handleCommand(c.c_str());
  }
  char status[48];
  buildStatus(status, sizeof(status));
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "text/plain", status);
}

void setupWifi() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(DEVICE_NAME, WIFI_PASSWORD, WIFI_CHANNEL);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // lower power = smaller current spikes on the 5V regulator
  web.on("/", handleRoot);
  web.on("/cmd", handleCmd);
  web.onNotFound([]() { web.sendHeader("Location", "/"); web.send(302, "text/plain", ""); });
  web.begin();
  Serial.printf("[WIFI] network \"%s\" (password \"%s\"): %s - open http://%s\n",
                DEVICE_NAME, WIFI_PASSWORD, ok ? "OK" : "FAILED", WiFi.softAPIP().toString().c_str());
}
#endif

// ============================= self-test =============================
void startTest() {
  testStep = 1;
  testStepStartMs = millis();
  Serial.println("[TEST] starting hardware test");
}

void runTest() {
  if (testStep == 0) return;
  const TestStep& s = TEST_STEPS[testStep - 1];
  static int announced = 0;
  if (announced != testStep) {
    announced = testStep;
    Serial.printf("[TEST] step %d/%d: %s\n", testStep, TEST_COUNT, s.name);
  }
  targetLeft = s.left;
  targetRight = s.right;
  servoTarget = s.servo;
  lastMotorCmdMs = millis();                     // keep the watchdog quiet during the test
  if (millis() - testStepStartMs >= s.ms) {
    testStep++;
    testStepStartMs = millis();
    if (testStep > TEST_COUNT) {
      testStep = 0;
      announced = 0;
      targetLeft = targetRight = 0;
      servoTarget = SERVO_REST_ANGLE;
      Serial.println("[TEST] done");
    }
  }
}

// =============================== setup ===============================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== MarioKart ===");

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

  // PWM channels - each result is printed so a failure is obvious
  bool a = ledcAttach(PIN_ENA, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  bool b = ledcAttach(PIN_ENB, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  bool s = ledcAttach(PIN_SERVO, SERVO_PWM_HZ, SERVO_PWM_BITS);
  pwmOk = a && b && s;
  Serial.printf("[PWM] ENA(GPIO%d): %s  ENB(GPIO%d): %s  SERVO(GPIO%d): %s\n",
                PIN_ENA, a ? "OK" : "FAILED", PIN_ENB, b ? "OK" : "FAILED", PIN_SERVO, s ? "OK" : "FAILED");

  stopMotorsNow();
  writeServo(SERVO_REST_ANGLE);

#if ENABLE_BLE
  setupBluetooth();
#endif
#if ENABLE_WIFI
  setupWifi();
#endif
  Serial.println("[READY] press BOOT to run the hardware test, or connect from the page");
}

// ================================ loop ===============================
void loop() {
#if ENABLE_WIFI
  web.handleClient();
#endif

  if (stopRequested) {
    stopRequested = false;
    testStep = 0;
    stopMotorsNow();
    jogDir = 0;
  }

  if (testRequested) { testRequested = false; startTest(); }

  // BOOT button -> self-test (debounced, on press)
  static bool lastBtn = HIGH;
  static unsigned long btnMs = 0;
  bool btn = digitalRead(PIN_BOOT_BUTTON);
  if (btn != lastBtn && millis() - btnMs > 50) {
    btnMs = millis();
    lastBtn = btn;
    if (btn == LOW && testStep == 0) startTest();
  }

  runTest();

  // Watchdog. Read the timestamp first; signed difference survives a
  // command arriving between the two reads.
  unsigned long last = lastMotorCmdMs;
  long silence = (long)(millis() - last);
  bool moving = targetLeft != 0 || targetRight != 0 || currentLeft != 0 || currentRight != 0;
  if (moving && testStep == 0 && silence > (long)WATCHDOG_MS) {
    Serial.println("[SAFETY] no command for 400 ms - motors stopped");
    stopMotorsNow();
  }

  // Weapon watchdog: L/R released if the page goes quiet
  if (jogDir != 0 && (long)(millis() - lastJogCmdMs) > (long)WATCHDOG_MS) jogDir = 0;

  rampMotors();
  rampServo();

  static int lastLoggedJog = 0;
  if (jogDir != lastLoggedJog) {
    lastLoggedJog = jogDir;
    Serial.printf("[WEAPON] %s (swing speed %d deg/s, retract speed %d deg/s)\n",
                  lastLoggedJog < 0 ? "sweep L" : lastLoggedJog > 0 ? "sweep R" : "released", (int)jogSpeed, (int)jogRestSpeed);
  }

  // Log motor changes (not every keepalive)
  if (targetLeft != lastLoggedLeft || targetRight != lastLoggedRight) {
    lastLoggedLeft = targetLeft; lastLoggedRight = targetRight;
    Serial.printf("[DRIVE] target left=%d right=%d\n", lastLoggedLeft, lastLoggedRight);
  }

#if ENABLE_BLE
  // Status to the page over Bluetooth, 4x per second
  if (bleConnected && txChar && millis() - lastStatusMs > 250) {
    lastStatusMs = millis();
    char st[48];
    buildStatus(st, sizeof(st));
    txChar->setValue((uint8_t*)st, strlen(st));
    txChar->notify();
  }
#endif

  // Heartbeat in the Serial Monitor every 3 s
  if (millis() - lastReportMs > 3000) {
    lastReportMs = millis();
    Serial.printf("[STATUS] cmds=%lu  left=%d right=%d  servo=%d  ble=%s  pwm=%s\n",
                  (unsigned long)commandCount, currentLeft, currentRight, servoCurrent,
                  bleConnected ? "yes" : "no", pwmOk ? "ok" : "FAILED");
  }

  delay(2);
}
