#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// Chân động cơ bánh xích (TB6612FNG)
const int PWMA = 32;
const int AIN1 = 33;
const int AIN2 = 25;
const int PWMB = 14;
const int BIN1 = 27;
const int BIN2 = 26;

// Động cơ bước tháp pháo
const int IN1 = 19;
const int IN2 = 21;
const int IN3 = 22;
const int IN4 = 23;

// Servo nòng súng
const int SERVO_PIN = 4;
Servo gunServo;
int gunAngle = 90;
int gunState = 0;
unsigned long lastGunTime = 0;
const int GUN_STEP_DELAY = 40;
const int GUN_MIN_ANGLE = 80;
const int GUN_MAX_ANGLE = 120;

// Cảm biến siêu âm
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const unsigned long ECHO_TIMEOUT = 25000;   // us (~4 m)

const int CH_A = 14;
const int CH_B = 15;

int turretState = 0;
int stepNumber = 0;
unsigned long lastStepTime = 0;
unsigned long lastRadarTime = 0;
bool radarActive = true;

// ---------- 4 MỨC TỐC ĐỘ ----------
const int SPEED_LEVELS[4] = {100, 150, 200, 255};
int maxSpeed = 255;

// ---------- CHẾ ĐỘ TỰ ĐỘNG (đơn vị khoảng cách: cm) ----------
bool autoMode = false;
const int TURN_DIST   = 25;      // cm: gần hơn thì dừng + quay
const int SLOW_START  = 50;      // cm: bắt đầu giảm tốc
const int MIN_SPEED   = 90;
const unsigned long RADAR_PERIOD = 60;    // ms giữa 2 lần quét
const unsigned long TURN_180_MS = 1000;
const unsigned long SETTLE_MS   = 300;

enum AutoState { AUTO_DRIVE, AUTO_TURNING, AUTO_WAIT };
AutoState autoState = AUTO_DRIVE;
unsigned long turnStartTime = 0;
unsigned long waitStartTime = 0;
int nearCount = 0;
int clearCount = 0;             // đếm số lần đo thoáng liên tiếp khi đang chờ
long lastValidDist = 999;       // lần đo hợp lệ gần nhất

const int stepMatrix[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

void chayTien(int tocDo); void chayLui(int tocDo);
void xoayTrai(int tocDo); void xoayPhai(int tocDo); void dungXe();
void quayPhaiTaiCho(int tocDo);
long doKhoangCach(); void capNhatThapPhao(); void xuatXungStep();
void capNhatNongSung();
void xuLyTuDong(long kc); void capNhatQuayDau(); void tatTuDong();

// ---------- PWM động cơ (tương thích core 2.x và 3.x) ----------
void pwmKhoiTao() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
#else
  ledcSetup(CH_A, 1000, 8); ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, 1000, 8); ledcAttachPin(PWMB, CH_B);
#endif
}

void pwmGhi(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pin == PWMA ? CH_A : CH_B, duty);
#endif
}

// ---------- BLE ----------
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      autoMode = false;
      dungXe();
      turretState = 0;
      gunState = 0;
      pServer->startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      auto rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        char command = rxValue[0];

        switch (command) {
          case '1': if (!autoMode) chayTien(maxSpeed);                  break;
          case '2': if (!autoMode) chayLui(maxSpeed);                   break;
          case '3': if (!autoMode) xoayTrai(min(maxSpeed, 200));        break;
          case '4': if (!autoMode) xoayPhai(min(maxSpeed, 200));        break;
          case '0': if (!autoMode) dungXe();                            break;

          // --- Tháp pháo ---
          case '7': turretState = 1;  break;
          case '8': turretState = -1; break;
          case '9': turretState = 0;  break;

          // --- Nòng súng ---
          case 'C': gunState = 1;  break;
          case 'D': gunState = -1; break;
          case 'E': gunState = 0;  break;

          // --- Radar ---
          case 'A': radarActive = true;  break;
          case 'B': radarActive = false; break;

          // --- Chế độ tự động ---
          case 'G':
            autoMode = true;
            autoState = AUTO_DRIVE;
            nearCount = 0;
            clearCount = 0;
            lastValidDist = 999;
            lastRadarTime = 0;   // Quét ngay
            break;
          case 'H':
            tatTuDong();
            break;

          // --- 4 mức tốc độ ---
          case 'I': maxSpeed = SPEED_LEVELS[0]; break;
          case 'J': maxSpeed = SPEED_LEVELS[1]; break;
          case 'K': maxSpeed = SPEED_LEVELS[2]; break;
          case 'L': maxSpeed = SPEED_LEVELS[3]; break;

          default: break;
        }
      }
    }
};

void setup() {
  ESP32PWM::allocateTimer(0);
  gunServo.setPeriodHertz(50);
  gunServo.attach(SERVO_PIN, 500, 2400);
  gunServo.write(gunAngle);

  pwmKhoiTao();
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  dungXe();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  BLEDevice::init("Tank_ESP32_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
}

void loop() {
  capNhatThapPhao();
  capNhatNongSung();
  capNhatQuayDau();

  bool canGuiRadar = deviceConnected && radarActive;
  bool dangQuay = (autoMode && autoState == AUTO_TURNING);

  if ((canGuiRadar || autoMode) && !dangQuay && (millis() - lastRadarTime >= RADAR_PERIOD)) {
    lastRadarTime = millis();
    long kc = doKhoangCach();

    if (canGuiRadar) {
      char txString[12];
      snprintf(txString, sizeof(txString), "%ld", kc);
      pTxCharacteristic->setValue((uint8_t*)txString, strlen(txString));
      pTxCharacteristic->notify();
    }

    if (autoMode) xuLyTuDong(kc);
  }
}

// ---------- Chế độ tự động ----------
void tatTuDong() {
  autoMode = false;
  autoState = AUTO_DRIVE;
  nearCount = 0;
  clearCount = 0;
  lastValidDist = 999;
  dungXe();
}

void xuLyTuDong(long kc) {
  if (autoState == AUTO_DRIVE) {

    // Mất echo (999): nếu lần đo hợp lệ trước đó đã gần vật cản
    // thì coi như vật cản quá sát, KHÔNG coi là đường trống
    if (kc >= 999) {
      if (lastValidDist < SLOW_START) kc = 0;
    } else {
      lastValidDist = kc;
    }

    if (kc < TURN_DIST) {
      dungXe();                 // DỪNG NGAY ở lần đo đầu tiên
      nearCount++;
      if (nearCount >= 2) {     // xác nhận lần 2 mới quay (chống nhiễu)
        nearCount = 0;
        lastValidDist = 999;
        quayPhaiTaiCho(200);
        turnStartTime = millis();
        autoState = AUTO_TURNING;
      }
      return;
    }

    nearCount = 0;

    int tocDo;
    if (kc >= SLOW_START) {
      tocDo = maxSpeed;
    } else {
      int minS = min(MIN_SPEED, maxSpeed);
      tocDo = map(kc, TURN_DIST, SLOW_START, minS, maxSpeed);
      tocDo = constrain(tocDo, minS, maxSpeed);
    }
    chayTien(tocDo);

  } else if (autoState == AUTO_WAIT) {
    // Đứng yên, quét lại; cần 2 lần đo thoáng liên tiếp mới chạy tiếp
    if (millis() - waitStartTime < SETTLE_MS) return;

    if (kc >= TURN_DIST) {
      clearCount++;
      if (clearCount >= 2) {
        clearCount = 0;
        nearCount = 0;
        lastValidDist = (kc < 999) ? kc : 999;
        autoState = AUTO_DRIVE;
      }
    } else {
      clearCount = 0;   // còn vật cản -> tiếp tục đứng yên
    }
  }
}

void capNhatQuayDau() {
  if (autoMode && autoState == AUTO_TURNING) {
    if (millis() - turnStartTime >= TURN_180_MS) {
      dungXe();
      waitStartTime = millis();
      clearCount = 0;
      autoState = AUTO_WAIT;
    }
  }
}

// ---------- Nòng súng ----------
void capNhatNongSung() {
  if (gunState == 0) return;

  if (millis() - lastGunTime >= GUN_STEP_DELAY) {
    lastGunTime = millis();
    gunAngle += gunState;

    if (gunAngle >= GUN_MAX_ANGLE) {
      gunAngle = GUN_MAX_ANGLE;
      gunState = 0;
    } else if (gunAngle <= GUN_MIN_ANGLE) {
      gunAngle = GUN_MIN_ANGLE;
      gunState = 0;
    }
    gunServo.write(gunAngle);
  }
}

// ---------- Tháp pháo ----------
void capNhatThapPhao() {
  if (turretState == 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    return;
  }

  if (millis() - lastStepTime >= 3) {
    lastStepTime = millis();
    if (turretState == 1) {
      stepNumber++; if (stepNumber > 7) stepNumber = 0;
    } else if (turretState == -1) {
      stepNumber--; if (stepNumber < 0) stepNumber = 7;
    }
    xuatXungStep();
  }
}

void xuatXungStep() {
  digitalWrite(IN1, stepMatrix[stepNumber][0]);
  digitalWrite(IN2, stepMatrix[stepNumber][1]);
  digitalWrite(IN3, stepMatrix[stepNumber][2]);
  digitalWrite(IN4, stepMatrix[stepNumber][3]);
}

// ---------- Radar (trả về cm, 999 = không có echo) ----------
long doKhoangCach() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ---------- Bánh xích ----------
void chayTien(int tocDo) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  pwmGhi(PWMA, tocDo); pwmGhi(PWMB, tocDo);
}
void chayLui(int tocDo) {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  pwmGhi(PWMA, tocDo); pwmGhi(PWMB, tocDo);
}
void xoayTrai(int tocDo) {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  pwmGhi(PWMA, 0); pwmGhi(PWMB, tocDo);
}
void xoayPhai(int tocDo) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  pwmGhi(PWMA, tocDo); pwmGhi(PWMB, 0);
}
void quayPhaiTaiCho(int tocDo) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  pwmGhi(PWMA, tocDo); pwmGhi(PWMB, tocDo);
}
void dungXe() {
  pwmGhi(PWMA, 0); pwmGhi(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}