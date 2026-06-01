/**
 * Mock iCar Pro (IOS-Vlink) BLE OBD2 Adapter
 * 
 * ESP32 기반 가짜 OBD2 어댑터
 * 실제 iCar Pro와 동일한 BLE 특성으로 게이지 디바이스 테스트용
 * 
 * BLE 설정:
 * - Device Name: IOS-Vlink
 * - Service UUID: 18F0
 * - RX (Notify): 2AF0
 * - TX (Write): 2AF1
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs (iCar Pro 호환)
#define SERVICE_UUID        "000018F0-0000-1000-8000-00805F9B34FB"
#define CHAR_RX_UUID        "00002AF0-0000-1000-8000-00805F9B34FB"  // Notify (Mock → Gauge)
#define CHAR_TX_UUID        "00002AF1-0000-1000-8000-00805F9B34FB"  // Write (Gauge → Mock)

BLEServer* pServer = nullptr;
BLECharacteristic* pCharRx = nullptr;  // Notify characteristic
BLECharacteristic* pCharTx = nullptr;  // Write characteristic
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Forward declarations
String processCommand(String cmd);
String processATCommand(String cmd);
String processOBD2Command(String cmd);
String processMode01(String pid);
String processMode09(String pid);
void sendResponse(String response);

// 시뮬레이션 데이터
struct OBD2Data {
  uint16_t rpm = 850;           // RPM (0-8000)
  uint8_t speed = 0;            // km/h (0-255)
  uint8_t coolantTemp = 85;     // °C (-40 ~ 215, raw 0-255)
  uint8_t throttle = 15;        // % (0-100)
  uint16_t engineLoad = 20;     // % (0-100)
  uint8_t fuelLevel = 75;       // % (0-100)
  uint16_t intakeTemp = 25;     // °C
  float voltage = 12.6;         // 배터리 전압
  uint8_t mapPressure = 101;    // MAP kPa (대기압 101, 부스트 시 150+)
  int8_t timingAdvance = 15;    // 점화시기 ° BTDC (-64 ~ +63.5)
} simData;

// 시뮬레이션 모드
enum SimMode {
  SIM_IDLE,       // 공회전
  SIM_DRIVING,    // 주행 중
  SIM_SPORT,      // 스포츠 주행
  SIM_OVERHEAT    // 과열 시나리오
};
SimMode currentMode = SIM_IDLE;

// Simulated fuel octane (q/w/e). Drives the knock-retard model: under load the ECU pulls
// timing back more for lower octane. This is what the gauge's fuel-grade detector reads.
enum FuelOctane { OCT_LOW, OCT_REG, OCT_PREM };
FuelOctane fuelOctane = OCT_REG;
unsigned long lastSimUpdate = 0;

// ELM327 상태
bool echoOn = true;
bool headersOn = false;
bool spacesOn = true;
String currentProtocol = "6";  // ISO 15765-4 CAN

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("✅ 게이지 디바이스 연결됨!");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("❌ 연결 해제됨");
  }
};

class TxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    value.trim();
    
    if (value.length() > 0) {
      Serial.print("📥 RX: ");
      Serial.println(value);
      
      String response = processCommand(value);
      if (response.length() > 0) {
        sendResponse(response);
      }
    }
  }
};

void sendResponse(String response) {
  // ELM327 형식: 응답 + ">" 프롬프트
  response += "\r>";
  
  Serial.print("📤 TX: ");
  Serial.println(response);
  
  // BLE로 전송
  pCharRx->setValue(response.c_str());
  pCharRx->notify();
}

String processCommand(String cmd) {
  cmd.toUpperCase();
  cmd.trim();
  
  // AT 명령어 처리
  if (cmd.startsWith("AT")) {
    return processATCommand(cmd);
  }
  
  // OBD2 PID 처리
  return processOBD2Command(cmd);
}

String processATCommand(String cmd) {
  // ATZ - 리셋
  if (cmd == "ATZ" || cmd == "AT Z") {
    echoOn = true;
    headersOn = false;
    spacesOn = true;
    return "ELM327 v1.5";
  }
  
  // ATE0/ATE1 - 에코 끄기/켜기
  if (cmd == "ATE0" || cmd == "AT E0") {
    echoOn = false;
    return "OK";
  }
  if (cmd == "ATE1" || cmd == "AT E1") {
    echoOn = true;
    return "OK";
  }
  
  // ATH0/ATH1 - 헤더 끄기/켜기
  if (cmd == "ATH0" || cmd == "AT H0") {
    headersOn = false;
    return "OK";
  }
  if (cmd == "ATH1" || cmd == "AT H1") {
    headersOn = true;
    return "OK";
  }
  
  // ATS0/ATS1 - 공백 끄기/켜기
  if (cmd == "ATS0" || cmd == "AT S0") {
    spacesOn = false;
    return "OK";
  }
  if (cmd == "ATS1" || cmd == "AT S1") {
    spacesOn = true;
    return "OK";
  }
  
  // ATSP - 프로토콜 설정
  if (cmd.startsWith("ATSP") || cmd.startsWith("AT SP")) {
    currentProtocol = cmd.substring(cmd.length() - 1);
    return "OK";
  }
  
  // ATI - 디바이스 정보
  if (cmd == "ATI" || cmd == "AT I") {
    return "ELM327 v1.5";
  }
  
  // ATRV - 배터리 전압
  if (cmd == "ATRV" || cmd == "AT RV") {
    char buf[10];
    sprintf(buf, "%.1fV", simData.voltage);
    return String(buf);
  }
  
  // ATDP - 현재 프로토콜
  if (cmd == "ATDP" || cmd == "AT DP") {
    return "ISO 15765-4 (CAN 11/500)";
  }
  
  // AT@1 - 디바이스 설명
  if (cmd == "AT@1") {
    return "Mock iCar Pro";
  }
  
  // 기타 AT 명령어는 OK 반환
  return "OK";
}

String processOBD2Command(String cmd) {
  // Mode 01 - 현재 데이터
  if (cmd.startsWith("01")) {
    String pid = cmd.substring(2, 4);
    return processMode01(pid);
  }
  
  // Mode 09 - 차량 정보
  if (cmd.startsWith("09")) {
    String pid = cmd.substring(2, 4);
    return processMode09(pid);
  }
  
  // 지원하지 않는 명령
  return "NO DATA";
}

String processMode01(String pid) {
  char response[32];
  String sep = spacesOn ? " " : "";
  
  // PID 00 - 지원 PID (01-20)
  if (pid == "00") {
    // 비트마스크: 01,04,05,0B,0C,0D,0E,0F,11,2F 지원
    // Bit positions: 01=bit31, 04=bit28, 05=bit27, 0B=bit21, 0C=bit20, 0D=bit19, 0E=bit18, 0F=bit17, 11=bit15, 20=bit0
    sprintf(response, "41%s00%s98%s1E%s80%s13", 
            sep.c_str(), sep.c_str(), sep.c_str(), sep.c_str(), sep.c_str());
    return String(response);
  }
  
  // PID 01 - MIL 상태 (체크엔진)
  if (pid == "01") {
    sprintf(response, "41%s01%s00%s00%s00%s00",
            sep.c_str(), sep.c_str(), sep.c_str(), sep.c_str(), sep.c_str());
    return String(response);
  }
  
  // PID 04 - 엔진 부하 (%)
  if (pid == "04") {
    uint8_t load = (simData.engineLoad * 255) / 100;
    sprintf(response, "41%s04%s%02X", sep.c_str(), sep.c_str(), load);
    return String(response);
  }
  
  // PID 05 - 냉각수 온도 (°C)
  if (pid == "05") {
    uint8_t temp = simData.coolantTemp + 40;  // offset 40
    sprintf(response, "41%s05%s%02X", sep.c_str(), sep.c_str(), temp);
    return String(response);
  }
  
  // PID 0B - MAP 흡기 매니폴드 압력 (kPa)
  if (pid == "0B") {
    sprintf(response, "41%s0B%s%02X", sep.c_str(), sep.c_str(), simData.mapPressure);
    return String(response);
  }
  
  // PID 0C - RPM
  if (pid == "0C") {
    uint16_t rpmVal = simData.rpm * 4;  // RPM = ((A*256)+B)/4
    sprintf(response, "41%s0C%s%02X%s%02X", 
            sep.c_str(), sep.c_str(), (rpmVal >> 8) & 0xFF, sep.c_str(), rpmVal & 0xFF);
    return String(response);
  }
  
  // PID 0D - 차량 속도 (km/h)
  if (pid == "0D") {
    sprintf(response, "41%s0D%s%02X", sep.c_str(), sep.c_str(), simData.speed);
    return String(response);
  }
  
  // PID 0E - 점화 시기 (° BTDC)
  if (pid == "0E") {
    // 공식: A/2 - 64, 역산: A = (timing + 64) * 2
    uint8_t timingRaw = (simData.timingAdvance + 64) * 2;
    sprintf(response, "41%s0E%s%02X", sep.c_str(), sep.c_str(), timingRaw);
    return String(response);
  }
  
  // PID 0F - 흡기 온도 (°C)
  if (pid == "0F") {
    uint8_t temp = simData.intakeTemp + 40;
    sprintf(response, "41%s0F%s%02X", sep.c_str(), sep.c_str(), temp);
    return String(response);
  }
  
  // PID 11 - 스로틀 위치 (%)
  if (pid == "11") {
    uint8_t throttle = (simData.throttle * 255) / 100;
    sprintf(response, "41%s11%s%02X", sep.c_str(), sep.c_str(), throttle);
    return String(response);
  }
  
  // PID 2F - 연료 레벨 (%)
  if (pid == "2F") {
    uint8_t fuel = (simData.fuelLevel * 255) / 100;
    sprintf(response, "41%s2F%s%02X", sep.c_str(), sep.c_str(), fuel);
    return String(response);
  }
  
  return "NO DATA";
}

String processMode09(String pid) {
  // PID 02 - VIN
  if (pid == "02") {
    return "49 02 01 4D 4F 43 4B 56 49 4E 31 32 33 34 35 36 37";  // MOCKVIN1234567
  }
  
  return "NO DATA";
}

void updateSimulation() {
  if (millis() - lastSimUpdate < 100) return;  // 100ms 주기
  lastSimUpdate = millis();
  
  switch (currentMode) {
    case SIM_IDLE:
      // 공회전: RPM 750-900 랜덤, 속도 0
      simData.rpm = 750 + random(150);
      simData.speed = 0;
      simData.throttle = 10 + random(5);
      simData.engineLoad = 15 + random(10);
      simData.mapPressure = 35 + random(10);    // 진공 상태 (낮은 MAP)
      simData.timingAdvance = 10 + random(5);   // 낮은 점화시기
      break;
      
    case SIM_DRIVING: {
      // 동적 주행: 속도 0~180 km/h 부드러운 사인 스윕 (주기 ~24s) + 미세 지터
      float t = millis() / 1000.0f;
      float sweep = 90.0f + 90.0f * sinf(t * 0.262f);          // 0..180
      simData.speed = (uint8_t)constrain((int)(sweep + random(-2, 3)), 0, 180);
      simData.rpm = constrain(800 + simData.speed * 28 + (int)random(150), 800, 7000);
      simData.throttle = map(simData.speed, 0, 180, 10, 90);
      simData.engineLoad = map(simData.speed, 0, 180, 20, 85);
      simData.mapPressure = 40 + simData.speed / 2;             // 부하에 따라 상승
      simData.timingAdvance = 15 + simData.speed / 15;
      break;
    }
      
    case SIM_SPORT:
      // 스포츠 주행: RPM 4000-7000, 속도 100-180
      simData.rpm = 4000 + random(3000);
      simData.speed = 100 + random(80);
      simData.throttle = 70 + random(30);
      simData.engineLoad = 70 + random(30);
      simData.mapPressure = 140 + random(60);   // 부스트! (대기압 초과)
      simData.timingAdvance = 25 + random(10);  // 높은 점화시기
      break;
      
    case SIM_OVERHEAT:
      // 과열: 수온 상승
      simData.rpm = 2000 + random(500);
      simData.speed = 30 + random(20);
      if (simData.coolantTemp < 120) {
        simData.coolantTemp++;
      }
      simData.mapPressure = 60 + random(20);
      break;
  }

  // --- knock-retard fuel model (overrides per-mode timing): at low load all fuels stay
  //     advanced; under high MAP/load the ECU pulls timing back, more for lower octane.
  //     PREM barely retards, LOW yanks hard -> this is the fuel-grade signal.
  float load = (simData.mapPressure - 90) / 110.0f; if (load < 0) load = 0;  // 0 below ~90kPa
  int kFactor = (fuelOctane==OCT_PREM) ? 3 : (fuelOctane==OCT_REG) ? 18 : 32;
  simData.timingAdvance = constrain(30 - (int)(load * kFactor), -10, 40);
}

void printStatus() {
  Serial.println("\n=== Mock OBD2 상태 ===");
  Serial.printf("모드: %s\n", 
    currentMode == SIM_IDLE ? "공회전" :
    currentMode == SIM_DRIVING ? "주행" :
    currentMode == SIM_SPORT ? "스포츠" : "과열");
  Serial.printf("RPM: %d\n", simData.rpm);
  Serial.printf("속도: %d km/h\n", simData.speed);
  Serial.printf("수온: %d°C\n", simData.coolantTemp);
  Serial.printf("MAP: %d kPa (%+.2f bar)\n", simData.mapPressure, (simData.mapPressure - 101) / 100.0);
  Serial.printf("점화시기: %d° BTDC\n", simData.timingAdvance);
  Serial.printf("연료옥탄: %s\n", fuelOctane==OCT_PREM?"고급(PREM)":fuelOctane==OCT_REG?"일반(REG)":"저급(LOW)");
  Serial.printf("스로틀: %d%%\n", simData.throttle);
  Serial.printf("연료: %d%%\n", simData.fuelLevel);
  Serial.printf("전압: %.1fV\n", simData.voltage);
  Serial.println("====================\n");
}

void handleSerialCommand() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1':
        currentMode = SIM_IDLE;
        Serial.println("🚗 모드: 공회전");
        break;
      case '2':
        currentMode = SIM_DRIVING;
        Serial.println("🚗 모드: 일반 주행");
        break;
      case '3':
        currentMode = SIM_SPORT;
        Serial.println("🚗 모드: 스포츠 주행");
        break;
      case '4':
        currentMode = SIM_OVERHEAT;
        simData.coolantTemp = 90;
        Serial.println("🚗 모드: 과열 시나리오");
        break;
      case 'q':
        fuelOctane = OCT_LOW;
        Serial.println("⛽ 연료: 저급 (LOW octane)");
        break;
      case 'w':
        fuelOctane = OCT_REG;
        Serial.println("⛽ 연료: 일반 (REGULAR)");
        break;
      case 'e':
        fuelOctane = OCT_PREM;
        Serial.println("⛽ 연료: 고급 (PREMIUM)");
        break;
      case 's':
        printStatus();
        break;
      case 'r':
        simData.coolantTemp = 85;
        Serial.println("♻️ 수온 리셋");
        break;
      case 'h':
        Serial.println("\n=== 명령어 ===");
        Serial.println("1: 공회전 모드");
        Serial.println("2: 일반 주행 모드");
        Serial.println("3: 스포츠 주행 모드");
        Serial.println("4: 과열 시나리오");
        Serial.println("q/w/e: 연료 저급/일반/고급");
        Serial.println("s: 상태 출력");
        Serial.println("r: 수온 리셋");
        Serial.println("h: 도움말");
        Serial.println("==============\n");
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n🚗 Mock iCar Pro 시작...");
  
  // BLE 초기화
  BLEDevice::init("IOS-Vlink");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  // 서비스 생성
  BLEService* pService = pServer->createService(SERVICE_UUID);
  
  // RX Characteristic (Notify - Mock → Gauge)
  pCharRx = pService->createCharacteristic(
    CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharRx->addDescriptor(new BLE2902());
  
  // TX Characteristic (Write - Gauge → Mock)
  pCharTx = pService->createCharacteristic(
    CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCharTx->setCallbacks(new TxCallbacks());
  
  // 서비스 시작
  pService->start();
  
  // 광고 시작
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ BLE 광고 시작: IOS-Vlink");
  Serial.println("📱 게이지 디바이스 연결 대기 중...");
  Serial.println("\n'h' 입력하면 명령어 도움말 표시\n");
}

void loop() {
  // 시리얼 명령 처리
  handleSerialCommand();
  
  // 시뮬레이션 업데이트
  updateSimulation();
  
  // 연결 상태 변경 처리
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("📢 광고 재시작");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(10);
}
