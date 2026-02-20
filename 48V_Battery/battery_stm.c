#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

// =====================
// HW Config (EDIT THIS)
// =====================
// Nucleo에서 CS 핀만 원하는 GPIO로 지정하면 됩니다.
// 예: D10, D9 등
#define SPI_CS_PIN 9

// MCP2515 CAN
mcp2515_can CAN(SPI_CS_PIN);

// =====================
// DALY CAN IDs (Extended)
// =====================
static const unsigned long DALY_REQ_ID = 0x18100140;  // Host -> BMS (ext)
static const unsigned long DALY_RSP_ID = 0x18104001;  // BMS  -> Host (ext)

// =====================
// App State
// =====================
bool systemStarted = false;
unsigned long lastSend = 0;
const uint8_t ID_LIST[4] = {0x90, 0x92, 0x93, 0x98};
int idx = 0;

// =====================
// Forward Decls
// =====================
void waitForStart();
void stop_control();
void initCAN();

void requestCAN(uint8_t dataId);
void receiveCAN(uint8_t expectedDataId);

void parse_DALY_0x90(const uint8_t* buf, uint8_t len);
void parse_DALY_0x92(const uint8_t* buf, uint8_t len);
void parse_DALY_0x93(const uint8_t* buf, uint8_t len);
void parse_DALY_0x98(const uint8_t* buf, uint8_t len);
void printFlag(const char* name);

// =====================
// Setup / Loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  waitForStart();

  SPI.begin();      // STM32에서는 명시적으로 해주는 편이 안전
  initCAN();

  Serial.println("CAN BUS init OK");
  delay(100);
}

void loop() {
  // 간단 명령 처리: "stop" 또는 "s"
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    if (command == "stop" || command == "s") {
      stop_control();
    }
  }

  // 1초마다 요청
  if (millis() - lastSend > 1000) {
    requestCAN(ID_LIST[idx]);
    lastSend = millis();
  }

  // 수신/파싱
  receiveCAN(ID_LIST[idx]);

  idx = (idx + 1) % 4;
  delay(90);
}

// =====================
// Functions
// =====================
void waitForStart() {
  Serial.println("Press 'S' to start the system.");
  while (!systemStarted) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'S' || c == 's') {
        systemStarted = true;
        Serial.println("System started.");
      }
    }
    delay(50);
  }
}

void stop_control() {
  Serial.println("System resetting...");
  delay(50);

  // STM32 리셋 (Watchdog 대신)
  NVIC_SystemReset();

  // 여긴 도달하지 않음
  while (true) {}
}

void initCAN() {
  // MCP2515 init loop
  while (CAN.begin(CAN_250KBPS) != CAN_OK) {
    Serial.println("CAN Init Fail");
    delay(100);
  }
  Serial.println("CAN Init OK");
  delay(100);
}

void requestCAN(uint8_t dataId) {
  uint8_t requestData[8] = {dataId, 0, 0, 0, 0, 0, 0, 0};

  // sendMsgBuf(id, ext, len, data)
  // ext=1 => Extended ID
  byte rc = CAN.sendMsgBuf(DALY_REQ_ID, 1, 8, requestData);
  if (rc != CAN_OK) {
    Serial.print("CAN send fail, rc=");
    Serial.println(rc);
  }
}

void receiveCAN(uint8_t expectedDataId) {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t buf[8];

    CAN.readMsgBuf(&len, buf);
    unsigned long id = CAN.getCanId();

    if (id == DALY_RSP_ID) {
      // DALY 응답은 같은 ID로 오므로,
      // 어떤 요청(0x90/0x92/0x93/0x98)을 보냈는지에 따라 파싱
      switch (expectedDataId) {
        case 0x90:
          parse_DALY_0x90(buf, len);  // 총전압/전류/SOC
          break;
        case 0x92:
          parse_DALY_0x92(buf, len);  // 셀 온도
          break;
        case 0x93:
          parse_DALY_0x93(buf, len);  // 충방전 상태, MOS 상태
          break;
        case 0x98:
          parse_DALY_0x98(buf, len);  // 배터리 오류 상태
          break;
        default:
          Serial.println("Unknown expectedDataId");
          break;
      }
    }
  }
}

// =====================
// Parsers
// =====================
void parse_DALY_0x90(const uint8_t* buf, uint8_t len) {
  if (len < 8) return;

  float totalV_acc  = ((buf[0] << 8) | buf[1]) * 0.1f;
  float totalV_real = ((buf[2] << 8) | buf[3]) * 0.1f;
  float current     = (((buf[4] << 8) | buf[5]) - 30000) * 0.1f;
  float soc         = ((buf[6] << 8) | buf[7]) * 0.1f;

  Serial.print("TotalV(acc): ");  Serial.print(totalV_acc);  Serial.print(" V  ");
  Serial.print("TotalV(real): "); Serial.print(totalV_real); Serial.print(" V  ");
  Serial.print("Current: ");      Serial.print(current);     Serial.print(" A  ");
  Serial.print("SOC: ");          Serial.print(soc);         Serial.println(" %");
}

void parse_DALY_0x92(const uint8_t* buf, uint8_t len) {
  if (len < 4) return;

  int8_t  t_max    = (int8_t)buf[0] - 40;
  uint8_t t_max_id = buf[1];
  int8_t  t_min    = (int8_t)buf[2] - 40;
  uint8_t t_min_id = buf[3];

  Serial.print("[0x92] Tmax="); Serial.print(t_max); Serial.print("C (#");
  Serial.print(t_max_id);       Serial.print(")  Tmin=");
  Serial.print(t_min);          Serial.print("C (#");
  Serial.print(t_min_id);       Serial.println(")");
}

void parse_DALY_0x93(const uint8_t* buf, uint8_t len) {
  if (len < 4) return;

  uint8_t cd_state = buf[0];
  bool chgMOS      = buf[1] != 0;
  bool dsgMOS      = buf[2] != 0;
  uint8_t bms_life = buf[3];

  const char* stateStr = (cd_state == 1) ? "CHG" : (cd_state == 2) ? "DSG" : "IDLE";

  Serial.print("[0x93] STATE=");   Serial.print(stateStr);
  Serial.print("  CHG_MOS=");      Serial.print(chgMOS ? "ON" : "OFF");
  Serial.print("  DSG_MOS=");      Serial.print(dsgMOS ? "ON" : "OFF");
  Serial.print("  Life=");         Serial.println(bms_life);
}

void printFlag(const char* name) {
  Serial.print(name);
  Serial.print(" | ");
}

void parse_DALY_0x98(const uint8_t* buf, uint8_t len) {
  if (len < 8) return;

  uint8_t b0 = buf[0];
  if (b0 & (1<<0)) printFlag("Cell OV Lv1");
  if (b0 & (1<<1)) printFlag("Cell OV Lv2");
  if (b0 & (1<<2)) printFlag("Cell UV Lv1");
  if (b0 & (1<<3)) printFlag("Cell UV Lv2");
  if (b0 & (1<<4)) printFlag("Pack V High Lv1");
  if (b0 & (1<<5)) printFlag("Pack V High Lv2");
  if (b0 & (1<<6)) printFlag("Pack V Low Lv1");
  if (b0 & (1<<7)) printFlag("Pack V Low Lv2");

  uint8_t b1 = buf[1];
  if (b1 & (1<<0)) printFlag("Chg Temp High Lv1");
  if (b1 & (1<<1)) printFlag("Chg Temp High Lv2");
  if (b1 & (1<<2)) printFlag("Chg Temp Low Lv1");
  if (b1 & (1<<3)) printFlag("Chg Temp Low Lv2");
  if (b1 & (1<<4)) printFlag("Dsg Temp High Lv1");
  if (b1 & (1<<5)) printFlag("Dsg Temp High Lv2");
  if (b1 & (1<<6)) printFlag("Dsg Temp Low Lv1");
  if (b1 & (1<<7)) printFlag("Dsg Temp Low Lv2");

  uint8_t b2 = buf[2];
  if (b2 & (1<<0)) printFlag("Chg OC Lv1");
  if (b2 & (1<<1)) printFlag("Chg OC Lv2");
  if (b2 & (1<<2)) printFlag("Dsg OC Lv1");
  if (b2 & (1<<3)) printFlag("Dsg OC Lv2");
  if (b2 & (1<<4)) printFlag("SOC High Lv1");
  if (b2 & (1<<5)) printFlag("SOC High Lv2");
  if (b2 & (1<<6)) printFlag("SOC Low Lv1");
  if (b2 & (1<<7)) printFlag("SOC Low Lv2");

  uint8_t b3 = buf[3];
  if (b3 & (1<<0)) printFlag("Delta P Lv1");
  if (b3 & (1<<1)) printFlag("Delta P Lv2");
  if (b3 & (1<<2)) printFlag("Delta T Lv1");
  if (b3 & (1<<3)) printFlag("Delta T Lv2");

  uint8_t b4 = buf[4];
  if (b4 & (1<<0)) printFlag("Chg MOS OT");
  if (b4 & (1<<1)) printFlag("Dsg MOS OT");
  if (b4 & (1<<2)) printFlag("Chg MOS Sensor Err");
  if (b4 & (1<<3)) printFlag("Dsg MOS Sensor Err");
  if (b4 & (1<<4)) printFlag("Chg MOS Stuck");
  if (b4 & (1<<5)) printFlag("Dsg MOS Stuck");
  if (b4 & (1<<6)) printFlag("Chg MOS Open");
  if (b4 & (1<<7)) printFlag("Dsg MOS Open");

  uint8_t b5 = buf[5];
  if (b5 & (1<<0)) printFlag("AFE Err");
  if (b5 & (1<<1)) printFlag("Cell Sample Drop");
  if (b5 & (1<<2)) printFlag("Temp Sensor Err");
  if (b5 & (1<<3)) printFlag("EEPROM Err");
  if (b5 & (1<<4)) printFlag("RTC Err");
  if (b5 & (1<<5)) printFlag("Precharge Fail");
  if (b5 & (1<<6)) printFlag("Vehicle Comm Err");
  if (b5 & (1<<7)) printFlag("Internal Comm Err");

  uint8_t b6 = buf[6];
  if (b6 & (1<<0)) printFlag("Current Module Err");
  if (b6 & (1<<1)) printFlag("Pack V Module Err");
  if (b6 & (1<<2)) printFlag("Short Protect Err");
  if (b6 & (1<<3)) printFlag("UV Charge Forbidden");
  if (b6 & (1<<4)) printFlag("GPS/SoftSW MOS OFF");

  uint8_t errCode = buf[7];
  Serial.print("ERR_CODE=");
  Serial.println(errCode);
}
