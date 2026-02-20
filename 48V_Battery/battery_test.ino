#include <SPI.h>
#include "mcp2515_can.h"
#include <avr/wdt.h>

#define SPI_CS_PIN 9

// CAN interface
mcp2515_can CAN(SPI_CS_PIN);

// CAN message IDs
static const unsigned long DALY_REQ_ID = 0x18100140;  // Host -> BMS (ext)
static const unsigned long DALY_RSP_ID = 0x18104001;  // BMS  -> Host (ext)


bool systemStarted = false;
unsigned long lastSend = 0;
const uint8_t ID[4] = {0x90, 0x92, 0x93, 0x98};
int idx = 0;


void setup() {
  Serial.begin(115200);
  waitForStart();

  initCAN();

  Serial.println("CAN BUS init OK");
  delay(100);

}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
 
    if (command == "stop") {
      stop_control();
    }
  }

  if (millis() - lastSend > 1000) {
    requestCAN(ID[idx]);
    lastSend = millis();
  }
  receiveCAN(ID[idx]);
  idx = (idx + 1) % 4;

  delay(90);
}





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
    delay(100);
  }
}


void stop_control() {
  Serial.println("System resetting...");
  wdt_enable(WDTO_250MS);
  while (true) {}
}

void initCAN() {
  while (CAN.begin(CAN_250KBPS) != CAN_OK) {
    Serial.println("CAN Init Fail");
    delay(100);
  }
  Serial.println("CAN Init OK");
  delay(100);
}


void receiveCAN(uint8_t dataId) {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t buf[8];
    CAN.readMsgBuf(&len, buf);           // 데이터 먼저 읽고
    unsigned long id = CAN.getCanId();   // 그 다음 ID 확인

    if (id == DALY_RSP_ID) {
      Serial.println("1");
      switch (dataId) {
        case 0x18904001:
          parse_DALY_0x90(buf, len);  // 총전압/전류/SOC
          break;
        case 0x18924001:
          parse_DALY_0x92(buf, len);  // 셀 온도
          break;
        case 0x18934001:
          parse_DALY_0x93(buf, len);  // 충방전 상태, MOS 상태
          break;
        case 0x18984001:
          parse_DALY_0x98(buf, len);  // 배터리 오류 상태
          break;
        default:
          Serial.println("알 수 없는 Data ID 응답");
          break;
      }
    }
  }
}

void parse_DALY_0x90(const uint8_t* buf, uint8_t len) {
  float totalV_acc = ((buf[0] << 8) | buf[1]) * 0.1;
  float totalV_real = ((buf[2] << 8) | buf[3]) * 0.1;
  float current = (((buf[4] << 8) | buf[5]) - 30000) * 0.1;
  float soc = ((buf[6] << 8) | buf[7]) * 0.1;

  Serial.print("TotalV(acc): "); Serial.print(totalV_acc); Serial.print(" V  ");
  Serial.print("TotalV(real): "); Serial.print(totalV_real); Serial.print(" V  ");
  Serial.print("Current: "); Serial.print(current); Serial.print(" A  ");
  Serial.print("SOC: "); Serial.print(soc); Serial.println(" %");
}

void parse_DALY_0x92(const uint8_t* buf, uint8_t len) {
  int8_t t_max = buf[0] - 40;
  uint8_t t_max_id = buf[1];
  int8_t t_min = buf[2] - 40;
  uint8_t t_min_id = buf[3];

  Serial.print("[0x92] Tmax="); Serial.print(t_max); Serial.print("°C (#");
  Serial.print(t_max_id); Serial.print(")  Tmin=");
  Serial.print(t_min); Serial.print("°C (#");
  Serial.print(t_min_id); Serial.println(")");
}

void parse_DALY_0x93(const uint8_t* buf, uint8_t len) {
  uint8_t cd_state = buf[0];
  bool chgMOS = buf[1] != 0;
  bool dsgMOS = buf[2] != 0;
  uint8_t bms_life = buf[3]; // 0~255

  const char* stateStr = (cd_state == 1) ? "충전" : (cd_state == 2) ? "방전" : "정지";

  Serial.print("[0x93] 상태="); Serial.print(stateStr);
  Serial.print("  CHG_MOS="); Serial.print(chgMOS ? "ON" : "OFF");
  Serial.print("  DSG_MOS="); Serial.print(dsgMOS ? "ON" : "OFF");
  Serial.print("  Life="); Serial.print(bms_life);
}



void printFlag(const char* name) {
  Serial.print(name); Serial.print(" | ");
}

void parse_DALY_0x98(const uint8_t* buf, uint8_t len) {
  uint8_t b0 = buf[0];
  if (b0 & (1<<0)) printFlag("셀OV Lv1");
  if (b0 & (1<<1)) printFlag("셀OV Lv2");
  if (b0 & (1<<2)) printFlag("셀UV Lv1");
  if (b0 & (1<<3)) printFlag("셀UV Lv2");
  if (b0 & (1<<4)) printFlag("총전압 High Lv1");
  if (b0 & (1<<5)) printFlag("총전압 High Lv2");
  if (b0 & (1<<6)) printFlag("총전압 Low Lv1");
  if (b0 & (1<<7)) printFlag("총전압 Low Lv2");

  uint8_t b1 = buf[1];
  if (b1 & (1<<0)) printFlag("충전온도 High Lv1");
  if (b1 & (1<<1)) printFlag("충전온도 High Lv2");
  if (b1 & (1<<2)) printFlag("충전온도 Low Lv1");
  if (b1 & (1<<3)) printFlag("충전온도 Low Lv2");
  if (b1 & (1<<4)) printFlag("방전온도 High Lv1");
  if (b1 & (1<<5)) printFlag("방전온도 High Lv2");
  if (b1 & (1<<6)) printFlag("방전온도 Low Lv1");
  if (b1 & (1<<7)) printFlag("방전온도 Low Lv2"); 

  uint8_t b2 = buf[2];
  if (b2 & (1<<0)) printFlag("충전 과전류 Lv1");
  if (b2 & (1<<1)) printFlag("충전 과전류 Lv2");
  if (b2 & (1<<2)) printFlag("방전 과전류 Lv1");
  if (b2 & (1<<3)) printFlag("방전 과전류 Lv2");
  if (b2 & (1<<4)) printFlag("SOC High Lv1");
  if (b2 & (1<<5)) printFlag("SOC High Lv2");
  if (b2 & (1<<6)) printFlag("SOC Low Lv1");
  if (b2 & (1<<7)) printFlag("SOC Low Lv2"); 

  uint8_t b3 = buf[3];
  if (b3 & (1<<0)) printFlag("압력차 Lv1");
  if (b3 & (1<<1)) printFlag("압력차 Lv2");
  if (b3 & (1<<2)) printFlag("온도차 Lv1");
  if (b3 & (1<<3)) printFlag("온도차 Lv2"); 

  uint8_t b4 = buf[4];
  if (b4 & (1<<0)) printFlag("충전MOS 과온");
  if (b4 & (1<<1)) printFlag("방전MOS 과온");
  if (b4 & (1<<2)) printFlag("충전MOS 센서 오류");
  if (b4 & (1<<3)) printFlag("방전MOS 센서 오류");
  if (b4 & (1<<4)) printFlag("충전MOS 점착");
  if (b4 & (1<<5)) printFlag("방전MOS 점착");
  if (b4 & (1<<6)) printFlag("충전MOS 단선");
  if (b4 & (1<<7)) printFlag("방전MOS 단선"); 

  uint8_t b5 = buf[5];
  if (b5 & (1<<0)) printFlag("AFE 오류");
  if (b5 & (1<<1)) printFlag("셀 수집 드롭");
  if (b5 & (1<<2)) printFlag("온도센서 오류");
  if (b5 & (1<<3)) printFlag("EEPROM 오류");
  if (b5 & (1<<4)) printFlag("RTC 오류");
  if (b5 & (1<<5)) printFlag("프리차지 실패");
  if (b5 & (1<<6)) printFlag("차량통신 오류");
  if (b5 & (1<<7)) printFlag("내부망 통신 오류"); 

  uint8_t b6 = buf[6];
  if (b6 & (1<<0)) printFlag("전류모듈 오류");
  if (b6 & (1<<1)) printFlag("내총전압 모듈 오류");
  if (b6 & (1<<2)) printFlag("단락 보호 오류");
  if (b6 & (1<<3)) printFlag("저전압 충전금지");
  if (b6 & (1<<4)) printFlag("GPS/소프트스위치 MOS OFF");

  uint8_t errCode = buf[7];
  Serial.print("ERR_CODE="); Serial.println(errCode);
}

void requestCAN(uint8_t dataId) {
  uint8_t requestData[8] = {dataId,0,0,0,0,0,0,0};

  CAN.sendMsgBuf(DALY_REQ_ID, 1, 8, requestData);
}



