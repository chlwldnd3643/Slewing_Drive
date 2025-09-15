#include <SPI.h>
#include "mcp_can.h"

// ===================== 사용자 설정 =====================
const uint8_t  NODE_ID          = 1;        
const bool     MCP2515_16MHZ    = true;     
const uint32_t CAN_BAUD_KBPS    = 500000;   
const uint8_t  PIN_CS           = 9;       
const uint8_t  PIN_INT          = 2;        
const uint8_t  PIN_POT          = A0;       

// ===================== 내부 상수/ID =====================
MCP_CAN CAN(PIN_CS);

// CANopen COB-IDs
uint32_t COB_NMT    = 0x000;
uint32_t COB_SDO_TX = 0x600 + NODE_ID;  
uint32_t COB_SDO_RX = 0x580 + NODE_ID;  
// uint32_t COB_RPDO1  = 0x200 + NODE_ID;  // (주석 처리)
// uint32_t COB_TPDO1  = 0x180 + NODE_ID;  // (주석 처리)

// ===================== 유틸 함수 =====================
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8) {
  byte st = CAN.sendMsgBuf(id, 0 /*std*/, len, (unsigned char*)data);
  return (st == CAN_OK);
}

bool waitSDO(uint16_t index, uint8_t sub, uint32_t timeout_ms = 100) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (digitalRead(PIN_INT) == LOW) {
      long unsigned int rxId; unsigned char len=0, buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);
      if (rxId == COB_SDO_RX && len >= 8) {
        uint16_t idx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        if (idx == index && buf[3] == sub) {
          if (buf[0] == 0x60) return true; // success
          if (buf[0] == 0x80) return false; // abort
        }
      }
    }
  }
  return false; // timeout
}

// ---- SDO write helpers ----
bool SDO_write_u8 (uint16_t idx, uint8_t sub, uint8_t  val) {
  uint8_t d[8] = { 0x2F, (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub, val, 0,0,0 };
  if (!sendCAN(COB_SDO_TX, d)) return false;
  return waitSDO(idx, sub);
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val) {
  uint8_t d[8] = { 0x2B, (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0,0 };
  if (!sendCAN(COB_SDO_TX, d)) return false;
  return waitSDO(idx, sub);
}
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val) {
  uint8_t d[8] = { 0x23, (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF),
                   (uint8_t)((val >>16) & 0xFF), (uint8_t)((val >>24) & 0xFF) };
  if (!sendCAN(COB_SDO_TX, d)) return false;
  return waitSDO(idx, sub);
}

// ---- NMT ----
void NMT_start(uint8_t node) {
  uint8_t d[2] = { 0x01, node }; 
  sendCAN(COB_NMT, d, 2);
}

// ===================== 메인 =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT);

  // MCP2515 초기화
  byte ret;
  if (MCP2515_16MHZ) ret = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  else               ret = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  if (ret != CAN_OK) { Serial.println(F("CAN init failed")); while(1){} }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("CAN init OK"));

  delay(50);

  // NMT Start
  NMT_start(NODE_ID);
  delay(20);

  // ========== 여기서부터는 전부 SDO로만 세팅/제어 ==========
  // 예: 모드 Profile Position (0x6060=1)
  if (SDO_write_u8(0x6060, 0x00, 1)) {
    Serial.println(F("Mode set OK (Profile Position)"));
  } else {
    Serial.println(F("Mode set FAIL"));
  }

  // Enable 시퀀스 (0x6040=0x06 → 0x0F)
  if (SDO_write_u16(0x6040, 0x00, 0x0006)) Serial.println(F("Shutdown OK"));
  if (SDO_write_u16(0x6040, 0x00, 0x000F)) Serial.println(F("SwitchOn+Enable OK"));
}

void loop() {
  // 예시: 2초마다 절대 위치 명령 보내기 (SDO로만)
  static uint32_t lastMs=0; static bool flip=false;
  if (millis() - lastMs > 2000) {
    int32_t target = flip ? +100000 : -100000;
    // 타깃 포지션(0x607A)
    if (SDO_write_u32(0x607A, 0x00, target)) {
      // Controlword = New Set-point (bit4=1 토글)
      uint16_t cw = 0x003F; 
      if (SDO_write_u16(0x6040, 0x00, cw)) {
        Serial.print(F("Move OK, target=")); Serial.println(target);
      }
    } else {
      Serial.println(F("Move FAIL"));
    }
    flip = !flip;
    lastMs = millis();
  }
}
