#include <SPI.h>
#include <Wire.h>
#include "mcp2515_can.h"

/* ===== [1] Library Compatibility Shims ===== */
#ifndef MCP_16MHZ
  #define MCP_16MHZ MCP_16MHz
#endif
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif

/* ===== [2] Hardware & Calibration Setup ===== */
const uint8_t PIN_CS = 9;
mcp2515_can CAN(PIN_CS);

// 엔코더가 없으므로 드라이버 명령치 기반 역산 상수를 그대로 활용합니다.
const float DEC_PER_RPM = 17896.14f;
const float INC_PER_DEG = (182.0f * 70.0f * 150.0f) / 62.0f;

// 소프트웨어 내부 위치 관리를 위한 누적 목표 각도 변수
float   g_currentCalculatedAngle = 0.0f;
uint8_t NODE_ID                 = 1;

/* ===== [3] CANopen SDO TX helpers ===== */
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val) {
  uint8_t d[8] = { 0x23,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF),  (uint8_t)((val >> 8)  & 0xFF),
                   (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF) };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val) {
  uint8_t d[8] = { 0x2B,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0 };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}
bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val) {
  uint8_t d[8] = { 0x2F,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   val, 0, 0, 0 };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}

// Read Statusword (0x6041) via SDO. Returns true on success.
bool SDO_read_statusword(uint16_t* out_sw) {
  uint8_t req[8] = { 0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0 };
  if (CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, req) != CAN_OK) return false;

  unsigned long t0 = millis();
  while ((millis() - t0) < 50) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned char len = 0;
      unsigned char buf[8];
      CAN.readMsgBuf(&len, buf);
      unsigned long id = CAN.getCanId();
      if ((id & 0x7FF) == (uint16_t)(0x580 + NODE_ID)) {
        if (buf[0] == 0x4B && buf[1] == 0x41 && buf[2] == 0x60) {
          *out_sw = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
          return true;
        }
      }
    }
  }
  return false;
}

void setProfileSpeedRpm(float rpm) {
  if (rpm < 0.1f) rpm = 0.1f;
  uint32_t decValue = (uint32_t)(rpm * DEC_PER_RPM);
  SDO_write_u32(0x6081, 0x00, decValue);
  Serial.print(F(">> Target speed set: ")); Serial.print(rpm);
  Serial.print(F(" RPM (DEC: ")); Serial.print(decValue); Serial.println(F(")"));
}

/* ===== [4] Settle detection - drive's Target_Reached flag ===== */
bool waitForTargetReached(uint16_t timeout_ms = 8000) {
  Serial.print(F(" -> moving"));
  uint16_t sw;

  // Phase 1: wait until bit10 clears (motion has started)
  unsigned long t0 = millis();
  while ((millis() - t0) < 300) {
    if (SDO_read_statusword(&sw)) {
      if (!(sw & (1 << 10))) break; 
    }
    delay(10);
  }

  // Phase 2: wait for bit10 to set again (target reached)
  t0 = millis();
  uint8_t dot_div = 0;
  while ((millis() - t0) < timeout_ms) {
    if (SDO_read_statusword(&sw)) {
      if (sw & (1 << 10)) { 
        Serial.println(F(" [reached]"));
        return true;
      }
    }
    if (++dot_div >= 5) { dot_div = 0; Serial.print('.'); }
    delay(20);
  }
  Serial.println(F(" [timeout]"));
  return false;
}

void printMenu() {
  Serial.println(F("\n======= [ Kinco FD1X5 Controller ] ======="));
  Serial.println(F("  en      : Enable motor (default 1000 RPM)"));
  Serial.println(F("  spd <n> : Set RPM speed (calibrated)"));
  Serial.println(F("  abs <n> : Move to absolute angle (deg)"));
  Serial.println(F("  z       : Zero current software position"));
  Serial.println(F("  h       : Show this help"));
  Serial.println(F("=========================================="));
}

/* ===== [5] Main ===== */
void setup() {
  Serial.begin(115200);
  Wire.begin(); // 타 장치 디바이스 구동 유지를 위해 Wire는 남겨둡니다.

  if (CAN.begin(CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("CAN Init Failed!"));
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  printMenu();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "en") {
      Serial.println(F(">> Enable sequence (default 1000 RPM)..."));
      SDO_write_u16(0x6040, 0x00, 0x0086); delay(50);
      SDO_write_u8 (0x6060, 0x00, 0x01);   delay(50);
      setProfileSpeedRpm(1000.0f);         delay(50);
      SDO_write_u16(0x6040, 0x00, 0x0006); delay(50);
      SDO_write_u16(0x6040, 0x00, 0x0007); delay(50);
      SDO_write_u16(0x6040, 0x00, 0x103F); delay(50);
      Serial.println(F(">> Motor online (1000 RPM)"));
    }
    else if (cmd.startsWith("spd ")) {
      float rpm = cmd.substring(4).toFloat();
      setProfileSpeedRpm(rpm);
    }
    else if (cmd.startsWith("abs ")) {
      float target = cmd.substring(4).toFloat();
      
      // 입력받은 타겟 각도를 드라이버 펄스로 변환하여 전송
      long targetPulse = (long)(target * INC_PER_DEG);
      SDO_write_u32(0x607A, 0x00, targetPulse);
      
      // 모터 물리 구동 완료 대기
      waitForTargetReached();

      // 하드웨어 엔코더 피드백 대신, 타겟 펄스 기반 역산으로 현재 위치 결정
      g_currentCalculatedAngle = (float)targetPulse / INC_PER_DEG;
      
      Serial.print(F(">> Move complete. Target Pulse: ")); Serial.print(targetPulse);
      Serial.print(F(" | Current angle: "));
      Serial.print(g_currentCalculatedAngle, 2); Serial.println(F(" deg"));
    }
    else if (cmd == "z") {
      // 소프트웨어 기준점 영점 초기화
      g_currentCalculatedAngle = 0.0f;
      Serial.println(F(">> Zero command (Software reference cleared)"));
    }
    else if (cmd == "h") {
      printMenu();
    }
  }
}
