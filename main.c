#include <SPI.h>

#include "mcp_can.h"
 
// ===================== 사용자 설정 =====================

const uint8_t  NODE_ID          = 1;

const bool     MCP2515_16MHZ    = true;

const uint8_t  PIN_CS           = 9;

const uint8_t  PIN_INT          = 2;

const uint8_t  PIN_POT          = A0;   // (미사용)
 
// ===================== 내부 상수/ID =====================

MCP_CAN CAN(PIN_CS);
 
// CANopen COB-IDs

uint32_t COB_NMT    = 0x000;

uint32_t COB_SDO_TX = 0x600 + NODE_ID;

uint32_t COB_SDO_RX = 0x580 + NODE_ID;
 
// ===================== 유틸 함수 =====================

bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8) {

  byte st = CAN.sendMsgBuf(id, 0 /*std*/, len, (unsigned char*)data);

  return (st == CAN_OK);

}
 
// getCanId() 사용 버전

bool waitSDO(uint16_t index, uint8_t sub, uint32_t timeout_ms = 200) {

  uint32_t t0 = millis();

  while (millis() - t0 < timeout_ms) {

    if (digitalRead(PIN_INT) == LOW) {

      unsigned char len = 0, buf[8] = {0};

      if (CAN.readMsgBuf(&len, buf) == CAN_OK) {

        unsigned long rxId = CAN.getCanId();

        if (rxId == COB_SDO_RX && len >= 8) {

          uint16_t idx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);

          if (idx == index && buf[3] == sub) {

            if (buf[0] == 0x60) return true;  // success

            if (buf[0] == 0x80) return false; // abort

          }

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
 
// ===================== 제어 유틸 =====================

bool enable_drive_profile_position() {

  // 모드 설정: Profile Position

  if (!SDO_write_u8(0x6060, 0x00, 1)) return false;
 
  // CiA-402 Enable 시퀀스: Shutdown(0x06) -> Switch On(0x07) -> Enable Operation(0x0F)

  if (!SDO_write_u16(0x6040, 0x00, 0x0006)) return false; // Shutdown

  if (!SDO_write_u16(0x6040, 0x00, 0x0007)) return false; // Switch On

  if (!SDO_write_u16(0x6040, 0x00, 0x000F)) return false; // Enable Operation

  return true;

}
 
void quickstop_and_shutdown() {

  // Quick Stop -> Shutdown (제조사별 동작 차 있음. 일반적인 안전 정지 패턴)

  SDO_write_u16(0x6040, 0x00, 0x000B); // Quick Stop

  delay(10);

  SDO_write_u16(0x6040, 0x00, 0x0006); // Shutdown

}
 
// Profile Position move

bool move_pp(int32_t target, bool absolute = true) {

  // 0x607A Target Position

  if (!SDO_write_u32(0x607A, 0x00, (uint32_t)target)) return false;
 
  // 0x6040 Controlword: new set-point 트리거

  // absolute/relative 선택은 드라이브별로 상이. 여기선 예시로:

  //  - ABS: 0x001F (bit4: set-point, bit3: enable operation 유지 가정)

  //  - REL: 0x005F (bit5: change immediately 가정)

  uint16_t cw = absolute ? 0x001F : 0x005F;

  if (!SDO_write_u16(0x6040, 0x00, cw)) return false;

  // set-point 토글 클리어

  cw &= ~(1 << 4);

  SDO_write_u16(0x6040, 0x00, cw);

  return true;

}
 
// ===================== 커맨드 파서/상태 =====================

enum RunState { IDLE, RUNNING };

RunState state = IDLE;
 
String readLine() {

  static String buf;

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {

      String out = buf;

      buf = "";

      return out;

    }

    buf += c;

  }

  return String();

}
 
void handleCommand(const String& line) {

  String cmd = line;

  cmd.trim();

  cmd.toUpperCase();
 
  if (cmd == "S") {

    Serial.println(F("[CMD] START"));

    if (enable_drive_profile_position()) {

      state = RUNNING;

      Serial.println(F("Drive ENABLED (PP). Ready for targets."));

    } else {

      Serial.println(F("Enable FAILED"));

    }

    return;

  }
 
  if (cmd == "STOP") {

    Serial.println(F("[CMD] STOP -> QuickStop+Shutdown"));

    quickstop_and_shutdown();

    state = IDLE;

    return;

  }
 
  // 숫자만 -> ABS 이동

  bool allDigitOrSign = cmd.length() > 0;

  for (size_t i = 0; i < cmd.length(); ++i) {

    char ch = cmd[i];

    if (!(isDigit(ch) || ch == '-' || ch == '+')) { allDigitOrSign = false; break; }

  }

  if (allDigitOrSign) {

    long target = cmd.toInt();

    if (state != RUNNING) { Serial.println(F("Not RUNNING. Send 'S' first.")); return; }

    if (move_pp(target, /*absolute=*/true)) {

      Serial.print(F("ABS Move OK -> ")); Serial.println(target);

    } else {

      Serial.println(F("ABS Move FAIL"));

    }

    return;

  }
 
  if (cmd.startsWith("ABS ")) {

    if (state != RUNNING) { Serial.println(F("Not RUNNING. Send 'S' first.")); return; }

    long target = cmd.substring(4).toInt();

    if (move_pp(target, /*absolute=*/true)) {

      Serial.print(F("ABS Move OK -> ")); Serial.println(target);

    } else {

      Serial.println(F("ABS Move FAIL"));

    }

    return;

  }
 
  if (cmd.startsWith("REL ")) {

    if (state != RUNNING) { Serial.println(F("Not RUNNING. Send 'S' first.")); return; }

    long delta = cmd.substring(4).toInt();

    if (move_pp(delta, /*absolute=*/false)) {

      Serial.print(F("REL Move OK -> ")); Serial.println(delta);

    } else {

      Serial.println(F("REL Move FAIL"));

    }

    return;

  }
 
  Serial.print(F("Unknown CMD: ")); Serial.println(line);

}
 
// ===================== 메인 =====================

void setup() {

  Serial.begin(115200);

  pinMode(PIN_INT, INPUT);
 
  // MCP2515 초기화: 250 kbps (주석/상수 일치)

  byte ret;

  if (MCP2515_16MHZ) ret = CAN.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ);

  else               ret = CAN.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ);

  if (ret != CAN_OK) { Serial.println(F("CAN init failed")); while(1){} }

  CAN.setMode(MCP_NORMAL);

  Serial.println(F("CAN init OK"));
 
  delay(50);
 
  // NMT Start

  NMT_start(NODE_ID);

  delay(20);
 
  Serial.println(F("READY. Type 'S' to start, 'STOP' to halt, number/ABS/REL to move."));

}
 
void loop() {

  // 커맨드 처리 (동기식)

  String line = readLine();

  if (line.length()) {

    handleCommand(line);

  }
 
  // 필요 시 상태 폴링/피드백 추가 가능

}

 
