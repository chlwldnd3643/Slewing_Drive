#include <SPI.h>
#include "mcp2515_can.h"
#include <SoftwareSerial.h>

/* ===== Compat ===== */
#ifndef MCP_16MHZ
  #ifdef MCP_16MHz
    #define MCP_16MHZ MCP_16MHz
  #endif
#endif
#ifndef MCP_16MHz
  #ifdef MCP_16MHZ
    #define MCP_16MHZ MCP_16MHZ
  #endif
#endif
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif

/* ===== User Config ===== */
uint8_t  NODE_ID   = 1;   // 드라이브 노드 ID
const uint8_t PIN_CS  = 9;
const uint8_t PIN_INT = 2;

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

// PDO 포지션 범위 (핸들 -1000~+1000 → -MAX_PDO_POS~+MAX_PDO_POS)
const long MAX_PDO_POS = 2500000L;   // ±2,500,000 inc

// 포지션 속도 (position_speed = 0x6081) 기본값
long g_positionSpeed = 1000L;        // 드라이브 내부 단위 그대로 사용

mcp2515_can CAN(PIN_CS);

/* ===== 핸들 수신 (별도 아두이노에서 D8으로 전송) =====
   포맷 예: "S0\n", "S-500\n", "S1000\n"
   연결: 핸들보드 TX(D8) → 이 보드 D8(RX)
*/
const uint8_t LINK_RX_PIN = 8;
const uint8_t LINK_TX_PIN = 9;   // 사용 안 해도 됨 (더미)
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN); // (RX, TX)

long steer_cmd = 0;          // -1000 ~ +1000
unsigned long lastSteerUpdate = 0;
String steerLine = "";

/* ===== 제어 모드 ===== */
enum ControlMode {
  MODE_SDO_CMD    = 0,   // SDO: ABS/REL/SPD 만 사용
  MODE_PDO_HANDLE = 1,   // 핸들 → PDO 포지션
  MODE_BOTH_DEBUG = 2    // 둘 다 활성 (디버그)
};

ControlMode controlMode = MODE_SDO_CMD;

/* ===== SDO용 상태 ===== */
long g_targetPos = 0;     // 우리가 관리하는 SDO 기준 target position

/* ===== 공통 CAN/SDO 유틸 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8) {
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}

bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val) {
  uint8_t d[8] = {
    0x2F,
    (uint8_t)(idx & 0xFF),
    (uint8_t)(idx >> 8),
    sub,
    val, 0, 0, 0
  };
  return sendCAN(0x600 + NODE_ID, d);
}

bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val) {
  uint8_t d[8] = {
    0x2B,
    (uint8_t)(idx & 0xFF),
    (uint8_t)(idx >> 8),
    sub,
    (uint8_t)(val & 0xFF),
    (uint8_t)(val >> 8),
    0, 0
  };
  return sendCAN(0x600 + NODE_ID, d);
}

bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val) {
  uint8_t d[8] = {
    0x23,
    (uint8_t)(idx & 0xFF),
    (uint8_t)(idx >> 8),
    sub,
    (uint8_t)(val & 0xFF),
    (uint8_t)((val >> 8) & 0xFF),
    (uint8_t)((val >> 16) & 0xFF),
    (uint8_t)((val >> 24) & 0xFF)
  };
  return sendCAN(0x600 + NODE_ID, d);
}

bool SDO_write_i32(uint16_t idx, uint8_t sub, int32_t val) {
  return SDO_write_u32(idx, sub, (uint32_t)val);
}

/* ===== PDO: Target Position 송신 (RPDO1 = 0x200 + NODE_ID, 607A 매핑 가정) ===== */
bool PDO_write_position(long pos) {
  uint8_t d[8];
  d[0] = (uint8_t)( pos        & 0xFF);
  d[1] = (uint8_t)((pos >> 8 ) & 0xFF);
  d[2] = (uint8_t)((pos >> 16) & 0xFF);
  d[3] = (uint8_t)((pos >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;
  return sendCAN(0x200 + NODE_ID, d);
}

/* ===== position_speed 설정 (0x6081) ===== */
void set_position_speed(long spd) {
  if (spd < 0) spd = -spd;
  if (spd == 0) spd = 1;   // 0 방지

  g_positionSpeed = spd;
  SDO_write_u32(0x6081, 0x00, (uint32_t)g_positionSpeed);

  Serial.print(F("[SPD] position_speed(0x6081) = "));
  Serial.println(g_positionSpeed);
}

/* ===== 드라이브 Enable (Position Mode + 103F) ===== */
bool enable_drive() {
  bool ok = true;

  // 1) Fault reset (0x86)
  ok &= SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // 2) Mode of operation = 1 (Profile Position Mode)
  ok &= SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 3) position_speed 설정 (0x6081)
  set_position_speed(g_positionSpeed);
  delay(20);

  // 4) Controlword = 0x103F (Target position 바뀔 때마다 즉시 절대 위치로 이동)
  ok &= SDO_write_u16(0x6040, 0x00, 0x103F);
  delay(20);

  return ok;
}

/* ===== SDO 포지션 명령 (ABS / REL) ===== */
void sdo_move_abs(long pos) {
  g_targetPos = pos;
  SDO_write_i32(0x607A, 0x00, g_targetPos);   // 103F 상태에서는 이거 바꾸면 바로 이동
  delay(5);

  Serial.print(F("[SDO ABS] targetPos = "));
  Serial.println(g_targetPos);
}

void sdo_move_rel(long delta) {
  g_targetPos += delta;
  sdo_move_abs(g_targetPos);
}

/* ===== 시리얼 명령 처리 =====
   명령:
     en          : enable_drive()
     rst         : 0x6040 = 0x0086
     abs 3000    : 절대 위치 3000 inc
     rel 2000    : 상대 +2000 inc
     spd 1000    : position_speed(0x6081) = 1000
     1           : MODE_SDO_CMD
     2           : MODE_PDO_HANDLE
     3           : MODE_BOTH_DEBUG
     h / help    : 도움말
*/
String serialLine = "";

void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (serialLine.length() == 0) return;

      String line = serialLine;
      serialLine = "";

      line.trim();
      line.toLowerCase();

      // 모드 전환
      if (line == "1") {
        controlMode = MODE_SDO_CMD;
        Serial.println(F("[MODE] SDO_CMD (ABS/REL/SPD)"));
        return;
      }
      if (line == "2") {
        controlMode = MODE_PDO_HANDLE;
        Serial.println(F("[MODE] PDO_HANDLE (steering → position PDO)"));
        return;
      }
      if (line == "3") {
        controlMode = MODE_BOTH_DEBUG;
        Serial.println(F("[MODE] BOTH (SDO + PDO)"));
        return;
      }
      if (line == "h" || line == "help") {
        Serial.println(F("=== HELP ==="));
        Serial.println(F(" en          : enable (pos mode + 6081 + 0x103F)"));
        Serial.println(F(" rst         : 0x6040 = 0x0086 (fault reset)"));
        Serial.println(F(" abs <pos>   : absolute position (inc) via SDO"));
        Serial.println(F(" rel <delta> : relative move (inc) via SDO"));
        Serial.println(F(" spd <val>   : position_speed(0x6081) = val"));
        Serial.println(F(" 1           : MODE_SDO_CMD"));
        Serial.println(F(" 2           : MODE_PDO_HANDLE"));
        Serial.println(F(" 3           : MODE_BOTH_DEBUG"));
        return;
      }

      // enable / reset
      if (line == "en") {
        bool ok = enable_drive();
        Serial.println(ok ? F("[EN] OK") : F("[EN] FAIL"));
        return;
      }
      if (line == "rst") {
        bool ok = SDO_write_u16(0x6040, 0x00, 0x0086);
        Serial.println(ok ? F("[RST] 0x86 OK") : F("[RST] FAIL"));
        return;
      }

      // spd
      if (line.startsWith("spd")) {
        String s = line.substring(3);
        s.trim();
        long v = s.toInt();
        set_position_speed(v);
        return;
      }

      // abs / rel (SDO 모드 또는 BOTH일 때 의미 있음)
      if (controlMode == MODE_SDO_CMD || controlMode == MODE_BOTH_DEBUG) {
        if (line.startsWith("abs")) {
          String s = line.substring(3);
          s.trim();
          long p = s.toInt();
          sdo_move_abs(p);
          return;
        }
        if (line.startsWith("rel")) {
          String s = line.substring(3);
          s.trim();
          long d = s.toInt();
          sdo_move_rel(d);
          return;
        }
      }

      Serial.print(F("[WARN] Unknown cmd: "));
      Serial.println(line);
      return;
    } else {
      if (serialLine.length() < 64) {
        serialLine += c;
      } else {
        serialLine = "";
      }
    }
  }
}

/* ===== 핸들 명령 수신 (S-500, S0, S1000 등) ===== */
void readSteerCommand() {
  while (linkSerial.available() > 0) {
    char c = linkSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (steerLine.length() == 0) return;

      if (steerLine[0] == 'S') {
        long val = steerLine.substring(1).toInt();  // "S-123456" → -123456

        // 혹시라도 범위를 넘는 값이 오면 한 번 더 클램프
        if (val >  2500000L) val =  2500000L;
        if (val < -2500000L) val = -2500000L;

        steer_cmd = val;
        lastSteerUpdate = millis();
      }
      steerLine = "";
    } else {
      if (steerLine.length() < 20) {
        steerLine += c;
      } else {
        steerLine = "";
      }
    }
  }
}


/* ===== Setup / Loop ===== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);   // SPI용 SS (일반적으로 LOW 유지)

  linkSerial.begin(57600);   // 핸들 아두이노와 동일 속도

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK) {
    Serial.println(F("CAN init failed"));
    while (1) {}
  }
  CAN.setMode(MCP_NORMAL);

  Serial.println(F("=== Kinco FD1x5 Slewing Controller ==="));
  Serial.println(F(" 먼저 'en' 한 번 쳐서 pos mode + 6081 + 0x103F 상태 만들어 놓고:"));
  Serial.println(F("  - SDO 제어: abs/rel/spd"));
  Serial.println(F("  - PDO 제어: MODE 2 선택 후 핸들 돌리기"));
  Serial.println(F(" 명령: en, rst, abs, rel, spd, 1/2/3, h/help"));

  // 시작은 SDO 모드로
  controlMode = MODE_SDO_CMD;
}

void loop() {
  // 1) 시리얼 명령 처리 (en, abs, rel, spd, 1/2/3 등)
  processSerial();

  // 2) PDO 핸들 모드일 때만 핸들 값 사용
  if (controlMode == MODE_PDO_HANDLE || controlMode == MODE_BOTH_DEBUG) {

    // 핸들 문자열 읽기
    readSteerCommand();

    // 1초 이상 새 값이 없으면 0으로 (안전)
    if (millis() - lastSteerUpdate > 1000) {
      steer_cmd = 0;
    }

    long pos = steer_cmd;    // 이미 -2,500,000 ~ +2,500,000으로 스케일된 값

    // 바로 PDO로 Target Position 전송
    PDO_write_position(pos);

    // 디버그 출력
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print("[PDO] steer_cmd(pos)=");
      Serial.println(pos);
      lastPrint = millis();
    }
  }

  delay(10);
}
