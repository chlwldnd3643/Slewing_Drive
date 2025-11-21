#include <SPI.h>
#include "mcp2515_can.h"
#include <SoftwareSerial.h>

/* ===== MCP2515 / CAN 설정 ===== */
#ifndef MCP_16MHZ
  #ifdef MCP_16MHz
    #define MCP_16MHZ MCP_16MHz
  #endif
#endif
#ifndef MCP_16MHz
  #ifdef MCP_16MHZ
    #define MCP_16MHz MCP_16MHZ
  #endif
#endif
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif

// ---- 유저 설정 ----
uint8_t  NODE_ID    = 1;     // Kinco 드라이브 노드 ID
const uint8_t PIN_CS  = 9;   // MCP2515 CS
const uint8_t PIN_INT = 2;   // MCP2515 INT

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

mcp2515_can CAN(PIN_CS);

/* ===== 핸들 보드와의 UART 링크 =====
   핸들 아두이노에서:
     long cmd = (long)(norm * 2500000.0f);
     linkSerial.print('S');
     linkSerial.println(cmd);
   이런 식으로 "S-123456\n" / "S2500000\n" 형태로 보낸다고 가정.
*/
const uint8_t LINK_RX_PIN = 8;  // 핸들 TX → 이쪽 RX
const uint8_t LINK_TX_PIN = 7;  // 안 써도 되는 더미 TX
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);

// 핸들에서 받은 최종 타겟 포지션 값 (-2,500,000 ~ +2,500,000)
long steer_cmd = 0;
String steerLine = "";

/* ===== 모드 설정 ===== */
enum ControlMode {
  MODE_SDO_CMD    = 0,   // SDO ABS/REL/SPD
  MODE_HANDLE     = 1,   // 핸들값으로 실시간 위치제어 (PDO 또는 SDO 스트리밍)
  MODE_BOTH_DEBUG = 2    // 둘 다 활성
};
ControlMode controlMode = MODE_SDO_CMD;

// 핸들 모드에서 진짜 PDO 쓸지, 아니면 SDO 스트리밍으로 갈지 선택
// true : RPDO1→607A 매핑 + PDO 사용
// false: PDO 안 믿고 그냥 SDO로 607A를 계속 써줌 (무조건 동작)
const bool USE_PDO_FOR_HANDLE = false;

/* ===== SDO용 상태 ===== */
long g_targetPos     = 0;      // 우리가 기억하는 SDO 명령 기준 타겟 위치
uint32_t g_profSpeed = 1000;   // profile_speed(0x6081)에 쓸 값 (Kinco 내부 단위)

/* ===== 공통 SDO 유틸 ===== */
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

/* ===== PDO : 타겟 포지션 (607A) 송신 =====
   가정: RPDO1(0x1400 / 0x1600)이 607A:00, 32bit로 매핑되어 있고
        COB-ID = 0x200 + NODE_ID
*/
bool PDO_write_target_position(long pos) {
  uint8_t d[8];
  d[0] = (uint8_t)( pos        & 0xFF);
  d[1] = (uint8_t)((pos >> 8 ) & 0xFF);
  d[2] = (uint8_t)((pos >> 16) & 0xFF);
  d[3] = (uint8_t)((pos >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;
  return sendCAN(0x200 + NODE_ID, d);
}

/* ===== profile_speed(0x6081) 설정 =====
   단위는 Kinco 내부 단위(매뉴얼 공식 참고). 여기서는 그냥 "속도 크기" 느낌으로 사용하고,
   실제 감은 실험으로 잡는 방식.
*/
void set_profile_speed(uint32_t spd) {
  if (spd == 0) spd = 1;
  g_profSpeed = spd;
  bool ok = SDO_write_u32(0x6081, 0x00, g_profSpeed);

  Serial.print(F("[SPD] profile_speed(0x6081) = "));
  Serial.print(g_profSpeed);
  Serial.println(ok ? F(" (OK)") : F(" (FAIL)"));
}

/* ===== (선택) RPDO1 → 607A 매핑 =====
   실제 PDO를 쓰고 싶으면 이 함수를 쓴다.
   - 1400:01 = COB-ID (0x200+ID)
   - 1600:01 = 0x607A0020 (index 0x607A, sub 0, 32bit)
*/
void configure_RPDO1_for_target_position() {
  // 1) RPDO1 disable (COB-ID 상위 bit = 1)
  //    예시로 0x80000200 + NODE_ID (벤더에 따라 다를 수 있음)
  SDO_write_u32(0x1400, 0x01, 0x80000200UL + NODE_ID);
  delay(10);

  // 2) 매핑 지우기 (0개)
  SDO_write_u8(0x1600, 0x00, 0);
  delay(10);

  // 3) 607A:00, 32bit로 매핑
  SDO_write_u32(0x1600, 0x01, 0x607A0020UL);
  delay(10);

  // 4) 매핑 개수 = 1
  SDO_write_u8(0x1600, 0x00, 1);
  delay(10);

  // 5) RPDO1 enable : COB-ID = 0x200 + NODE_ID
  SDO_write_u32(0x1400, 0x01, 0x200 + NODE_ID);
  delay(10);

  Serial.println(F("[RPDO] RPDO1 mapped to 0x607A:00, COB-ID=0x200+ID"));
}

/* ===== 드라이브 Enable (Position Mode + 0x103F) ===== */
bool enable_drive_position_mode() {
  bool ok = true;

  // 1) Fault reset (0x86)
  ok &= SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // 2) Mode of operation = 1 (Profile Position Mode)
  ok &= SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 3) profile_speed (0x6081)
  set_profile_speed(g_profSpeed);
  delay(20);

  // 4) Controlword = 0x103F
  ok &= SDO_write_u16(0x6040, 0x00, 0x103F);
  delay(20);

  return ok;
}

/* ===== SDO 포지션 명령 (ABS / REL) ===== */
void sdo_move_abs(long pos) {
  g_targetPos = pos;
  SDO_write_i32(0x607A, 0x00, g_targetPos);  // 103F 상태에서 607A 변경 → 바로 이동
  delay(5);

  Serial.print(F("[SDO ABS] targetPos = "));
  Serial.println(g_targetPos);
}

void sdo_move_rel(long delta) {
  g_targetPos += delta;
  sdo_move_abs(g_targetPos);
}

/* ===== 시리얼 명령 처리 =====
   en          : enable (pos mode + profile_speed + 103F)
   rst         : 0x6040 = 0x0086
   abs <pos>   : 절대 위치 (inc)
   rel <delta> : 상대 이동 (inc)
   spd <val>   : 6081 profile_speed 값 설정
   1           : MODE_SDO_CMD
   2           : MODE_HANDLE
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
        Serial.println(F("[MODE] SDO CMD (ABS/REL/SPD)"));
        return;
      }
      if (line == "2") {
        controlMode = MODE_HANDLE;
        Serial.println(F("[MODE] HANDLE (steering → target position)"));
        return;
      }
      if (line == "3") {
        controlMode = MODE_BOTH_DEBUG;
        Serial.println(F("[MODE] BOTH (SDO + HANDLE)"));
        return;
      }
      if (line == "h" || line == "help") {
        Serial.println(F("=== HELP ==="));
        Serial.println(F(" en          : enable (pos mode + 6081 + 0x103F)"));
        Serial.println(F(" rst         : 0x6040 = 0x0086 (fault reset)"));
        Serial.println(F(" abs <pos>   : absolute position via SDO (inc)"));
        Serial.println(F(" rel <delta> : relative move via SDO (inc)"));
        Serial.println(F(" spd <val>   : profile_speed(0x6081) set"));
        Serial.println(F(" 1           : MODE_SDO_CMD"));
        Serial.println(F(" 2           : MODE_HANDLE"));
        Serial.println(F(" 3           : MODE_BOTH_DEBUG"));
        return;
      }

      // enable / reset
      if (line == "en") {
        bool ok = enable_drive_position_mode();
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
        uint32_t v = (uint32_t)s.toInt();
        set_profile_speed(v);
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

/* ===== 핸들 아두이노에서 오는 S<long>\n 파싱 ===== */
void readSteerCommand() {
  while (linkSerial.available() > 0) {
    char c = linkSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (steerLine.length() == 0) return;

      if (steerLine[0] == 'S') {
        long val = steerLine.substring(1).toInt();  // "S-123456" → -123456

        // 혹시라도 이상값이면 한 번 더 클램프
        if (val >  2500000L) val =  2500000L;
        if (val < -2500000L) val = -2500000L;

        steer_cmd = val;
      }
      steerLine = "";
    } else {
      if (steerLine.length() < 24) {
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
  pinMode(10, OUTPUT);   // SPI SS (MCP2515용)

  linkSerial.begin(57600);   // 핸들 아두이노와 동일 속도

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK) {
    Serial.println(F("CAN init failed"));
    while (1) {}
  }
  CAN.setMode(MCP_NORMAL);

  Serial.println(F("=== Kinco FD1x5 Slewing Controller (Final) ==="));
  Serial.println(F(" 1) 전원 후 'en' → pos mode + 6081 + 0x103F"));
  Serial.println(F(" 2) MODE 1: abs/rel/spd (SDO)"));
  Serial.println(F(" 3) MODE 2: 핸들 → target position (-2.5M~+2.5M)"));
  Serial.println(F(" 명령: en, rst, abs, rel, spd, 1/2/3, h/help"));

  if (USE_PDO_FOR_HANDLE) {
    configure_RPDO1_for_target_position();
  }

  controlMode = MODE_SDO_CMD;
}

void loop() {
  // 1) PC 시리얼 명령
  processSerial();

  // 2) 핸들 모드일 때: 핸들값 반영
  if (controlMode == MODE_HANDLE || controlMode == MODE_BOTH_DEBUG) {
    readSteerCommand();

    long pos = steer_cmd;   // 이미 -2,500,000 ~ +2,500,000 범위의 최종 위치값

    if (USE_PDO_FOR_HANDLE) {
      // ---- 정석 PDO 방식 ----
      PDO_write_target_position(pos);
    } else {
      // ---- 확실하게 움직이게 하는 SDO 스트리밍 방식 ----
      SDO_write_i32(0x607A, 0x00, pos);
      // 필요시 아래 한 줄 켜서 새 세트포인트 토글 강화도 가능
      // SDO_write_u16(0x6040, 0x00, 0x103F);
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print(F("[HANDLE] targetPos="));
      Serial.println(pos);
      lastPrint = millis();
    }
  }

  delay(10);
}
