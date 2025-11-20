#include <SPI.h>
#include "mcp2515_can.h"
#include <SoftwareSerial.h>

/* ===== Compat (라이브러리 상수 호환) ===== */
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
uint8_t  NODE_ID   = 1;   // FD1X5 Node ID
const uint8_t PIN_CS  = 9;   // MCP2515 CS
const uint8_t PIN_INT = 2;   // MCP2515 INT

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

// 핸들 → PDO 모드에서 쓸 최대 위치 (중앙 기준 ±MAX_POS)
long MAX_POS = 200000L;      // 기어비/엔코더에 맞게 조정

// Profile Position mode에서 쓸 Target speed (Profile velocity, obj 0x6081)
// 기본값 1000
long targetSpeed = 1000L;

mcp2515_can CAN(PIN_CS);

/* ===== 핸들 명령 수신용 SoftwareSerial =====
   실제 배선: 핸들 보드 D8 → 이 보드 D8
*/
const uint8_t LINK_RX_PIN = 8;  // RX
const uint8_t LINK_TX_PIN = 9;  // 더미 TX (안 써도 됨)
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN); // (RX, TX)

// 최신 핸들 명령 값(–1000 ~ +1000)
int16_t steer_cmd = 0;
unsigned long lastSteerUpdate = 0;

// 시리얼 및 핸들 수신 버퍼
String serialLine = "";
String steerLine  = "";

/* ===== 제어 모드 ===== */
enum ControlMode {
  MODE_SDO_CMD    = 0,   // ABS/REL/SPD SDO 명령 모드
  MODE_PDO_HANDLE = 1,   // 핸들 → PDO 실시간 위치
  MODE_BOTH_DEBUG = 2
};

ControlMode controlMode = MODE_SDO_CMD; // 기본: SDO 명령 모드

// 우리가 기억하는 "현재 target position" (절대값)
long currentPos = 0;

/* ===== 공통 CAN / SDO 함수 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8) {
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}

/* SDO write 8bit */
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

/* SDO write 16bit */
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

/* SDO write 32bit (unsigned) */
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

/* SDO write 32bit (signed) */
bool SDO_write_i32(uint16_t idx, uint8_t sub, int32_t val) {
  return SDO_write_u32(idx, sub, (uint32_t)val);
}

/* ===== Target Speed(0x6081) 적용 ===== */
bool apply_target_speed() {
  return SDO_write_u32(0x6081, 0x00, (uint32_t)targetSpeed);
}

/* ===== PDO: Target Position 송신 =====
   RPDO1: COB-ID = 0x200 + NODE_ID 에
   4바이트 Target Position(0x607A)이 매핑되어 있다고 가정
*/
bool PDO_write_position(long pos) {
  uint8_t d[8];
  d[0] = (uint8_t)( pos        & 0xFF);
  d[1] = (uint8_t)((pos >> 8 ) & 0xFF);
  d[2] = (uint8_t)((pos >> 16) & 0xFF);
  d[3] = (uint8_t)((pos >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;
  return sendCAN(0x200 + NODE_ID, d); // RPDO1 기본 COB-ID
}

/* ===== 드라이브 Enable (Profile Position Mode + 0x103F) ===== */
bool enable_drive() {
  bool ok = true;

  // 1) Fault reset (0x86)
  ok &= SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // 2) Mode of operation = 1 (Profile Position Mode)
  ok &= SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 3) Target Speed(Profile velocity, 0x6081) 설정
  ok &= apply_target_speed();
  delay(20);

  // 4) Controlword = 0x103F로 세팅
  //    (이 상태에서 607A만 바꿔주면 바로 절대 위치로 움직이는 동작을 네가 직접 확인한 값)
  ok &= SDO_write_u16(0x6040, 0x00, 0x103F);
  delay(20);

  return ok;
}

/* ===== SDO 기반 위치 명령 (ABS / REL) ===== */
void move_absolute(long pos) {
  currentPos = pos;

  // 필요하면 매 번 SPD 적용 (안 해도 되지만 확실히 하려면)
  apply_target_speed();
  delay(5);

  // 0x103F 상태에서는 607A 갱신만으로 바로 이동
  SDO_write_i32(0x607A, 0x00, currentPos);
  delay(5);

  Serial.print("[ABS] targetPos = ");
  Serial.println(currentPos);
}

void move_relative(long delta) {
  currentPos += delta;
  move_absolute(currentPos);   // 내부적으로는 ABS로 처리
}

/* ===== PDO 핸들용 ABS 위치 명령 ===== */
void pdo_move_absolute(long pos) {
  currentPos = pos;
  PDO_write_position(currentPos);
}

/* ===== 시리얼(USB) 명령 처리 =====
   예:
     en        -> enable_drive()
     rst       -> 0x6040 = 0x0086
     abs 3000  -> 절대 위치 3000으로 이동
     rel 2000  -> 현재 target에서 +2000
     spd 1000  -> targetSpeed = 1000, 0x6081 업데이트
     1         -> MODE_SDO_CMD
     2         -> MODE_PDO_HANDLE
     3         -> MODE_BOTH_DEBUG
     h / help  -> 도움말
*/
void processSerialLine() {
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
        Serial.println("[MODE] SDO CMD (ABS/REL/SPD via SDO)");
        return;
      }
      if (line == "2") {
        controlMode = MODE_PDO_HANDLE;
        Serial.println("[MODE] PDO HANDLE (steering → PDO position)");
        return;
      }
      if (line == "3") {
        controlMode = MODE_BOTH_DEBUG;
        Serial.println("[MODE] BOTH DEBUG (SDO CMD + PDO HANDLE)");
        return;
      }
      if (line == "h" || line == "help") {
        Serial.println("=== COMMAND HELP ===");
        Serial.println("en          : enable drive (pos mode + SPD + 0x103F)");
        Serial.println("rst         : 0x6040 = 0x0086 (fault reset)");
        Serial.println("abs 3000    : absolute move to 3000 (SDO)");
        Serial.println("rel 2000    : relative move +2000 (SDO)");
        Serial.println("spd 1000    : set target speed(Profile velocity, 0x6081) = 1000");
        Serial.println("1           : SDO CMD mode (ABS/REL/SPD via SDO)");
        Serial.println("2           : PDO HANDLE mode (steering → PDO position)");
        Serial.println("3           : BOTH DEBUG");
        return;
      }

      // enable / reset
      if (line == "en") {
        bool ok = enable_drive();
        Serial.println(ok ? "[EN] OK" : "[EN] FAIL");
        return;
      }
      if (line == "rst") {
        bool ok = SDO_write_u16(0x6040, 0x00, 0x0086);
        Serial.println(ok ? "[RST] 0x86 OK" : "[RST] FAIL");
        return;
      }

      // SPD 명령 (어느 모드에서나 허용)
      if (line.startsWith("spd")) {
        String s = line.substring(3);
        s.trim();
        long v = s.toInt();
        if (v <= 0) v = 1;
        targetSpeed = v;
        bool ok = apply_target_speed();
        Serial.print("[SPD] targetSpeed(0x6081) = ");
        Serial.print(targetSpeed);
        Serial.println(ok ? " (OK)" : " (FAIL)");
        return;
      }

      // ABS / REL 명령 (SDO CMD 모드 또는 BOTH에서만 처리)
      if (controlMode == MODE_SDO_CMD || controlMode == MODE_BOTH_DEBUG) {
        if (line.startsWith("abs")) {
          String s = line.substring(3);
          s.trim();
          long p = s.toInt();
          move_absolute(p);
          return;
        }
        if (line.startsWith("rel")) {
          String s = line.substring(3);
          s.trim();
          long d = s.toInt();
          move_relative(d);
          return;
        }
      }

      // 그 외는 로그만 찍고 무시
      Serial.print("[WARN] Unknown cmd: ");
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

/* ===== 핸들 보드에서 들어오는 명령 파싱 (S-500, S1000 등) ===== */
void readSteerCommand() {
  while (linkSerial.available() > 0) {
    char c = linkSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (steerLine.length() == 0) return;

      if (steerLine[0] == 'S') {
        int val = steerLine.substring(1).toInt();
        if (val >  1000) val =  1000;
        if (val < -1000) val = -1000;
        steer_cmd = (int16_t)val;
        lastSteerUpdate = millis();
      }
      steerLine = "";
    } else {
      if (steerLine.length() < 16) {
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
  pinMode(10, OUTPUT);   // SS: SPI 마스터 유지용

  linkSerial.begin(57600);   // 핸들 보드와 동일 속도로 맞추기

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK) {
    Serial.println("CAN init failed");
    while (1) {}
  }
  CAN.setMode(MCP_NORMAL);

  Serial.println("=== Slewing Drive (FD1X5) Controller ===");
  Serial.println("Commands:");
  Serial.println(" en          : enable (pos mode, SPD, 0x103F)");
  Serial.println(" rst         : fault reset (0x86)");
  Serial.println(" abs N       : absolute move to N");
  Serial.println(" rel N       : relative move by N");
  Serial.println(" spd N       : set profile speed(0x6081)=N (default 1000)");
  Serial.println(" 1 / 2 / 3   : mode select (SDO / PDO / BOTH)");
  Serial.println(" h / help    : show help");
  Serial.println("----------------------------------------");
  Serial.println("기본 모드: SDO CMD (ABS/REL/SPD). 먼저 'en' 한 번 쳐서 103F 상태 맞춰줘.");
}

void loop() {
  // 1) PC → 시리얼 명령 처리
  processSerialLine();

  // 2) PDO 핸들 모드일 때만 핸들 값 사용
  if (controlMode == MODE_PDO_HANDLE || controlMode == MODE_BOTH_DEBUG) {
    readSteerCommand();

    // 1초 이상 핸들 명령 없으면 0으로
    if (millis() - lastSteerUpdate > 1000) {
      steer_cmd = 0;
    }

    // –1000 ~ +1000 → –1.0 ~ +1.0
    float norm = (float)steer_cmd / 1000.0f;
    if (norm > -0.02f && norm < 0.02f) {
      norm = 0.0f;
    }

    // –1.0 ~ +1.0 → –MAX_POS ~ +MAX_POS
    long pos = (long)(norm * (float)MAX_POS);

    // ABS 포지션을 PDO로 계속 갱신
    pdo_move_absolute(pos);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print("[PDO] steer_cmd=");
      Serial.print(steer_cmd);
      Serial.print(" norm=");
      Serial.print(norm, 3);
      Serial.print(" pos=");
      Serial.println(pos);
      lastPrint = millis();
    }
  }

  delay(10);
}
