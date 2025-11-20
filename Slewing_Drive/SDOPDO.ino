#include <SPI.h>
#include "mcp2515_can.h"
#include <stdint.h>

/* ===== Compat ===== */
#ifndef MCP_16MHZ
  #ifdef MCP_16MHz
    #define MCP_16MHZ MCP_16MHz
  #endif
#endif
#ifndef MCP_16MHz
  #ifdef MCP_16MHZ
    #define MCP_16MHZ MCP_16MHz
  #endif
#endif
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif

/* ===== User Config ===== */
uint8_t  NODE_ID   = 1;     // 서보 드라이브 노드 ID
const uint8_t PIN_CS  = 9;  // MCP2515 CS
const uint8_t PIN_INT = 2;  // MCP2515 INT 핀

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

// Kinco 매뉴얼 Appendix II 참고 (encoder resolution = 65536)
const uint32_t ENC_RES      = 65536UL;
const long     SPD_MAX_RPM  = 3000;   // 안전상 적당히 제한
const long     SPD_MIN_RPM  = 1;      // 0 이면 정지. 여기선 1rpm 이상만 허용

// PDO 속도 스케일 (기존 코드 유지)
long MAX_VEL = 30000000L;   // ±30,000,000 (임의 스케일, 드라이브에서 상대 값으로 사용)

mcp2515_can CAN(PIN_CS);

/* ===== 제어 모드 ===== */
enum ControlMode {
  MODE_SDO_POS = 0,    // SDO 포지션 모드 (ABS/REL)
  MODE_PDO_VEL = 1     // PDO 속도 모드 (아날로그 A0 → 속도)
};

ControlMode g_mode = MODE_SDO_POS;

/* ===== 상태 변수 ===== */
long g_cmdPosInc = 0;         // 우리가 관리하는 명령 위치 (inc 단위, 절대값)
long g_speedRpm  = 1000;      // position speed / target speed 기본값 (rpm)

/* ===== 기본 유틸 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8) {
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}

// SDO write 8bit
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

// SDO write 16bit
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

// SDO write 32bit
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

/* ===== rpm → 내부 DEC 변환 (매뉴얼 Appendix II 공식 사용) =====
 * DEC = (RPM * 512 * encoder_resolution) / 1875
 * encoder_resolution = 65536 으로 가정 (매뉴얼 Note 기준)
 */
uint32_t rpm_to_dec(long rpm) {
  if (rpm < 0) rpm = -rpm;

  long long num = (long long)rpm * 512LL * (long long)ENC_RES;
  long long dec = num / 1875LL;

  if (dec < 0) dec = 0;
  if (dec > 0xFFFFFFFFLL) dec = 0xFFFFFFFFLL;
  return (uint32_t)dec;
}

/* ===== PDO : Target Velocity (0x200 + NODE_ID) ===== */
// 주의: 여기 vel 값은 rpm이 아니라, 드라이브 내부 스케일과 맞춘 "임의 단위" 그대로.
bool PDO_write_velocity(long vel) {
  uint8_t d[8];
  d[0] = (uint8_t)(vel & 0xFF);
  d[1] = (uint8_t)((vel >> 8) & 0xFF);
  d[2] = (uint8_t)((vel >> 16) & 0xFF);
  d[3] = (uint8_t)((vel >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;

  return sendCAN(0x200 + NODE_ID, d);  // RPDO1 기본 COB-ID
}

/* ===== 속도 설정 (position_speed + target_speed 동시 설정) =====
 * SPD <rpm> 명령에서 사용.
 * - 포지션 모드: 0x6081 Trapezoidal velocity (position speed)
 * - 속도 모드:   0x60FF Target speed
 */
void setSpeedRpm(long rpm) {
  if (rpm < 0) rpm = -rpm;
  if (rpm == 0) {
    // 0 을 주면 정지, 원하면 허용해도 되지만 여기선 최소 1rpm 이상으로 제한
    rpm = SPD_MIN_RPM;
  }
  if (rpm > SPD_MAX_RPM) rpm = SPD_MAX_RPM;

  g_speedRpm = rpm;

  uint32_t dec = rpm_to_dec(rpm);

  // 포지션 모드용 position_speed (Trapezoidal velocity)
  SDO_write_u32(0x6081, 0x00, dec);

  // 속도 모드용 Target speed (-3 / 3 모드에서 사용)
  SDO_write_u32(0x60FF, 0x00, dec);

  Serial.print(F("[SPD] rpm="));
  Serial.print(rpm);
  Serial.print(F(" -> DEC="));
  Serial.println(dec);
}

/* ===== 모드 설정 ===== */

// SDO 포지션 모드 (ABS/REL)
void configure_mode_sdo_position() {
  g_mode = MODE_SDO_POS;

  // Error reset
  SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // Position mode (1)
  SDO_write_u8(0x6060, 0x00, 1);
  delay(20);

  // 기본 position speed 설정
  setSpeedRpm(g_speedRpm);

  Serial.println(F("[MODE] SDO Position mode (ABS/REL)"));
}

// PDO 속도 모드 (아날로그 입력 → 속도)
void configure_mode_pdo_velocity() {
  g_mode = MODE_PDO_VEL;

  // Error reset
  SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // Speed mode with position loop (3) 또는 -3 (즉시 속도)
  SDO_write_u8(0x6060, 0x00, 3);   // 필요시 -3 으로 변경 가능
  delay(20);

  // Enable operation (표준 DS402 스타일로 한 번 Enable)
  SDO_write_u16(0x6040, 0x00, 0x0006); // Switch on disabled → ready
  delay(10);
  SDO_write_u16(0x6040, 0x00, 0x000F); // Enable operation
  delay(10);

  // 속도모드용 target_speed도 갱신
  setSpeedRpm(g_speedRpm);

  Serial.println(F("[MODE] PDO Velocity mode (A0 → Target Velocity)"));
}

/* ===== 포지션 명령 (SDO) ===== */

// 내부 g_cmdPosInc 를 드라이브에 쓰고, 즉시 이동 트리거
bool sdo_send_position(long posInc) {
  g_cmdPosInc = posInc;

  // Target position (0x607A)
  if (!SDO_write_u32(0x607A, 0x00, (uint32_t)g_cmdPosInc)) {
    Serial.println(F("[ERR] SDO write 0x607A failed"));
    return false;
  }
  delay(5);

  // 0x103F: "Immediate absolute positioning as target position changes"
  if (!SDO_write_u16(0x6040, 0x00, 0x103F)) {
    Serial.println(F("[ERR] SDO write 0x6040=0x103F failed"));
    return false;
  }

  Serial.print(F("[MOVE] ABS pos="));
  Serial.println(g_cmdPosInc);
  return true;
}

// ABS 명령: 절대 위치로 이동
void cmd_abs(long posInc) {
  sdo_send_position(posInc);
}

// REL 명령: 현재 위치에서 상대 이동 (우리는 내부에서 절대값으로 변환)
void cmd_rel(long deltaInc) {
  long newPos = g_cmdPosInc + deltaInc;
  sdo_send_position(newPos);
}

/* ===== 시리얼 명령 파서 =====
 * 지원 명령:
 *  - ABS <pos>   : 절대 위치 (inc)
 *  - REL <delta> : 상대 이동 (inc)
 *  - SPD <rpm>   : position speed / target speed 설정
 *  - MODE SDO    : SDO 포지션 모드
 *  - MODE PDO    : PDO 속도 모드
 *  - HELP        : 도움말
 */
void handleSerial() {
  static char buf[40];

  if (!Serial.available()) return;

  size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  if (n == 0) return;
  buf[n] = '\0';

  // 공백 스킵
  char *p = buf;
  while (*p == ' ' || *p == '\t') p++;

  if (strncmp(p, "ABS", 3) == 0) {
    long v = 0;
    if (sscanf(p + 3, "%ld", &v) == 1) {
      cmd_abs(v);
    } else {
      Serial.println(F("Usage: ABS <pos_inc>"));
    }
  }
  else if (strncmp(p, "REL", 3) == 0) {
    long v = 0;
    if (sscanf(p + 3, "%ld", &v) == 1) {
      cmd_rel(v);
    } else {
      Serial.println(F("Usage: REL <delta_inc>"));
    }
  }
  else if (strncmp(p, "SPD", 3) == 0) {
    long rpm = 0;
    if (sscanf(p + 3, "%ld", &rpm) == 1) {
      setSpeedRpm(rpm);
    } else {
      Serial.print(F("[SPD] current rpm="));
      Serial.println(g_speedRpm);
      Serial.println(F("Usage: SPD <rpm>"));
    }
  }
  else if (strncmp(p, "MODE", 4) == 0) {
    // MODE 뒤 두 번째 토큰
    char m[8] = {0};
    if (sscanf(p + 4, "%7s", m) == 1) {
      if (strcmp(m, "SDO") == 0) {
        configure_mode_sdo_position();
      } else if (strcmp(m, "PDO") == 0) {
        configure_mode_pdo_velocity();
      } else {
        Serial.println(F("Usage: MODE SDO | MODE PDO"));
      }
    } else {
      Serial.println(F("Usage: MODE SDO | MODE PDO"));
    }
  }
  else if (strncmp(p, "HELP", 4) == 0) {
    Serial.println(F("Commands:"));
    Serial.println(F("  ABS <pos_inc>   : move to absolute position (inc)"));
    Serial.println(F("  REL <delta_inc> : move relative (inc)"));
    Serial.println(F("  SPD <rpm>       : set position_speed / target_speed (rpm)"));
    Serial.println(F("  MODE SDO        : SDO position mode (ABS/REL)"));
    Serial.println(F("  MODE PDO        : PDO velocity mode (A0 -> Target Velocity)"));
  }
  else {
    Serial.print(F("Unknown cmd: "));
    Serial.println(p);
    Serial.println(F("Type HELP for list."));
  }
}

/* ===== Setup / Loop ===== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);        // MCP2515 라이브러리 예제 호환용
  analogReference(DEFAULT);

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK) {
    Serial.println(F("CAN init failed"));
    while (1) {}
  }
  CAN.setMode(MCP_NORMAL);

  Serial.println(F("Kinco FD1x5 / iSMK Control via CANopen"));
  Serial.println(F("Commands: ABS, REL, SPD, MODE SDO/PDO, HELP"));

  // 기본은 SDO 포지션 모드로 시작
  configure_mode_sdo_position();
}

void loop() {
  // 시리얼 명령 처리
  handleSerial();

  // PDO 속도 모드일 때만 A0 → PDO 전송
  if (g_mode == MODE_PDO_VEL) {
    int raw = analogRead(A0);                       // 0~1023
    float norm = (raw / 1023.0f) * 2.0f - 1.0f;     // -1.0 ~ +1.0
    long vel = (long)(norm * MAX_VEL);

    PDO_write_velocity(vel);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      Serial.print(F("[PDO] A0=")); Serial.print(raw);
      Serial.print(F(" -> vel=")); Serial.println(vel);
      lastPrint = millis();
    }
    delay(10);
  }
  else {
    // SDO 포지션 모드에서는 루프에서 할 일 없음 (명령은 시리얼로만)
    delay(5);
  }
}
