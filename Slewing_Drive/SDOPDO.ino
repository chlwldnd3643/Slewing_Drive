#include <SPI.h>
#include "mcp2515_can.h"

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

mcp2515_can CAN(PIN_CS);

/* ===== 글로벌 상태 ===== */
long     g_targetPos    = 0;      // 우리가 관리하는 현재 target position
uint32_t g_profileSpeed = 1000;   // 0x6081, 기본 1000

/* ===== 공통 함수 ===== */
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

/* SDO write 32bit (signed) -> 그냥 u32로 쓰되 캐스팅 */
bool SDO_write_i32(uint16_t idx, uint8_t sub, int32_t val) {
  return SDO_write_u32(idx, sub, (uint32_t)val);
}

/* ===== 드라이브 Enable (포지션 모드 + 0x103F) ===== */
bool enable_drive() {
  bool ok = true;

  // 1) Fault reset (0x86)
  ok &= SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(20);

  // 2) Mode of operation = 1 (Position mode)
  ok &= SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 3) Profile speed 초기값 써주기
  ok &= SDO_write_u32(0x6081, 0x00, g_profileSpeed);
  delay(20);

  // 4) Controlword = 0x103F  (타겟 포지션 바뀔 때마다 바로 절대 위치로 움직임)
  ok &= SDO_write_u16(0x6040, 0x00, 0x103F);
  delay(20);

  return ok;
}

/* ===== 위치 명령 함수 ===== */
void move_absolute(long pos) {
  g_targetPos = pos;

  // 현재 profile speed 적용
  SDO_write_u32(0x6081, 0x00, g_profileSpeed);
  delay(5);

  // Target position 갱신하면, 0x103F 상태에서는 바로 그 위치로 움직여야 함
  SDO_write_i32(0x607A, 0x00, g_targetPos);
  delay(5);

  Serial.print("[ABS] target = ");
  Serial.println(g_targetPos);
}

void move_relative(long delta) {
  g_targetPos += delta;
  move_absolute(g_targetPos);  // 실제로는 절대값으로만 쓰지만 사용자는 rel처럼 사용
}

/* ===== 시리얼 명령 파서 ===== */
void handleSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  cmd.toLowerCase();

  if (cmd == "en") {
    bool ok = enable_drive();
    Serial.println(ok ? "EN OK" : "EN FAIL");
    return;
  }

  if (cmd == "rst") {
    bool ok = SDO_write_u16(0x6040, 0x00, 0x0086);
    Serial.println(ok ? "RST OK" : "RST FAIL");
    return;
  }

  if (cmd.startsWith("abs")) {
    long p = cmd.substring(3).toInt();  // "abs3000" or "abs 3000" 모두 처리됨
    move_absolute(p);
    return;
  }

  if (cmd.startsWith("rel")) {
    long d = cmd.substring(3).toInt();
    move_relative(d);
    return;
  }

  if (cmd.startsWith("spd")) {
    long s = cmd.substring(3).toInt();
    if (s <= 0) s = 1;
    g_profileSpeed = (uint32_t)s;
    bool ok = SDO_write_u32(0x6081, 0x00, g_profileSpeed);
    Serial.print("SPD = ");
    Serial.print(g_profileSpeed);
    Serial.println(ok ? " (OK)" : " (FAIL)");
    return;
  }

  Serial.print("Unknown cmd: ");
  Serial.println(cmd);
}

/* ===== Setup / Loop ===== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK) {
    Serial.println("CAN init failed");
    while (1) {}
  }
  CAN.setMode(MCP_NORMAL);

  Serial.println("READY.");
  Serial.println("Commands: en / rst / abs N / rel N / spd N");
  Serial.println("1) en  -> 드라이브 enable + pos 모드 + 0x103F");
  Serial.println("2) spd 1000  -> profile speed 설정 (다음 이동부터 반영)");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialCommand(line);
  }
}
