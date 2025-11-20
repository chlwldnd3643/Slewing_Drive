#include <SPI.h>
#include "mcp2515_can.h"
#include <SoftwareSerial.h"

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
uint8_t  NODE_ID   = 1;
const uint8_t PIN_CS  = 9;    // MCP2515 CS
const uint8_t PIN_INT = 2;    // MCP2515 INT

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

// 핸들 → PDO 모드에서 쓸 최대 위치 (중앙 기준 ±MAX_POS)
long MAX_POS = 200000L;   // 기어비/엔코더에 맞게 조정

// Profile Position Mode에서 쓸 Target speed (Profile velocity, obj 0x6081)
// 기본값 1000
long targetSpeed = 1000L;

mcp2515_can CAN(PIN_CS);

/* ===== 핸들 명령 수신용 SoftwareSerial =====
   실제 배선: 핸들 보드 D8 → 이 보드 D8
*/
const uint8_t LINK_RX_PIN = 8;  // RX
const uint8_t LINK_TX_PIN = 9;  // 더미 TX
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN); // (RX, TX)

// 최신 핸들 명령 값(–1000 ~ +1000)
int16_t steer_cmd = 0;
unsigned long lastSteerUpdate = 0;

// 시리얼 명령/핸들 수신 버퍼
String serialLine = "";
String steerLine  = "";

/* ===== 제어 모드 ===== */
enum ControlMode {
  MODE_SDO_CMD    = 0,   // ABS 3000, REL 2000 같은 SDO 명령 모드
  MODE_PDO_HANDLE = 1,   // 핸들 → PDO 실시간 포지션
  MODE_BOTH_DEBUG = 2
};

ControlMode controlMode = MODE_SDO_CMD; // 기본: SDO 명령 모드

// Position mode에서 New set-point(bit4) 토글용
bool toggleNewSetpoint = false;

// 내부에서 기억하는 "지금까지 보낸 절대 위치"
long currentPos = 0;

/* ===== 기본 CAN 유틸 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len=8){
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}

bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val){
  uint8_t d[8] = {
    0x2F,
    (uint8_t)(idx & 0xFF),
    (uint8_t)(idx >> 8),
    sub,
    val, 0, 0, 0
  };
  return sendCAN(0x600 + NODE_ID, d);
}

bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val){
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

// 32bit SDO (Target Position 0x607A, Target Speed 0x6081 등에 사용)
bool SDO_write_i32(uint16_t idx, uint8_t sub, int32_t val){
  uint8_t d[8] = {
    0x23,  // 4바이트 write
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

/* ===== Target Speed(0x6081) 적용 함수 ===== */
bool apply_target_speed(){
  // Profile Velocity = targetSpeed
  return SDO_write_i32(0x6081, 0x00, targetSpeed);
}

/* ===== PDO: Target Position 송신 =====
   RPDO1: COB-ID = 0x200 + NODE_ID 에
   4바이트 Target Position(0x607A)이 매핑되어 있다고 가정
*/
bool PDO_write_position(long pos){
  uint8_t d[8];
  d[0] = (uint8_t)( pos        & 0xFF);
  d[1] = (uint8_t)((pos >> 8 ) & 0xFF);
  d[2] = (uint8_t)((pos >> 16) & 0xFF);
  d[3] = (uint8_t)((pos >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;
  return sendCAN(0x200 + NODE_ID, d); // RPDO1 COB-ID
}

/* ===== Drive Enable (Profile Position Mode) ===== */
bool enable_drive(){
  // 1) Mode of operation = 1 (Profile Position)
  SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 2) Target speed(Profile Velocity) 설정
  apply_target_speed();
  delay(10);

  // 3) Fault reset-ish
  SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(10);

  // 4) Enable operation
  return SDO_write_u16(0x6040, 0x00, 0x000F);
}

/* ===== Position 명령 공통 함수 =====
   isRelative = false → ABS pos, true → REL delta
   useSDO     = true  → 0x607A에 SDO write
   usePDO     = true  → RPDO1로 Target Position 전송
*/
void send_position_core(bool isRelative, long value, bool useSDO, bool usePDO)
{
  if (useSDO) {
    // ABS면 0x607A에 최종 위치, REL이면 이동량
    SDO_write_i32(0x607A, 0x00, value);
  }

  if (usePDO) {
    PDO_write_position(value);
  }

  // Controlword 구성
  uint16_t cw = 0x000F;        // enable operation 유지
  cw |= (1 << 5);              // bit5 = 1: change set immediately

  if (isRelative) {
    cw |= (1 << 6);            // bit6 = 1: relative
  } else {
    // ABS: bit6 = 0
  }

  if (toggleNewSetpoint) {
    cw |= (1 << 4);            // bit4 = 1: new set-point
  }
  toggleNewSetpoint = !toggleNewSetpoint;

  SDO_write_u16(0x6040, 0x00, cw);
}

/* ===== SDO ABS/REL 전용 래퍼 ===== */
void sdo_move_abs(long pos){
  currentPos = pos;
  send_position_core(false, pos, true, false); // ABS, SDO만
}

void sdo_move_rel(long delta){
  currentPos += delta;
  send_position_core(true, delta, true, false); // REL, SDO만 (value=delta)
}

/* ===== PDO 핸들용 ABS 포지션 ===== */
void pdo_move_abs(long pos){
  currentPos = pos;
  send_position_core(false, pos, false, true); // ABS, PDO만
}

/* ===== 시리얼(USB) 한 줄 읽기: 모드 + SDO 명령 + Target Speed =====
   예:
     "1"           → 모드 SDO_CMD
     "2"           → 모드 PDO_HANDLE
     "3"           → BOTH_DEBUG
     "ABS 3000"    → 절대 위치 3000으로 이동 (SDO)
     "REL 2000"    → 현재 위치 기준 +2000 (SDO)
     "REL -1000"   → 현재 위치 기준 -1000 (SDO)
     "SPD 1000"    → Target speed(0x6081) = 1000 으로 설정
*/
void processSerialLine()
{
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (serialLine.length() == 0) return;

      String line = serialLine;
      serialLine = "";

      line.trim();
      line.toUpperCase();

      // 모드 전환
      if (line == "1") {
        controlMode = MODE_SDO_CMD;
        Serial.println("[MODE] SDO CMD (ABS/REL via SDO)");
        return;
      }
      if (line == "2") {
        controlMode = MODE_PDO_HANDLE;
        Serial.println("[MODE] PDO HANDLE (steering → PDO pos)");
        return;
      }
      if (line == "3") {
        controlMode = MODE_BOTH_DEBUG;
        Serial.println("[MODE] BOTH DEBUG (SDO CMD + PDO HANDLE)");
        return;
      }
      if (line == "H" || line == "HELP") {
        Serial.println("=== HELP ===");
        Serial.println("1         : SDO CMD mode (ABS/REL via SDO)");
        Serial.println("2         : PDO HANDLE mode (steering → PDO pos)");
        Serial.println("3         : BOTH DEBUG");
        Serial.println("ABS 3000  : absolute move to 3000 (SDO)");
        Serial.println("REL 2000  : relative move +2000 (SDO)");
        Serial.println("REL -1000 : relative move -1000 (SDO)");
        Serial.println("SPD 1000  : set target speed(Profile Velocity) = 1000");
        return;
      }

      // Target speed 설정 (모드 상관없이 항상 허용)
      if (line.startsWith("SPD")) {
        line.remove(0, 3);
        line.trim();
        long v = line.toInt();
        if (v <= 0) v = 1;   // 0 이하 방지
        targetSpeed = v;
        apply_target_speed();
        Serial.print("[SPD] target speed(Profile Velocity) = ");
        Serial.println(targetSpeed);
        return;
      }

      // SDO ABS/REL 명령 (SDO CMD 모드 또는 BOTH에서만 처리)
      if (controlMode == MODE_SDO_CMD || controlMode == MODE_BOTH_DEBUG) {
        if (line.startsWith("ABS")) {
          line.remove(0, 3);
          line.trim();
          long val = line.toInt();
          sdo_move_abs(val);
          Serial.print("[SDO ABS] target = ");
          Serial.println(val);
          return;
        }
        if (line.startsWith("REL")) {
          line.remove(0, 3);
          line.trim();
          long delta = line.toInt();
          sdo_move_rel(delta);
          Serial.print("[SDO REL] delta = ");
          Serial.print(delta);
          Serial.print("  currentPos = ");
          Serial.println(currentPos);
          return;
        }
      }

      // 그 외 문자열은 무시
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
void readSteerCommand()
{
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

void setup(){
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);   // SS: SPI 마스터 유지용
  analogReference(DEFAULT);

  linkSerial.begin(57600);   // 핸들 보드와 동일

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK){
    Serial.println("CAN init failed");
    while(1){}
  }
  CAN.setMode(MCP_NORMAL);

  if(!enable_drive()){
    Serial.println("Drive enable failed");
  } else {
    Serial.print("Drive Enabled (Profile Position Mode). Default SPD=");
    Serial.println(targetSpeed);
  }

  currentPos = 0;
  sdo_move_abs(0);   // 초기 위치 0으로 한번 맞추기

  Serial.println("Slewing Arduino READY (Position Mode).");
  Serial.println("Use:");
  Serial.println(" 1          → SDO CMD mode (ABS/REL via SDO)");
  Serial.println(" 2          → PDO HANDLE mode (steering → PDO pos)");
  Serial.println(" 3          → BOTH DEBUG");
  Serial.println(" ABS 3000   → absolute move to 3000");
  Serial.println(" REL 2000   → relative move +2000");
  Serial.println(" REL -1000  → relative move -1000");
  Serial.println(" SPD 1000   → set target speed(Profile Velocity) = 1000");
  Serial.println(" H / HELP   → show help");
}

void loop(){
  // 1) PC → 시리얼 명령 처리 (모드 전환 + SDO ABS/REL + SPD)
  processSerialLine();

  // 2) 핸들 → PDO 모드일 때만 핸들 값 사용
  if (controlMode == MODE_PDO_HANDLE || controlMode == MODE_BOTH_DEBUG) {
    readSteerCommand();

    if (millis() - lastSteerUpdate > 1000) {
      steer_cmd = 0;
    }

    float norm = (float)steer_cmd / 1000.0f;
    if (norm > -0.02f && norm < 0.02f) {
      norm = 0.0f;
    }

    long pos = (long)(norm * (float)MAX_POS);
    pdo_move_abs(pos);   // ABS 포지션을 PDO로 계속 갱신

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
