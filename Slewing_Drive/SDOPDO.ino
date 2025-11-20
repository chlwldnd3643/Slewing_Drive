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
    #define MCP_16MHz MCP_16MHz
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

// 슬루잉 모터 목표 위치 범위 (절대 펄스 단위, 기어비 / 엔코더 해상도에 맞게 조정)
long MAX_POS = 200000L;   // 중앙 기준 ±200,000 펄스 예시

mcp2515_can CAN(PIN_CS);

/* ===== 핸들 명령 수신용 SoftwareSerial =====
   실제 배선: 핸들 보드 D8 → 이 보드 D8
*/
const uint8_t LINK_RX_PIN = 8;  // RX (실제 사용 핀)
const uint8_t LINK_TX_PIN = 9;  // 더미 TX (아무것도 안 연결)
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN); // (RX, TX)

// 최신 핸들 명령 값(–1000 ~ +1000)
int16_t steer_cmd = 0;
unsigned long lastSteerUpdate = 0;

// 수신 버퍼
String rxLine = "";

/* ===== 제어 모드 ===== */
enum ControlMode {
  MODE_SDO_ONLY = 0,
  MODE_PDO_ONLY = 1,
  MODE_BOTH     = 2
};

ControlMode controlMode = MODE_SDO_ONLY; // 기본: SDO만 사용

// Position mode에서 New set-point bit(6040 bit4) 토글용
bool toggleNewSetpoint = false;

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

// 32bit SDO (Target Position 0x607A 등에 사용)
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

/* ===== PDO: Target Position 송신 함수 =====
   RPDO1: COB-ID = 0x200 + NODE_ID 에
   4바이트 Target Position(0x607A)을 매핑했다고 가정
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

/* ===== Drive Enable (Profile Position Mode, ABS) ===== */
bool enable_drive(){
  // 1) Mode of operation = 1 (Profile Position)
  SDO_write_u8(0x6060, 0x00, 0x01);
  delay(20);

  // 2) Fault reset-ish (기존에 쓰던 0x0086 유지)
  SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(10);

  // 3) Enable operation (bit0~3 = 1111)
  return SDO_write_u16(0x6040, 0x00, 0x000F);
}

/* ===== 핸들 명령 수신 파싱 =====
   포맷: "S<정수>\n"  예) S0, S-523, S1000
*/
void readSteerCommand()
{
  while (linkSerial.available() > 0) {
    char c = linkSerial.read();
    if (c == '\r') continue; // CR 무시

    if (c == '\n') {
      // 한 줄 완성
      if (rxLine.length() > 0 && rxLine[0] == 'S') {
        // "S-523" 형식
        int val = rxLine.substring(1).toInt(); // 부호 포함 정수
        if (val >  1000) val =  1000;
        if (val < -1000) val = -1000;
        steer_cmd = (int16_t)val;
        lastSteerUpdate = millis();
      }
      rxLine = "";
    } else {
      if (rxLine.length() < 16) {
        rxLine += c;
      } else {
        rxLine = ""; // 너무 길면 버림
      }
    }
  }
}

/* ===== 시리얼 키로 제어 모드 전환 =====
   '1' → SDO only (607A에 SDO로만 쓰기)
   '2' → PDO only (0x200+ID RPDO1만 전송)
   '3' → BOTH
*/
void readModeKey()
{
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1') {
      controlMode = MODE_SDO_ONLY;
      Serial.println("[MODE] SDO only (0x607A SDO write)");
    } else if (c == '2') {
      controlMode = MODE_PDO_ONLY;
      Serial.println("[MODE] PDO only (0x200+ID RPDO1)");
    } else if (c == '3') {
      controlMode = MODE_BOTH;
      Serial.println("[MODE] SDO + PDO 둘 다 사용");
    } else if (c == 'h' || c == 'H') {
      Serial.println("=== MODE HELP ===");
      Serial.println("1 : SDO only  (0x607A:00에 SDO write)");
      Serial.println("2 : PDO only  (0x200+NodeID RPDO1만 사용)");
      Serial.println("3 : BOTH      (SDO + PDO 모두 전송)");
    }
  }
}

/* ===== ABS Position 명령 전송 (Profile Position Mode) =====
   - pos: 목표 위치(절대 값, 펄스 단위)
   - SDO: 0x607A:00 에 Target position 설정
   - PDO: RPDO1로 Target position 전송 (매핑되어 있을 때)
   - Controlword(0x6040)의 bit4(New set-point)를 토글해서 세트포인트 래치
   - bit5(Change set immediately)=1, bit6(Relative/Absolute)=0(ABS)
*/
void send_position_command(long pos)
{
  // 1) 선택된 경로로 Target Position 전송
  if (controlMode == MODE_SDO_ONLY || controlMode == MODE_BOTH) {
    SDO_write_i32(0x607A, 0x00, pos);   // Target position (ABS)
  }
  if (controlMode == MODE_PDO_ONLY || controlMode == MODE_BOTH) {
    PDO_write_position(pos);           // RPDO1로 Target position
  }

  // 2) Controlword 업데이트 (ABS, change immediately, new set-point 토글)
  uint16_t cw = 0x000F;        // enable operation 상태 유지 (bit0~3=1)
  cw |= (1 << 5);              // bit5 = 1: change set immediately
  // bit6 = 0: absolute mode (REL 쓰려면 여기 1로 바꾸면 됨)

  if (toggleNewSetpoint) {
    cw |= (1 << 4);            // bit4 = 1
  }
  // 다음 호출에서 bit4 토글
  toggleNewSetpoint = !toggleNewSetpoint;

  SDO_write_u16(0x6040, 0x00, cw);
}

void setup(){
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);   // SS 핀: SPI 마스터 유지용
  analogReference(DEFAULT);

  linkSerial.begin(57600);   // 핸들 보드와 동일 속도

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK){
    Serial.println("CAN init failed");
    while(1){}
  }
  CAN.setMode(MCP_NORMAL);

  if(!enable_drive()){
    Serial.println("Drive enable failed");
  } else {
    Serial.println("Drive Enabled (Profile Position Mode, ABS).");
  }

  Serial.println("Slewing Arduino READY (Position Mode).");
  Serial.println("Keyboard mode:");
  Serial.println("  1 : SDO only (0x607A SDO)");
  Serial.println("  2 : PDO only (0x200+ID RPDO1)");
  Serial.println("  3 : BOTH");
  Serial.println("  h : help");
}

void loop(){
  // 1) 모드 전환 키 입력
  readModeKey();

  // 2) 핸들 명령 수신
  readSteerCommand();

  // 3) Fail-safe: 1초 이상 명령 없으면 0으로
  if (millis() - lastSteerUpdate > 1000) {
    steer_cmd = 0;
  }

  // 4) –1000 ~ +1000 → –1.0 ~ +1.0
  float norm = (float)steer_cmd / 1000.0f;

  // Deadband: 약간의 중립 영역
  if (norm > -0.02f && norm < 0.02f) {
    norm = 0.0f;
  }

  // 5) –1.0 ~ +1.0 → –MAX_POS ~ +MAX_POS (절대 위치)
  long pos = (long)(norm * (float)MAX_POS);

  // 6) ABS Position 명령 전송 (SDO/PDO 선택적 사용)
  send_position_command(pos);

  // 7) 디버그 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("[MODE=");
    if (controlMode == MODE_SDO_ONLY)      Serial.print("SDO");
    else if (controlMode == MODE_PDO_ONLY) Serial.print("PDO");
    else                                   Serial.print("BOTH");
    Serial.print("] steer_cmd=");
    Serial.print(steer_cmd);
    Serial.print("  norm=");
    Serial.print(norm, 3);
    Serial.print("  pos=");
    Serial.println(pos);
    lastPrint = millis();
  }

  delay(20); // 약 50 Hz로 포지션 업데이트
}
