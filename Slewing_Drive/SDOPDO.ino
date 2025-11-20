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

// Kinco FD1X5 Target speed 범위 (필요시 조정)
long MAX_VEL = 30000000L;   // ±30,000,000

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

ControlMode controlMode = MODE_SDO_ONLY; // 초기값: SDO만 사용

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

// Target speed용 32bit SDO (0x60FF:00에 쓰기)
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

/* ===== PDO 송신 함수 =====
   RPDO1: COB-ID = 0x200 + NODE_ID 에
   4바이트 속도값만 싣는다고 가정
*/
bool PDO_write_velocity(long vel){
  uint8_t d[8];
  d[0] = (uint8_t)( vel        & 0xFF);
  d[1] = (uint8_t)((vel >> 8 ) & 0xFF);
  d[2] = (uint8_t)((vel >> 16) & 0xFF);
  d[3] = (uint8_t)((vel >> 24) & 0xFF);
  d[4] = d[5] = d[6] = d[7] = 0;   // 나머지 비움
  return sendCAN(0x200 + NODE_ID, d); // RPDO1 기본 COB-ID
}

/* ===== Drive Enable (Velocity mode) ===== */
bool enable_drive(){
  // Velocity mode: 0x6060:00 = 3
  SDO_write_u8(0x6060, 0x00, 0x03);
  delay(20);

  // Fault reset-ish: 0x6040:00 = 0x0086
  SDO_write_u16(0x6040, 0x00, 0x0086);
  delay(10);

  // Enable Operation: 0x6040:00 = 0x000F
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
   '1' → SDO only
   '2' → PDO only
   '3' → BOTH
*/
void readModeKey()
{
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1') {
      controlMode = MODE_SDO_ONLY;
      Serial.println("[MODE] SDO only (0x60FF에 SDO로만 쓰기)");
    } else if (c == '2') {
      controlMode = MODE_PDO_ONLY;
      Serial.println("[MODE] PDO only (0x200+ID RPDO1만 전송)");
    } else if (c == '3') {
      controlMode = MODE_BOTH;
      Serial.println("[MODE] SDO + PDO 둘 다 전송");
    } else if (c == 'h' || c == 'H') {
      Serial.println("=== MODE HELP ===");
      Serial.println("1 : SDO only  (0x60FF:00에만 SDO write)");
      Serial.println("2 : PDO only  (0x200+NodeID RPDO1만 전송)");
      Serial.println("3 : BOTH      (SDO + PDO 모두 전송)");
    }
  }
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
    Serial.println("Drive Enabled.");
  }

  Serial.println("Slewing Arduino READY.");
  Serial.println("Keyboard mode:");
  Serial.println("  1 : SDO only");
  Serial.println("  2 : PDO only");
  Serial.println("  3 : BOTH (default=SDO only)");
  Serial.println("  h : help");
}

void loop(){
  // 1) 제어 모드 전환 키 읽기
  readModeKey();

  // 2) 핸들 명령 수신
  readSteerCommand();

  // 3) Fail-safe: 1초 이상 명령 없으면 0으로
  if (millis() - lastSteerUpdate > 1000) {
    steer_cmd = 0;
  }

  // 4) –1000 ~ +1000 → –1.0 ~ +1.0
  float norm = (float)steer_cmd / 1000.0f;

  // Deadband: |norm| < 0.05 이면 0으로
  if (norm > -0.05f && norm < 0.05f) {
    norm = 0.0f;
  }

  // 5) Target speed 계산
  long vel = (long)(norm * (float)MAX_VEL);

  // 6) 선택된 모드에 따라 SDO/PDO 전송
  if (controlMode == MODE_SDO_ONLY) {
    SDO_write_i32(0x60FF, 0x00, vel);
  } else if (controlMode == MODE_PDO_ONLY) {
    PDO_write_velocity(vel);
  } else { // MODE_BOTH
    SDO_write_i32(0x60FF, 0x00, vel);
    PDO_write_velocity(vel);
  }

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
    Serial.print("  vel=");
    Serial.println(vel);
    lastPrint = millis();
  }

  delay(10); // ≈100 Hz 주기로 속도 갱신
}
