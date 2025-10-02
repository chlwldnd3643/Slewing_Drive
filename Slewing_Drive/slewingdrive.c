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
    #define MCP_16MHz MCP_16MHZ
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
const uint8_t PIN_CS  = 9;
const uint8_t PIN_INT = 2;

#define FIXED_BAUD  CAN_250KBPS
#define FIXED_CLK   MCP_16MHz

/* ===== Profile ===== */
long MAX_VEL = 30000000;   // ±30,000,000

mcp2515_can CAN(PIN_CS);

/* ===== 기본 유틸 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len=8){
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}
bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val){
  uint8_t d[8]={0x2F,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,
               val,0,0,0};
  return sendCAN(0x600+NODE_ID, d);
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val){
  uint8_t d[8]={0x2B,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,
               (uint8_t)(val&0xFF),(uint8_t)(val>>8),0,0};
  return sendCAN(0x600+NODE_ID, d);
}

/* ===== PDO 송신 함수 ===== */
// Target Velocity (obj 0x6042)는 표준적으로 32bit
bool PDO_write_velocity(long vel){
  uint8_t d[8];
  d[0] = (uint8_t)(vel & 0xFF);
  d[1] = (uint8_t)((vel>>8) & 0xFF);
  d[2] = (uint8_t)((vel>>16)& 0xFF);
  d[3] = (uint8_t)((vel>>24)& 0xFF);
  d[4]=d[5]=d[6]=d[7]=0;   // 나머지 비움
  return sendCAN(0x200+NODE_ID, d); // RPDO1 기본 COB-ID
}

/* ===== Enable ===== */
bool enable_drive(){
  // Velocity mode (0x6060=3)
  SDO_write_u8(0x6060,0x00,0x03);

  delay(20);
  SDO_write_u16(0x6040,0x00,0x0086); // fault reset-ish
  delay(10);
  return SDO_write_u16(0x6040,0x00,0x000F); // Enable Operation
}

/* ===== Setup/Loop ===== */
void setup(){
  Serial.begin(115200);
  while(!Serial){}
  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);
  analogReference(DEFAULT);

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);
  if (ret != CAN_OK){ Serial.println("CAN init failed"); while(1){} }
  CAN.setMode(MCP_NORMAL);

  enable_drive();

  Serial.println("READY: analog A0 → PDO Target Velocity ±30M");
}

void loop(){
  int raw = analogRead(A0);        // 0~1023
  float norm = (raw/1023.0f)*2.0f - 1.0f; // -1.0 ~ +1.0
  long vel = (long)(norm * MAX_VEL);

  PDO_write_velocity(vel);

  static unsigned long last=0;
  if(millis()-last>500){
    Serial.print("A0="); Serial.print(raw);
    Serial.print(" -> vel="); Serial.println(vel);
    last=millis();
  }
  delay(10); // PDO 전송 주기
}
