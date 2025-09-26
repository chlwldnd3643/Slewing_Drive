#include <SPI.h>
#include "mcp2515_can.h"

/* ===== User Config ===== */
uint8_t  NODE_ID   = 1;     // 대상 Node-ID
const uint8_t PIN_CS  = 9;
const uint8_t PIN_INT = 2;

#define FIXED_BAUD  CAN_250KBPS   // 무조건 250kbps
#define FIXED_CLK   MCP_16MHZ     // 무조건 16MHz

/* ===== Profile ===== */
uint32_t G_VEL = 20000000;  // counts/s
uint32_t G_ACC = 366200;
uint32_t G_DEC = 366200;

mcp2515_can CAN(PIN_CS);

/* ===== 기본 유틸 ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len=8){
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val){
  uint8_t d[8]={0x23,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,
               (uint8_t)(val&0xFF),(uint8_t)((val>>8)&0xFF),
               (uint8_t)((val>>16)&0xFF),(uint8_t)((val>>24)&0xFF)};
  return sendCAN(0x600+NODE_ID, d);
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val){
  uint8_t d[8]={0x2B,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,
               (uint8_t)(val&0xFF),(uint8_t)(val>>8),0,0};
  return sendCAN(0x600+NODE_ID, d);
}

/* ===== Enable / Stop ===== */
bool enable_drive(){
  // Profile Position 모드, accel/vel 세팅 후 enable 시퀀스
  SDO_write_u32(0x6081,0x00,G_VEL);
  SDO_write_u32(0x6083,0x00,G_ACC);
  SDO_write_u32(0x6084,0x00,G_DEC);
  delay(20);
  SDO_write_u16(0x6040,0x00,0x0086); // fault reset-ish
  delay(10);
  return SDO_write_u16(0x6040,0x00,0x103F); // enable
}
void quick_stop(){
  SDO_write_u16(0x6040,0x00,0x000B);
}

/* ===== Commands ===== */
enum RunState { IDLE, RUNNING };
RunState state = IDLE;

void handleCommand(String cmd){
  cmd.trim(); cmd.toUpperCase();

  if (cmd=="S"){ if(enable_drive()){ state=RUNNING; Serial.println("ENABLED"); } return; }
  if (cmd=="STOP"){ quick_stop(); state=IDLE; Serial.println("STOPPED"); return; }

  if (cmd.startsWith("VEL ")){ G_VEL=cmd.substring(4).toInt(); SDO_write_u32(0x6081,0x00,G_VEL); Serial.println(G_VEL); return; }
  if (cmd.startsWith("ACC ")){ G_ACC=cmd.substring(4).toInt(); SDO_write_u32(0x6083,0x00,G_ACC); Serial.println(G_ACC); return; }
  if (cmd.startsWith("DEC ")){ G_DEC=cmd.substring(4).toInt(); SDO_write_u32(0x6084,0x00,G_DEC); Serial.println(G_DEC); return; }

  if (cmd.startsWith("ABS ")){
    if(state!=RUNNING){ Serial.println("Send S first."); return; }
    long t = cmd.substring(4).toInt();
    SDO_write_u32(0x607A,0x00,t);
    uint16_t cw=0x001F; SDO_write_u16(0x6040,0x00,cw); cw&=~(1<<4); SDO_write_u16(0x6040,0x00,cw);
    Serial.print("ABS->"); Serial.println(t);
    return;
  }
  if (cmd.startsWith("REL ")){
    if(state!=RUNNING){ Serial.println("Send S first."); return; }
    long inc = cmd.substring(4).toInt();
    SDO_write_u32(0x607A,0x00,inc);
    uint16_t cw=0x005F; SDO_write_u16(0x6040,0x00,cw); cw&=~(1<<4); SDO_write_u16(0x6040,0x00,cw);
    Serial.print("REL->"); Serial.println(inc);
    return;
  }
}

/* ===== Setup/Loop ===== */
void setup(){
  Serial.begin(115200);
  while(!Serial){}
  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);

  byte ret = CAN.begin(FIXED_BAUD, FIXED_CLK);  // 무조건 250k, 16MHz
  if (ret != CAN_OK){ Serial.println("CAN init failed"); while(1){} }
  CAN.setMode(MCP_NORMAL);

  Serial.println("READY: S | STOP | ABS | REL | VEL/ACC/DEC");
}

void loop(){
  static String buf;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){ if(buf.length()){ handleCommand(buf); buf=""; } }
    else { buf+=c; }
  }
}
