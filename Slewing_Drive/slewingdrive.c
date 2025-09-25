#include <SPI.h>
#include "mcp2515_can.h"

/* ========= USER CONFIG ========= */
uint8_t  NODE_ID         = 1;            // 대상 드라이브 Node-ID
const bool MCP2515_16MHZ = true;         // MCP2515 클럭
const uint8_t PIN_CS     = 9;
const uint8_t PIN_INT    = 2;
#define DEFAULT_BAUD      CAN_500KBPS     // 필요시 CAN_250KBPS 등으로

/* ========= GLOBAL PROFILE (런타임 변경됨) ========= */
uint32_t G_VEL = 10000000;  // 0x6081 Profile Velocity (counts/s)
uint32_t G_ACC = 366200;    // 0x6083 Accel
uint32_t G_DEC = 366200;    // 0x6084 Decel

/* ========= MCP2515 ========= */
mcp2515_can CAN(PIN_CS);

/* ========= Dynamic COB-IDs ========= */
inline uint32_t COB_NMT()    { return 0x000; }
inline uint32_t COB_SDO_TX() { return 0x600 + NODE_ID; }
inline uint32_t COB_SDO_RX() { return 0x580 + NODE_ID; }

/* ========= CAN Utils ========= */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8){
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}

// 경험적 기본값: 대부분의 SDO는 수십 ms 내 응답 → 200ms면 충분
bool waitSDO(uint16_t index, uint8_t sub, uint32_t timeout_ms = 200){
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms){
    if (CAN.checkReceive() == CAN_MSGAVAIL){
      unsigned long rxId=0; unsigned char len=0, buf[8]={0};
      if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK){
        if (rxId == COB_SDO_RX() && len >= 8){
          uint16_t idx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
          if (idx == index && buf[3]==sub){
            if (buf[0]==0x60) return true;   // SDO ack
            if (buf[0]==0x80) return false;  // SDO abort
          }
        }
      }
    }
  }
  return false;
}

/* ========= SDO helpers ========= */
bool SDO_write_u8 (uint16_t idx, uint8_t sub, uint8_t  val, bool ack=true){
  uint8_t d[8]={0x2F,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,val,0,0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return ack ? waitSDO(idx, sub) : true;
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val, bool ack=true){
  uint8_t d[8]={0x2B,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,(uint8_t)(val&0xFF),(uint8_t)(val>>8),0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return ack ? waitSDO(idx, sub) : true;
}
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val, bool ack=true){
  uint8_t d[8]={0x23,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,
               (uint8_t)(val&0xFF),(uint8_t)((val>>8)&0xFF),
               (uint8_t)((val>>16)&0xFF),(uint8_t)((val>>24)&0xFF)};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return ack ? waitSDO(idx, sub) : true;
}

/* ========= NMT ========= */
void NMT_cmd(uint8_t cmd, uint8_t node){ uint8_t d[2]={cmd,node}; sendCAN(COB_NMT(), d, 2); }
void NMT_start(uint8_t node){ NMT_cmd(0x01, node); }

/* ========= FAST ENABLE (PP mode + 현재 G_* 적용) ========= */
bool enable_drive_fast(){
  // 모드/프로파일(ack 최소화)
  SDO_write_u8 (0x6060,0x00,1,false);             // Profile Position
  SDO_write_u32(0x6081,0x00,G_VEL, true);         // Profile Velocity
  SDO_write_u32(0x607F,0x00,G_VEL*5, true);       // Max Profile Velocity(여유)
  SDO_write_u32(0x6083,0x00,G_ACC, true);         // Accel
  SDO_write_u32(0x6084,0x00,G_DEC, true);         // Decel
  SDO_write_u8 (0x6086,0x00,0,     true);         // Trapezoidal

  // 빠른 Controlword 시퀀스
  SDO_write_u16(0x6040,0x00,0x0086,false);        // fault reset-ish
  delay(10);
  bool ok = SDO_write_u16(0x6040,0x00,0x103F,false); // enable + newSP + change immed
  delay(10);
  return ok;
}

/* ========= RUNTIME COMMANDS ========= */
enum RunState { IDLE, RUNNING };
RunState state = IDLE;

void handleCommand(String cmd){
  cmd.trim(); cmd.toUpperCase();

  // --- S: Fast enable ---
  if (cmd=="S"){
    if (enable_drive_fast()){ state=RUNNING; Serial.println(F("[OK] ENABLED (PP)")); }
    else                    { Serial.println(F("[WARN] Enable failed")); }
    return;
  }

  // --- VEL/ACC/DEC: 프로파일 파라미터 변경 ---
  if (cmd.startsWith("VEL ")){
    long v = cmd.substring(4).toInt();
    if (v>0){ G_VEL=v; SDO_write_u32(0x6081,0x00,G_VEL,true); SDO_write_u32(0x607F,0x00,G_VEL*5,true); }
    Serial.print(F("VEL=")); Serial.println(G_VEL);
    return;
  }
  if (cmd.startsWith("ACC ")){
    long a = cmd.substring(4).toInt();
    if (a>0){ G_ACC=a; SDO_write_u32(0x6083,0x00,G_ACC,true); }
    Serial.print(F("ACC=")); Serial.println(G_ACC);
    return;
  }
  if (cmd.startsWith("DEC ")){
    long d = cmd.substring(4).toInt();
    if (d>0){ G_DEC=d; SDO_write_u32(0x6084,0x00,G_DEC,true); }
    Serial.print(F("DEC=")); Serial.println(G_DEC);
    return;
  }

  // --- ABS / 숫자만: 절대이동 ---
  if (cmd.startsWith("ABS ")){
    if (state!=RUNNING){ Serial.println(F("Send S first.")); return; }
    long t = cmd.substring(4).toInt();
    SDO_write_u32(0x607A,0x00,(uint32_t)t,false);
    uint16_t cw=0x001F; SDO_write_u16(0x6040,0x00,cw,false);
    cw&=~(1<<4);        SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("ABS -> ")); Serial.println(t);
    return;
  }
  bool allDigitOrSign = cmd.length()>0;
  for(size_t i=0;i<cmd.length();++i){ char ch=cmd[i]; if(!(isDigit(ch)||ch=='-'||ch=='+')){allDigitOrSign=false;break;} }
  if(allDigitOrSign){
    if (state!=RUNNING){ Serial.println(F("Send S first.")); return; }
    long t = cmd.toInt();
    SDO_write_u32(0x607A,0x00,(uint32_t)t,false);
    uint16_t cw=0x001F; SDO_write_u16(0x6040,0x00,cw,false);
    cw&=~(1<<4);        SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("ABS -> ")); Serial.println(t);
    return;
  }

  // --- REL: 상대이동 ---
  if (cmd.startsWith("REL ")){
    if (state!=RUNNING){ Serial.println(F("Send S first.")); return; }
    long inc = cmd.substring(4).toInt();
    SDO_write_u32(0x607A,0x00,(uint32_t)inc,false);
    uint16_t cw=0x005F; SDO_write_u16(0x6040,0x00,cw,false); // relative bit 포함
    cw&=~(1<<4);        SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("REL -> ")); Serial.println(inc);
    return;
  }
}

/* ========= Setup/Loop ========= */
void setup(){
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT);

  byte ret = MCP2515_16MHZ ? CAN.begin(DEFAULT_BAUD, MCP_16MHZ)
                           : CAN.begin(DEFAULT_BAUD, MCP_8MHZ);
  if (ret != CAN_OK){ Serial.println(F("CAN init failed")); while(1){} }

  CAN.setMode(MCP_NORMAL);
  NMT_start(0x00); // CANopen 장비에만 의미

  Serial.println(F("READY: S | ABS <inc> | REL <inc> | VEL <counts/s> | ACC <counts/s^2> | DEC <counts/s^2>"));
}

void loop(){
  static String buf;
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r'){
      if (buf.length()){ handleCommand(buf); buf=""; }
    } else {
      buf += c;
      // 숫자는 즉시 ABS로
      bool allDigitOrSign = buf.length()>0;
      for(size_t i=0;i<buf.length();++i){ char ch=buf[i]; if(!(isDigit(ch)||ch=='-'||ch=='+')){allDigitOrSign=false;break;} }
      if (allDigitOrSign){ handleCommand(buf); buf=""; }
    }
  }
}
