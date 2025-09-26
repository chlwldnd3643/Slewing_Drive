#include <SPI.h>
#include "mcp2515_can.h"

/* ===== Compat (라이브러리 차이 흡수) ===== */
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef MCP_LOOPBACK
  #define MCP_LOOPBACK 0x02
#endif
#ifndef MCP_16MHz
  #ifdef MCP_16MHZ
    #define MCP_16MHz MCP_16MHZ
  #endif
#endif
#ifndef MCP_8MHz
  #ifdef MCP_8MHZ
    #define MCP_8MHz MCP_8MHZ
  #endif
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif
#ifndef CAN_MSGAVAIL
  #define CAN_MSGAVAIL (1)
#endif

/* ===== User Config ===== */
uint8_t  NODE_ID         = 1;            // 대상 Node-ID
const bool MCP2515_16MHZ = true;         // MCP2515 크리스탈 (true=16MHz / false=8MHz)
const uint8_t PIN_CS     = 9;
const uint8_t PIN_INT    = 2;
#define DEFAULT_BAUD      CAN_500KBPS    // 필요시 CAN_250KBPS 등

/* ===== Motion Profile (runtime 변경 가능) ===== */
uint32_t G_VEL = 20000000;  // 0x6081 counts/s
uint32_t G_ACC = 366200;    // 0x6083
uint32_t G_DEC = 366200;    // 0x6084

/* ===== MCP2515 & helpers ===== */
mcp2515_can CAN(PIN_CS);

static inline bool CAN_readFrame(mcp2515_can& dev, unsigned long &id,
                                 unsigned char &len, unsigned char *buf) {
  if (dev.readMsgBuf(&len, buf) == CAN_OK) { id = dev.getCanId(); return true; }
  return false;
}
static inline byte CAN_begin_compat(mcp2515_can& dev, uint8_t baud, bool use16MHz){
#if defined(MCP_16MHz) && defined(MCP_8MHz)
  return dev.begin(baud, use16MHz ? MCP_16MHz : MCP_8MHz);
#else
  (void)use16MHz; return dev.begin(baud);
#endif
}

/* ===== COB-IDs ===== */
inline uint32_t COB_NMT()    { return 0x000; }
inline uint32_t COB_SDO_TX() { return 0x600 + NODE_ID; }
inline uint32_t COB_SDO_RX() { return 0x580 + NODE_ID; }

/* ===== CAN Utils ===== */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8){
  return CAN.sendMsgBuf(id, 0, len, (unsigned char*)data) == CAN_OK;
}
bool waitSDO(uint16_t index, uint8_t sub, uint32_t timeout_ms = 200){
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms){
    if (CAN.checkReceive() == CAN_MSGAVAIL){
      unsigned long rxId=0; unsigned char len=0, buf[8]={0};
      if (CAN_readFrame(CAN, rxId, len, buf)){
        if (rxId == COB_SDO_RX() && len >= 8){
          uint16_t idx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
          if (idx == index && buf[3]==sub){
            if (buf[0]==0x60) return true;   // ack
            if (buf[0]==0x80) return false;  // abort
          }
        }
      }
    }
  }
  return false;
}

/* ===== SDO helpers ===== */
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

/* ===== NMT ===== */
void NMT_cmd  (uint8_t cmd, uint8_t node){ uint8_t d[2]={cmd,node}; sendCAN(COB_NMT(), d, 2); }
void NMT_start(uint8_t node){ NMT_cmd(0x01, node); }  // Operational
void NMT_preop(uint8_t node){ NMT_cmd(0x80, node); }  // Pre-Operational

/* ===== READ (any) ===== */
bool SDO_read_any(uint16_t idx, uint8_t sub, uint32_t &val, uint8_t &nbytes, uint32_t to_ms=300){
  uint8_t rq[8]={0x40,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,0,0,0,0};
  if(!sendCAN(COB_SDO_TX(), rq)) return false;
  uint32_t t0=millis();
  while(millis()-t0<to_ms){
    if(CAN.checkReceive()==CAN_MSGAVAIL){
      unsigned long rx; unsigned char len; unsigned char b[8];
      if(CAN_readFrame(CAN, rx, len, b)){
        if(rx==COB_SDO_RX() && len>=8){
          if(b[0]==0x80) return false; // abort
          if((b[0]&0xE0)==0x40){
            uint8_t n = 4 - ((b[0]>>2)&0x03);
            nbytes = n;
            val = (uint32_t)b[4] | ((uint32_t)b[5]<<8) | ((uint32_t)b[6]<<16) | ((uint32_t)b[7]<<24);
            return true;
          }
        }
      }
    }
  }
  return false;
}

/* ===== ENABLE (요청 버전: 0x0086 → 0x103F) ===== */
bool enable_drive_fast(){
  // Pre-Op에서 프로파일 셋업
  NMT_preop(NODE_ID); delay(20);
  SDO_write_u8 (0x6060,0x00,1,false);             // Profile Position
  if(!SDO_write_u32(0x6081,0x00,G_VEL, true)) return false;
  SDO_write_u32(0x607F,0x00,G_VEL*5, true);
  SDO_write_u32(0x6083,0x00,G_ACC, true);
  SDO_write_u32(0x6084,0x00,G_DEC, true);
  SDO_write_u8 (0x6086,0x00,0,     true);         // trapezoidal

  // Operational 진입
  NMT_start(NODE_ID); delay(20);

  // Fault reset-ish (ack 안 기다림)
  SDO_write_u16(0x6040,0x00,0x0086,false);
  delay(10);

  // enable + newSP + change immed (벤더 의존)
  bool ok = SDO_write_u16(0x6040,0x00,0x103F,false);
  delay(10);
  return ok;
}

/* ===== 진단/유틸 커맨드 ===== */
void cmd_DIAG(){
  uint32_t v; uint8_t nb;
  if(SDO_read_any(0x6041,0x00,v,nb)) { Serial.print(F("6041 Status=0x")); Serial.println((uint16_t)v,HEX); }
  if(SDO_read_any(0x603F,0x00,v,nb)) { Serial.print(F("603F ErrCode=0x")); Serial.println((uint16_t)v,HEX); }
  if(SDO_read_any(0x1001,0x00,v,nb)) { Serial.print(F("1001 ErrReg=0x")); Serial.println((uint8_t)v,HEX); }
  if(SDO_read_any(0x6061,0x00,v,nb)) { Serial.print(F("6061 OpModeDisp=")); Serial.println((int32_t)v); }
  if(SDO_read_any(0x605A,0x00,v,nb)) { Serial.print(F("605A QStopOpt="));  Serial.println((int32_t)v); }
}
void cmd_QSTOP_FIX(){
  NMT_preop(NODE_ID); delay(20);
  SDO_write_u16(0x605A,0x00,2,true);   // 표준적으로 2가 무난
  Serial.println(F("[OK] 605A set to 2"));
}

/* ===== RUNTIME ===== */
enum RunState { IDLE, RUNNING };
RunState state = IDLE;

void handleCommand(String cmd){
  cmd.trim(); cmd.toUpperCase();

  if (cmd=="S"){ if (enable_drive_fast()){ state=RUNNING; Serial.println(F("[OK] ENABLED (PP)")); } else { Serial.println(F("[WARN] Enable failed")); } return; }
  if (cmd=="DIAG"){ cmd_DIAG(); return; }
  if (cmd=="QSTOP"){ cmd_QSTOP_FIX(); return; }

  if (cmd.startsWith("READ ")){
    int sp = cmd.indexOf(' '); if (sp<0) return;
    String rest = cmd.substring(sp+1);
    int sp2 = rest.indexOf(' '); if (sp2<0) return;
    uint16_t idx = (uint16_t) strtoul(rest.substring(0,sp2).c_str(), nullptr, 16);
    uint8_t sub  = (uint8_t)  strtoul(rest.substring(sp2+1).c_str(), nullptr, 16);
    uint32_t v; uint8_t nb;
    if(SDO_read_any(idx, sub, v, nb, 400)){
      Serial.print(F("[READ] ")); Serial.print(rest); Serial.print(F(" = ")); Serial.print(v);
      Serial.print(F(" (bytes=")); Serial.print(nb); Serial.println(F(")"));
    } else {
      Serial.println(F("[ERR] read failed (abort or timeout)"));
    }
    return;
  }

  if (cmd.startsWith("VEL ")){ long v = cmd.substring(4).toInt(); if (v>0){ G_VEL=v; SDO_write_u32(0x6081,0x00,G_VEL,true); SDO_write_u32(0x607F,0x00,G_VEL*5,true);} Serial.print(F("VEL=")); Serial.println(G_VEL); return; }
  if (cmd.startsWith("ACC ")){ long a = cmd.substring(4).toInt(); if (a>0){ G_ACC=a; SDO_write_u32(0x6083,0x00,G_ACC,true);} Serial.print(F("ACC=")); Serial.println(G_ACC); return; }
  if (cmd.startsWith("DEC ")){ long d = cmd.substring(4).toInt(); if (d>0){ G_DEC=d; SDO_write_u32(0x6084,0x00,G_DEC,true);} Serial.print(F("DEC=")); Serial.println(G_DEC); return; }

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

/* ===== Setup/Loop ===== */
void setup(){
  Serial.begin(115200);
  while(!Serial){}                 // 일부 보드용
  pinMode(PIN_INT, INPUT);
  pinMode(10, OUTPUT);             // UNO: SPI master 강제 (CS가 9여도 권장)

  byte ret = CAN_begin_compat(CAN, DEFAULT_BAUD, MCP2515_16MHZ);
  if (ret != CAN_OK){ Serial.println(F("CAN init failed")); while(1){} }

  CAN.setMode(MCP_NORMAL);
  // NMT_start(0x00); // 전체 broadcast Operational: 네 환경에 맞춰 필요 시 켜기

  Serial.println(F("READY: S | DIAG | READ <index hex> <sub hex> | QSTOP | ABS/REL/VEL/ACC/DEC ..."));
}

void loop(){
  static String buf;
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r'){
      if (buf.length()){ handleCommand(buf); buf=""; }
    } else {
      buf += c;
      bool allDigitOrSign = buf.length()>0;
      for(size_t i=0;i<buf.length();++i){ char ch=buf[i]; if(!(isDigit(ch)||ch=='-'||ch=='+')){allDigitOrSign=false;break;} }
      if (allDigitOrSign){ handleCommand(buf); buf=""; }
    }
  }
}
