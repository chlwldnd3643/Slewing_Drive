#include <SPI.h>
#include "mcp_can.h"

/* ========= USER CONFIG ========= */
uint8_t  NODE_ID         = 1;        // SCAN으로 찾은 ID로 바꿔라
const bool MCP2515_16MHZ = true;
const uint8_t PIN_CS     = 9;
const uint8_t PIN_INT    = 2;

#define USE_605A 0                  // 일부 드라이브는 0x605A 미지원 → 0 유지
#define DEFAULT_BAUD CAN_500KBPS    // 필요 시 CAN_250KBPS / CAN_500KBPS 등 시도

/* ========= MCP_CAN PLUS (getCanId 지원) ========= */
class MCP_CAN_PLUS : public MCP_CAN {
public:
  using MCP_CAN::MCP_CAN;
  volatile unsigned long last_id = 0;
  byte readMsgBuf_plus(unsigned char* len, unsigned char buf[8]) {
    unsigned long id = 0;
    byte st = MCP_CAN::readMsgBuf(&id, len, buf);
    if (st == CAN_OK) last_id = id;
    return st;
  }
  unsigned long getCanId() const { return last_id; }
};
MCP_CAN_PLUS CAN(PIN_CS);

/* ========= Dynamic COB-IDs ========= */
inline uint32_t COB_NMT()    { return 0x000; }
inline uint32_t COB_SDO_TX() { return 0x600 + NODE_ID; }
inline uint32_t COB_SDO_RX() { return 0x580 + NODE_ID; }

/* ========= Utils ========= */
bool sendCAN(uint32_t id, const uint8_t* data, uint8_t len = 8){
  byte st = CAN.sendMsgBuf(id, 0, len, (unsigned char*)data);
  return (st == CAN_OK);
}
bool waitSDO(uint16_t index, uint8_t sub, uint32_t timeout_ms = 200){
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms){
    if (CAN.checkReceive() == CAN_MSGAVAIL){
      unsigned char len=0, buf[8]={0};
      if (CAN.readMsgBuf_plus(&len, buf) == CAN_OK){
        unsigned long rxId = CAN.getCanId();
        if (rxId == COB_SDO_RX() && len >= 8){
          uint16_t idx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
          if (idx == index && buf[3]==sub){
            if (buf[0]==0x60) return true;  // SDO ack
            if (buf[0]==0x80) return false; // SDO abort
          }
        } 
      }
    }
  }
  return false;
}

/* ========= SDO helpers ========= */
bool SDO_write_u8 (uint16_t idx, uint8_t sub, uint8_t val, bool mustAck=true){
  uint8_t d[8]={0x2F,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,val,0,0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return mustAck ? waitSDO(idx, sub) : true;
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val, bool mustAck=true){
  uint8_t d[8]={0x2B,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,(uint8_t)(val&0xFF),(uint8_t)(val>>8),0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return mustAck ? waitSDO(idx, sub) : true;
}
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val, bool mustAck=true){
  uint8_t d[8]={0x23,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,(uint8_t)(val&0xFF),(uint8_t)((val>>8)&0xFF),(uint8_t)((val>>16)&0xFF),(uint8_t)((val>>24)&0xFF)};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  return mustAck ? waitSDO(idx, sub) : true;
}
bool SDO_read_u16(uint16_t idx, uint8_t sub, uint16_t *out){
  uint8_t d[8]={0x40,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,0,0,0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  uint32_t t0=millis();
  while(millis()-t0<300){
    if(CAN.checkReceive()==CAN_MSGAVAIL){
      unsigned char len=0, buf[8]={0};
      if(CAN.readMsgBuf_plus(&len,buf)==CAN_OK){
        unsigned long rxId=CAN.getCanId();
        if(rxId==COB_SDO_RX() && len>=8 && buf[1]==(uint8_t)(idx&0xFF) && buf[2]==(uint8_t)(idx>>8) && buf[3]==sub){
          if(buf[0]==0x4B || buf[0]==0x4F){ *out=(uint16_t)buf[4] | ((uint16_t)buf[5]<<8); return true; }
          if(buf[0]==0x80) return false;
        }
      }
    }
  }
  return false;
}
bool SDO_read_u32(uint16_t idx, uint8_t sub, uint32_t *out){
  uint8_t d[8]={0x40,(uint8_t)(idx&0xFF),(uint8_t)(idx>>8),sub,0,0,0,0};
  if(!sendCAN(COB_SDO_TX(), d)) return false;
  uint32_t t0=millis();
  while(millis()-t0<300){
    if(CAN.checkReceive()==CAN_MSGAVAIL){
      unsigned char len=0, buf[8]={0};
      if(CAN.readMsgBuf_plus(&len,buf)==CAN_OK){
        unsigned long rxId=CAN.getCanId();
        if(rxId==COB_SDO_RX() && len>=8 && buf[1]==(uint8_t)(idx&0xFF) && buf[2]==(uint8_t)(idx>>8) && buf[3]==sub){
          if(buf[0]==0x43){ *out=(uint32_t)buf[4] | ((uint32_t)buf[5]<<8) | ((uint32_t)buf[6]<<16) | ((uint32_t)buf[7]<<24); return true; }
          if(buf[0]==0x80) return false;
        }
      }
    }
  }
  return false;
}

/* ========= Statusword decoder ========= */
void printStatusWord(uint16_t sw){
  Serial.print(F("[SW] 0x")); Serial.println(sw, HEX);
  Serial.print(F("     "));
  Serial.print((sw&0x0001)?F("ReadyToSwitchOn "):F(""));
  Serial.print((sw&0x0002)?F("SwitchedOn "):F(""));
  Serial.print((sw&0x0004)?F("OperationEnabled "):F(""));
  Serial.print((sw&0x0008)?F("Fault "):F(""));
  Serial.print((sw&0x0010)?F("VoltageEnabled "):F(""));
  Serial.print((sw&0x0020)?F("QuickStopActive "):F(""));
  Serial.print((sw&0x0040)?F("SwitchOnDisabled "):F(""));
  Serial.print((sw&0x0080)?F("Warning "):F(""));
  Serial.print((sw&0x0800)?F("Remote "):F(""));
  Serial.println();
}

/* ========= NMT ========= */
void NMT_cmd(uint8_t cmd, uint8_t node){ uint8_t d[2]={cmd,node}; sendCAN(COB_NMT(), d, 2); }
void NMT_start(uint8_t node){ NMT_cmd(0x01, node); }
void NMT_preop(uint8_t node){ NMT_cmd(0x80, node); }

/* ========= FAST ENABLE (0x0086 → 0x103F) ========= */
bool enable_drive_fast(){
  Serial.println(F("[FAST] Mode=PP (0x6060=1) & profile params"));
  // 모드/프로파일 (ACK 없이 빠르게) — 필요 시 단위 맞춰 조정
  SDO_write_u8 (0x6060,0x00,1,false);     // Profile Position
  if (USE_605A) SDO_write_u16(0x605A,0x00,0x0000,false); // QuickStop option (optional)
    // 예: 목표속도 30rpm, CPR=65536 → 32768 counts/s
  SDO_write_u32(0x6081,0x00, 10000000, true);  // Profile Velocity, 17920 = 1rpm
  SDO_write_u32(0x607F,0x00, 50000000, true);  // Max Profile Velocity
  SDO_write_u32(0x6083,0x00, 366200, true);  // Accel (3662=1rps/s)
  SDO_write_u32(0x6084,0x00, 366200, true);  // Decel
  SDO_write_u8 (0x6086,0x00, 0, true);      // Trapezoidal


  Serial.println(F("[FAST] CW=0x0086 (fault reset + shutdown-ish)"));
  SDO_write_u16(0x6040,0x00,0x0086,false);
  delay(10);

  Serial.println(F("[FAST] CW=0x103F (Enable+NewSP+ChangeImmed)"));
  bool ok = SDO_write_u16(0x6040,0x00,0x103F,false);
  delay(10);

  uint16_t sw=0;
  if (SDO_read_u16(0x6041,0x00,&sw)) {
    printStatusWord(sw);
  } else {
    Serial.println(F("[FAST] WARN: SW read fail"));
  }
  return ok;
}

/* ========= Diagnostics ========= */
void cmdPING(){
  Serial.print(F("[PING] Node ")); Serial.println(NODE_ID);
  uint32_t dev=0;
  if (SDO_read_u32(0x1000,0x00,&dev)) { Serial.print(F("[PING] 0x1000 DeviceType=0x")); Serial.println(dev,HEX); }
  else { Serial.println(F("[PING] no SDO response (ID/baud/wiring/protocol)")); }
}
void cmdSCAN(){
  Serial.println(F("[SCAN] probing Node-ID 1..8 (SDO 0x1000)"));
  uint8_t found=0;
  for(uint8_t nid=1;nid<=8;++nid){
    uint8_t d[8]={0x40,0x00,0x10,0x00,0,0,0,0};
    if(!sendCAN(0x600+nid,d)) continue;
    uint32_t t0=millis();
    while(millis()-t0<120){
      if(CAN.checkReceive()==CAN_MSGAVAIL){
        unsigned char len=0,buf[8]={0};
        if(CAN.readMsgBuf_plus(&len,buf)==CAN_OK){
          unsigned long rxId=CAN.getCanId();
          if(rxId==(0x580+nid) && len>=8 && buf[1]==0x00 && buf[2]==0x10 && buf[3]==0x00){
            if(buf[0]==0x43){
              uint32_t dev=(uint32_t)buf[4] | ((uint32_t)buf[5]<<8) | ((uint32_t)buf[6]<<16) | ((uint32_t)buf[7]<<24);
              Serial.print(F("[SCAN] Node ")); Serial.print(nid); Serial.print(F(" -> DevType 0x")); Serial.println(dev,HEX);
              found=nid;
            } else if(buf[0]==0x80){
              Serial.print(F("[SCAN] Node ")); Serial.print(nid); Serial.println(F(" -> SDO abort (alive)"));
              found=nid;
            }
          }
        }
      }
    }
  }
  if(!found) Serial.println(F("[SCAN] no responders (maybe not CANopen or wrong baud/wiring)."));
  else       Serial.println(F("[SCAN] done. Set NODE_ID accordingly."));
}

/* ========= Sniffer ========= */
bool sniff_on = false;
void do_sniff_once(){
  unsigned char len=0, buf[8]={0};
  if (CAN.checkReceive()==CAN_MSGAVAIL && CAN.readMsgBuf_plus(&len,buf)==CAN_OK){
    unsigned long id = CAN.getCanId();
    bool looks_ext = (id > 0x7FF); // heuristic
    Serial.print(F("[RX] ID=0x")); Serial.print(id,HEX);
    Serial.print(looks_ext?F(" (EXT) "):F(" (STD) "));
    Serial.print(F("LEN=")); Serial.print(len);
    Serial.print(F(" DATA="));
    for(int i=0;i<len;i++){ if(buf[i]<16) Serial.print('0'); Serial.print(buf[i],HEX); Serial.print(' '); }
    Serial.println();
  }
}

/* ========= Commands ========= */
enum RunState { IDLE, RUNNING };
RunState state = IDLE;

void handleCommand(String cmd){
  cmd.trim(); cmd.toUpperCase();

  if(cmd=="SNIFF ON"){ sniff_on=true;  Serial.println(F("[MODE] Sniff=ON")); return; }
  if(cmd=="SNIFF OFF"){ sniff_on=false; Serial.println(F("[MODE] Sniff=OFF")); return; }

  if (cmd=="SW") { 
    uint16_t sw=0; 
    if (SDO_read_u16(0x6041,0x00,&sw)) printStatusWord(sw); 
    else Serial.println(F("[SW] read FAIL"));
    return;
  }

  if (cmd=="MD") {
    uint32_t v=0;
    if (SDO_read_u32(0x6061,0x00,&v)) {
      uint8_t md = (uint8_t)(v & 0xFF);
      Serial.print(F("[MD] Mode Display 0x6061="));
      Serial.println(md);
    } else {
      Serial.println(F("[MD] read FAIL"));
    }
    return;
  }

  // 에러 확인
  if (cmd=="ERR") {
    uint32_t ec=0; uint16_t er=0;
    if (SDO_read_u32(0x603F,0x00,&ec)) { Serial.print(F("[ERR] 0x603F=")); Serial.println(ec,HEX); }
    else Serial.println(F("[ERR] 0x603F read FAIL"));
    if (SDO_read_u16(0x1001,0x00,&er)) { Serial.print(F("[ERR] 0x1001=")); Serial.println(er,HEX); }
    else Serial.println(F("[ERR] 0x1001 read FAIL"));
    return;
  }

  if (cmd=="S"){  // <-- 빠른 Enable로 변경
    Serial.println(F("[CMD] FAST START (0x0086 -> 0x103F)"));
    if (enable_drive_fast()){
      state=RUNNING; Serial.println(F("[CMD] ✔ Drive FAST-ENABLED (PP). Ready for targets."));
    } else {
      Serial.println(F("[CMD] ✖ FAST enable may have failed. Try SCAN/PING/standard sequence."));
    }
    return;
  }

  if (cmd=="STOP"){
    Serial.println(F("[CMD] STOP -> QuickStop+Shutdown"));
    SDO_write_u16(0x6040,0x00,0x000B,false); delay(10); // Quick stop
    SDO_write_u16(0x6040,0x00,0x0006,false);           // Shutdown
    state=IDLE;
    return;
  }

  if (cmd=="PING"){ cmdPING(); return; }
  if (cmd=="SCAN"){ cmdSCAN(); return; }

  // ==== 이동 명령 ====
  // 숫자만 → ABS 이동
  bool allDigitOrSign = cmd.length()>0;
  for(size_t i=0;i<cmd.length();++i){ char ch=cmd[i]; if(!(isDigit(ch)||ch=='-'||ch=='+')){allDigitOrSign=false;break;} }
  if(allDigitOrSign){
    if (state!=RUNNING){ Serial.println(F("Not RUNNING. Send 'S' first.")); return; }
    long target = cmd.toInt();
    if(!SDO_write_u32(0x607A,0x00,(uint32_t)target,false)) Serial.println(F("[MOVE] note: no ACK @ target"));
    uint16_t cw=0x001F; if(!SDO_write_u16(0x6040,0x00,cw,false)) Serial.println(F("[MOVE] note: no ACK @ trigger"));
    cw&=~(1<<4); SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("ABS Move -> ")); Serial.println(target);
    return;
  }
  if (cmd.startsWith("ABS ")){
    if (state!=RUNNING){ Serial.println(F("Not RUNNING. Send 'S' first.")); return; }
    long t=cmd.substring(4).toInt();
    if(!SDO_write_u32(0x607A,0x00,(uint32_t)t,false)) Serial.println(F("[MOVE] note: no ACK @ target"));
    uint16_t cw=0x001F; if(!SDO_write_u16(0x6040,0x00,cw,false)) Serial.println(F("[MOVE] note: no ACK @ trigger"));
    cw&=~(1<<4); SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("ABS Move -> ")); Serial.println(t);
    return;
  }
  if (cmd.startsWith("REL ")){
    if (state!=RUNNING){ Serial.println(F("Not RUNNING. Send 'S' first.")); return; }
    long d=cmd.substring(4).toInt();
    if(!SDO_write_u32(0x607A,0x00,(uint32_t)d,false)) Serial.println(F("[MOVE] note: no ACK @ target"));
    uint16_t cw=0x005F; if(!SDO_write_u16(0x6040,0x00,cw,false)) Serial.println(F("[MOVE] note: no ACK @ trigger")); // relative bit 포함
    cw&=~(1<<4); SDO_write_u16(0x6040,0x00,cw,false);
    Serial.print(F("REL Move -> ")); Serial.println(d);
    return;
  }

  if(cmd.length()){ Serial.print(F("[CMD] Unknown: ")); Serial.println(cmd); }
}

/* ========= Setup ========= */
void setup(){
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT);

  byte ret;
  if (MCP2515_16MHZ) ret = CAN.begin(MCP_ANY, DEFAULT_BAUD, MCP_16MHZ); // STD+EXT 수신
  else               ret = CAN.begin(MCP_ANY, DEFAULT_BAUD, MCP_8MHZ);
  if (ret != CAN_OK){ Serial.println(F("CAN init failed")); while(1){} }

  CAN.setMode(MCP_LISTENONLY);
  Serial.println(F("Entering Configuration Mode Successful!"));
  Serial.println(F("Setting Baudrate Successful!"));
  Serial.println(F("CAN init OK"));

  // 부팅 스니프 3초 (표준/확장 감지)
  Serial.println(F("SNIFF: 3s listen-only..."));
  uint32_t t0=millis(); uint16_t std_cnt=0, ext_cnt=0;
  while(millis()-t0<3000){
    unsigned char len=0, buf[8]={0};
    if(CAN.checkReceive()==CAN_MSGAVAIL && CAN.readMsgBuf_plus(&len,buf)==CAN_OK){
      unsigned long id = CAN.getCanId();
      if (id > 0x7FF) ext_cnt++; else std_cnt++;
    }
  }
  Serial.print(F("SNIFF summary: STD=")); Serial.print(std_cnt);
  Serial.print(F(" EXT=")); Serial.println(ext_cnt);
  if (ext_cnt > 0 && std_cnt==0) Serial.println(F(">> Looks like EXT-only (NMEA2000/J1939?)"));
  else if (ext_cnt > 0)          Serial.println(F(">> Mixed frames present"));
  else                           Serial.println(F(">> No traffic or STD-only"));

  CAN.setMode(MCP_NORMAL);
  NMT_start(0x00); // all nodes start (CANopen일 때 의미 있음)
  Serial.println(F("READY. Type: SCAN / PING / S / STOP / ABS <pos> / REL <inc> / number / ERR / MD / SNIFF ON|OFF"));
}

/* ========= Loop ========= */
void loop(){
  static String buf;

  if (sniff_on) do_sniff_once();

  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r'){
      if (buf.length()){ handleCommand(buf); buf=""; }
    } else {
      buf += c;
      // 숫자 즉시 실행
      bool allDigitOrSign = buf.length()>0;
      for(size_t i=0;i<buf.length();++i){ char ch=buf[i]; if(!(isDigit(ch)||ch=='-'||ch=='+')){allDigitOrSign=false;break;} }
      if (allDigitOrSign){ handleCommand(buf); buf=""; }
    }
  }
}
