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

/* ========= Enable (PP mode) ========= */
bool enable_drive_profile_position(){
  uint16_t sw=0;
  Serial.println(F("[S] Step0: Fault Reset (0x6040=0x0080)"));
  SDO_write_u16(0x6040,0x00,0x0080,false); // Fault reset (no-ACK)

  if (USE_605A){
    Serial.println(F("[S] Step1: QuickStop option -> 0 (0x605A=0)"));
    SDO_write_u16(0x605A,0x00,0x0000,false);
  } else {
    Serial.println(F("[S] Step1: (skip 0x605A per config)"));
  }

  Serial.println(F("[S] Step2: Mode=Profile Position (0x6060=1)"));
  SDO_write_u8(0x6060,0x00,1,false); // PP mode

  // ===== 기본 프로파일 파라미터(예시값: 단위=counts/s, counts/s^2) =====
  // 드라이브 단위/스케일 맞춰서 조정하라.
  SDO_write_u32(0x607F,0x00,40000,false); // Max Profile Velocity (상한)
  SDO_write_u32(0x6081,0x00,20000,false); // Profile Velocity (목표속도)
  SDO_write_u32(0x6083,0x00,50000,false); // Profile Acceleration
  SDO_write_u32(0x6084,0x00,50000,false); // Profile Deceleration
  SDO_write_u8 (0x6086,0x00,0,false);     // Motion Profile Type (0=Trap, 벤더 의존)

  Serial.println(F("[S] Step3: CW=0 -> 0x06 -> 0x07 -> 0x0F"));
  SDO_write_u16(0x6040,0x00,0x0000,false);
  SDO_write_u16(0x6040,0x00,0x0006,false);
  SDO_write_u16(0x6040,0x00,0x0007,false);
  SDO_write_u16(0x6040,0x00,0x000F,false);

  if (SDO_read_u16(0x6041,0x00,&sw)) {
    printStatusWord(sw);
    if ((sw & 0x0007) == 0x0007){ Serial.println(F("[S] ✔ Operation Enabled")); return true; }
  } else {
    Serial.println(F("[S] WARN: no SW read"));
  }
  Serial.println(F("[S] ✖ Not enabled. Use SCAN/PING/SNIFF."));
  return false;
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

  // MD (Mode Display)
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

  // MAP/MAPALL/MAPSET (벤더 확장 0x60FE 예시)
  if (cmd=="MAPALL") {
    uint32_t n=0;
    if (SDO_read_u32(0x60FE,0x00,&n)) {
      Serial.print(F("[MAP] count=")); Serial.println(n);
      uint32_t v=0;
      for (uint8_t sub=1; sub<=min((uint32_t)8,n); ++sub){
        if (SDO_read_u32(0x60FE, sub, &v)) {
          Serial.print(F("[MAP] 0x60FE:")); Serial.print(sub); Serial.print(F("=0x")); Serial.println(v,HEX);
        } else {
          Serial.print(F("[MAP] read FAIL @ sub ")); Serial.println(sub);
        }
      }
    } else {
      Serial.println(F("[MAP] 0x60FE:00 read FAIL"));
    }
    return;
  }
  if (cmd.startsWith("MAPSET ")) {
    int sp = cmd.indexOf(' ');
    String rest = cmd.substring(sp+1);
    int sp2 = rest.indexOf(' ');
    uint8_t sub = (uint8_t) rest.substring(0, sp2).toInt();
    uint32_t val = (uint32_t) strtoul(rest.substring(sp2+1).c_str(), NULL, 0);
    if (SDO_write_u32(0x60FE, sub, val, false)) {
      Serial.print(F("[MAPSET] 0x60FE:")); Serial.print(sub); Serial.print(F("=0x")); Serial.println(val,HEX);
    } else {
      Serial.println(F("[MAPSET] write FAIL (RO or vendor-protect)"));
    }
    return;
  }
  if (cmd=="MAP") {
    for (uint8_t sub=1; sub<=6; ++sub){
      uint32_t v=0;
      if (SDO_read_u32(0x60FE, sub, &v)) {
        Serial.print(F("[MAP] 0x60FE:")); Serial.print(sub); Serial.print(F("=0x")); Serial.println(v,HEX);
      } else {
        Serial.print(F("[MAP] read FAIL @ sub ")); Serial.println(sub);
      }
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

  // Quick Stop 옵션
  if (cmd.startsWith("QS ")) {
    uint16_t op = (uint16_t)cmd.substring(3).toInt(); // 0~?
    if (SDO_write_u16(0x605A,0x00,op,false)) { Serial.print(F("[CFG] 0x605A=")); Serial.println(op); }
    else Serial.println(F("[CFG] set 0x605A FAIL"));
    return;
  }

  // 강제 enable 시퀀스
  if (cmd=="SFORCE") {
    SDO_write_u16(0x6040,0x00,0x0080,false); delay(20); // Fault reset
    SDO_write_u16(0x6040,0x00,0x0000,false); delay(10);
    SDO_write_u16(0x6040,0x00,0x0006,false); delay(10);
    SDO_write_u16(0x6040,0x00,0x0007,false); delay(10);
    SDO_write_u16(0x6040,0x00,0x000F,false); delay(10);
    uint16_t sw=0;
    if (SDO_read_u16(0x6041,0x00,&sw)) printStatusWord(sw);
    else Serial.println(F("[SFORCE] SW read FAIL"));
    return;
  }

  // ==== Profile 파라미터 세팅 커맨드 ====
  // VSET <rpm> <cpr>  -> rpm과 엔코더 카운트/회전(cpr)로 0x6081/0x607F 셋팅
  if (cmd.startsWith("VSET ")) {
    long rpm = 0, cpr = 0;
    int sp = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp+1);
    if (sp2 > 0) {
      rpm = cmd.substring(sp+1, sp2).toInt();
      cpr = cmd.substring(sp2+1).toInt();
      long v_counts_per_s = (long)((rpm * (long)cpr) / 60L);
      if (!SDO_write_u32(0x607F, 0x00, (uint32_t)( (v_counts_per_s>0)? v_counts_per_s*2 : -v_counts_per_s*2 ), false))
        Serial.println(F("[VSET] note: no ACK @ 0x607F"));
      if (!SDO_write_u32(0x6081, 0x00, (uint32_t)((v_counts_per_s>=0)? v_counts_per_s : -v_counts_per_s), false))
        Serial.println(F("[VSET] note: no ACK @ 0x6081"));
      Serial.print(F("[VSET] rpm=")); Serial.print(rpm);
      Serial.print(F(", cpr=")); Serial.print(cpr);
      Serial.print(F(" -> 0x6081=")); Serial.println(v_counts_per_s);
    } else {
      Serial.println(F("Usage: VSET <rpm> <cpr>"));
    }
    return;
  }
  // ACC <counts_per_s2>
  if (cmd.startsWith("ACC ")) {
    long a = cmd.substring(4).toInt();
    if (!SDO_write_u32(0x6083, 0x00, (uint32_t)a, false))
      Serial.println(F("[ACC] note: no ACK @ 0x6083"));
    else { Serial.print(F("[ACC] 0x6083=")); Serial.println(a); }
    return;
  }
  // DEC <counts_per_s2>
  if (cmd.startsWith("DEC ")) {
    long d = cmd.substring(4).toInt();
    if (!SDO_write_u32(0x6084, 0x00, (uint32_t)d, false))
      Serial.println(F("[DEC] note: no ACK @ 0x6084"));
    else { Serial.print(F("[DEC] 0x6084=")); Serial.println(d); }
    return;
  }
  // PTYPE <0|1> (0=Trapezoidal, 1=S-curve … 벤더 의존)
  if (cmd.startsWith("PTYPE ")) {
    long t = cmd.substring(6).toInt();
    if (!SDO_write_u8(0x6086, 0x00, (uint8_t)t, false))
      Serial.println(F("[PTYPE] note: no ACK @ 0x6086"));
    else { Serial.print(F("[PTYPE] 0x6086=")); Serial.println(t); }
    return;
  }

  // ==== Enable / Stop ====
  if (cmd=="S"){
    Serial.println(F("[CMD] START requested"));
    if (enable_drive_profile_position()){
      state=RUNNING; Serial.println(F("[CMD] ✔ Drive ENABLED (PP). Ready for targets."));
    } else {
      Serial.println(F("[CMD] ✖ Enable FAILED (try SCAN/PING/SNIFF)"));
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

  // 기본 진단
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
  Serial.println(F("READY. Type: SCAN / PING / S / STOP / ABS / REL / number / VSET / ACC / DEC / PTYPE / SNIFF ON|OFF"));
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
