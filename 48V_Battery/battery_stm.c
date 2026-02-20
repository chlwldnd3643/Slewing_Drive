#include <Arduino.h>
#include "ACANFD_STM32.h"

// =====================
// DALY CAN IDs (Extended 29-bit)
// =====================
static const uint32_t DALY_REQ_ID = 0x18100140;  // Host -> BMS (ext)
static const uint32_t DALY_RSP_ID = 0x18104001;  // BMS  -> Host (ext)

// =====================
// Request rotation
// =====================
static const uint8_t REQ_LIST[4] = {0x90, 0x92, 0x93, 0x98};
static uint8_t reqIdx = 0;

// =====================
// Timing / state
// =====================
static bool systemStarted = false;
static uint32_t lastSendMs = 0;
static uint8_t lastReqId = 0x90;

// =====================
// Forward decl
// =====================
static void waitForStart();
static void stop_control();
static void initCAN_250k_extFilter();

static void requestCAN(uint8_t dataId);
static void receiveAndHandle();

static void dumpFrame(const CANFDMessage &rx);
static void handleByLastRequest(const uint8_t* buf, uint8_t len, uint8_t expectedDataId);

// DALY parsers (original logic)
static void parse_DALY_0x90(const uint8_t* buf, uint8_t len);
static void parse_DALY_0x92(const uint8_t* buf, uint8_t len);
static void parse_DALY_0x93(const uint8_t* buf, uint8_t len);
static void parse_DALY_0x98(const uint8_t* buf, uint8_t len);
static void printFlag(const char* name);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  waitForStart();
  initCAN_250k_extFilter();

  Serial.println("FDCAN init OK (Classic CAN 2.0, 250kbps, Extended ID)");
  Serial.println("Type 's' then Enter to reset.");
}

void loop() {
  // Serial command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "s" || cmd == "S") stop_control();
  }

  // Send request every 1000 ms
  if (millis() - lastSendMs >= 1000) {
    lastReqId = REQ_LIST[reqIdx];
    requestCAN(lastReqId);

    reqIdx = (reqIdx + 1) % 4;
    lastSendMs = millis();
  }

  // Receive + parse
  receiveAndHandle();

  delay(5);
}

// =====================
// Start/Stop
// =====================
static void waitForStart() {
  Serial.println("Press 'S' to start the system.");
  while (!systemStarted) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'S' || c == 's') {
        systemStarted = true;
        Serial.println("System started.");
      }
    }
    delay(50);
  }
}

static void stop_control() {
  Serial.println("System resetting...");
  delay(50);
  NVIC_SystemReset();
}

// =====================
// CAN init (NUCLEO-G431RB + SN65HVD230)
// Pins assumed: PB9=TX, PB8=RX
// =====================
static void initCAN_250k_extFilter() {
  ACANFD_STM32_Settings settings(250 * 1000, DataBitRateFactor::x1);
  settings.mModuleMode = ACANFD_STM32_Settings::DISABLE_FD; // Classic CAN
  settings.mEnableRetransmission = true;

  // Fixed pins (assumed)
  settings.mTxPin = PB_9;  // FDCAN1_TX
  settings.mRxPin = PB_8;  // FDCAN1_RX

  // Filters: accept only DALY_RSP_ID (extended)
  ACANFD_STM32_StandardFilters stdFilters;
  ACANFD_STM32_ExtendedFilters extFilters;
  extFilters.addSingle(DALY_RSP_ID, ACANFD_STM32_FilterAction::FIFO0);

  settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::REJECT;
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  const uint32_t err = fdcan1.beginFD(settings, stdFilters, extFilters);
  if (err != 0) {
    Serial.print("FDCAN init fail: 0x");
    Serial.println(err, HEX);
    while (true) delay(100);
  }
}

// =====================
// Send request
// =====================
static void requestCAN(uint8_t dataId) {
  uint8_t payload[8] = {dataId,0,0,0,0,0,0,0};

  CANFDMessage tx;
  tx.id  = DALY_REQ_ID;
  tx.ext = true;
  tx.rtr = false;
  tx.len = 8;
  memcpy(tx.data, payload, 8);

  if (!fdcan1.tryToSend(tx)) {
    Serial.println("TX fail (mailbox full / bus issue)");
  } else {
    Serial.print("TX req 0x");
    Serial.println(dataId, HEX);
  }
}

// =====================
// Receive and handle
// =====================
static void receiveAndHandle() {
  CANFDMessage rx;
  while (fdcan1.available()) {
    fdcan1.receive(rx);

    if (!rx.ext) continue;
    if (rx.id != DALY_RSP_ID) continue;
    if (rx.len != 8) continue;

    // Always dump raw first (for protocol verification)
    dumpFrame(rx);

    // Parse using last requested dataId (robust enough for single device test)
    handleByLastRequest(rx.data, rx.len, lastReqId);
  }
}

// =====================
// RAW dump
// =====================
static void dumpFrame(const CANFDMessage &rx) {
  Serial.print("RX ID=0x");
  Serial.print(rx.id, HEX);
  Serial.print(" LEN=");
  Serial.print(rx.len);
  Serial.print(" DATA=");
  for (uint8_t i = 0; i < rx.len; i++) {
    if (rx.data[i] < 0x10) Serial.print('0');
    Serial.print(rx.data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// =====================
// Parse dispatch
// NOTE: If your DALY response includes a header byte (e.g., buf[0]=dataId),
// adjust offsets inside parse functions accordingly.
// =====================
static void handleByLastRequest(const uint8_t* buf, uint8_t len, uint8_t expectedDataId) {
  switch (expectedDataId) {
    case 0x90: parse_DALY_0x90(buf, len); break;
    case 0x92: parse_DALY_0x92(buf, len); break;
    case 0x93: parse_DALY_0x93(buf, len); break;
    case 0x98: parse_DALY_0x98(buf, len); break;
    default: break;
  }
}

// =====================
// Parsers (your original logic)
// =====================
static void parse_DALY_0x90(const uint8_t* buf, uint8_t len) {
  if (len < 8) return;

  float totalV_acc  = ((buf[0] << 8) | buf[1]) * 0.1f;
  float totalV_real = ((buf[2] << 8) | buf[3]) * 0.1f;
  float current     = (((buf[4] << 8) | buf[5]) - 30000) * 0.1f;
  float soc         = ((buf[6] << 8) | buf[7]) * 0.1f;

  Serial.print("[0x90] TotalV(acc)=");  Serial.print(totalV_acc);  Serial.print("V  ");
  Serial.print("TotalV(real)=");       Serial.print(totalV_real); Serial.print("V  ");
  Serial.print("Current=");            Serial.print(current);     Serial.print("A  ");
  Serial.print("SOC=");                Serial.print(soc);         Serial.println("%");
}

static void parse_DALY_0x92(const uint8_t* buf, uint8_t len) {
  if (len < 4) return;

  int8_t  t_max    = (int8_t)buf[0] - 40;
  uint8_t t_max_id = buf[1];
  int8_t  t_min    = (int8_t)buf[2] - 40;
  uint8_t t_min_id = buf[3];

  Serial.print("[0x92] Tmax="); Serial.print(t_max); Serial.print("C(#");
  Serial.print(t_max_id);       Serial.print(")  Tmin=");
  Serial.print(t_min);          Serial.print("C(#");
  Serial.print(t_min_id);       Serial.println(")");
}

static void parse_DALY_0x93(const uint8_t* buf, uint8_t len) {
  if (len < 4) return;

  uint8_t cd_state = buf[0];
  bool chgMOS      = buf[1] != 0;
  bool dsgMOS      = buf[2] != 0;
  uint8_t bms_life = buf[3];

  const char* stateStr = (cd_state == 1) ? "충전" : (cd_state == 2) ? "방전" : "정지";

  Serial.print("[0x93] 상태="); Serial.print(stateStr);
  Serial.print("  CHG_MOS=");   Serial.print(chgMOS ? "ON" : "OFF");
  Serial.print("  DSG_MOS=");   Serial.print(dsgMOS ? "ON" : "OFF");
  Serial.print("  Life=");      Serial.println(bms_life);
}

static void printFlag(const char* name) {
  Serial.print(name);
  Serial.print(" | ");
}

static void parse_DALY_0x98(const uint8_t* buf, uint8_t len) {
  if (len < 8) return;

  uint8_t b0 = buf[0];
  if (b0 & (1<<0)) printFlag("셀OV Lv1");
  if (b0 & (1<<1)) printFlag("셀OV Lv2");
  if (b0 & (1<<2)) printFlag("셀UV Lv1");
  if (b0 & (1<<3)) printFlag("셀UV Lv2");
  if (b0 & (1<<4)) printFlag("총전압 High Lv1");
  if (b0 & (1<<5)) printFlag("총전압 High Lv2");
  if (b0 & (1<<6)) printFlag("총전압 Low Lv1");
  if (b0 & (1<<7)) printFlag("총전압 Low Lv2");

  uint8_t errCode = buf[7];
  Serial.print("ERR_CODE=");
  Serial.println(errCode);
}
