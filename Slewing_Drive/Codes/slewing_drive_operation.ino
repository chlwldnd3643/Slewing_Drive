#include <SPI.h>
#include <Wire.h>
#include <AS5600.h>
#include "mcp2515_can.h"

/* ===== [1] Library Compatibility Shims ===== */
#ifndef MCP_16MHZ
  #define MCP_16MHZ MCP_16MHz
#endif
#ifndef MCP_NORMAL
  #define MCP_NORMAL 0x00
#endif
#ifndef CAN_OK
  #define CAN_OK (0)
#endif

/* ===== [2] Hardware & Calibration Setup ===== */
const uint8_t PIN_CS = 9;
mcp2515_can CAN(PIN_CS);
AS5600 encoder;

const float DEC_PER_RPM = 17896.14f;
const float INC_PER_DEG = (182.0f * 70.0f * 150.0f) / 62.0f;
const float ENC_RES     = 4096.0f;
const float ENC_DIR     = -1.0f;

float   g_encCorrection = 35.0f / 360.0f;
long    g_offsetPulse   = 0;
uint8_t NODE_ID         = 1;

/* ===== [3] CANopen SDO TX helpers ===== */
bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val) {
  uint8_t d[8] = { 0x23,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF),  (uint8_t)((val >> 8)  & 0xFF),
                   (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF) };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}
bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val) {
  uint8_t d[8] = { 0x2B,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0 };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}
bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val) {
  uint8_t d[8] = { 0x2F,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   val, 0, 0, 0 };
  return CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, d) == CAN_OK;
}

// Read Statusword (0x6041) via SDO. Returns true on success.
bool SDO_read_statusword(uint16_t* out_sw) {
  // SDO Read request: cmd=0x40, idx=0x6041 (LE: 41 60), sub=0
  uint8_t req[8] = { 0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0 };
  if (CAN.sendMsgBuf(0x600 + NODE_ID, 0, 8, req) != CAN_OK) return false;

  // Wait for response (max 50 ms)
  unsigned long t0 = millis();
  while ((millis() - t0) < 50) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned char len = 0;
      unsigned char buf[8];
      CAN.readMsgBuf(&len, buf);
      unsigned long id = CAN.getCanId();
      if ((id & 0x7FF) == (uint16_t)(0x580 + NODE_ID)) {
        // Expected response: cmd=0x4B (2-byte read), idx LE = 41 60, data in buf[4..5]
        if (buf[0] == 0x4B && buf[1] == 0x41 && buf[2] == 0x60) {
          *out_sw = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
          return true;
        }
      }
    }
  }
  return false;
}

void setProfileSpeedRpm(float rpm) {
  if (rpm < 0.1f) rpm = 0.1f;
  uint32_t decValue = (uint32_t)(rpm * DEC_PER_RPM);
  SDO_write_u32(0x6081, 0x00, decValue);
  Serial.print(F(">> Target speed set: ")); Serial.print(rpm);
  Serial.print(F(" RPM (DEC: ")); Serial.print(decValue); Serial.println(F(")"));
}

/* ===== [4] Settle detection - drive's Target_Reached flag ===== */
//
// Strategy:
//   The drive itself reports "target reached" via Statusword bit10.
//   This is far more reliable than watching an external AS5600,
//   because the output shaft moves slowly (170:1 reduction) and
//   small AS5600 changes can be misread as "stopped".
//
//   Statusword bit10 is sometimes set even before motion starts
//   (residual from previous move), so we first wait for it to
//   CLEAR (motion confirmed started), then wait for it to SET
//   (motion confirmed finished).
//
//   bit10:
//     0 = target NOT reached
//     1 = target reached
bool waitForTargetReached(uint16_t timeout_ms = 8000) {
  Serial.print(F(" -> moving"));
  uint16_t sw;

  // Phase 1: wait until bit10 clears (motion has started)
  // Some drives keep bit10=1 from previous move. We wait up to 300ms
  // for it to drop, but skip if drive is already showing not-reached.
  unsigned long t0 = millis();
  while ((millis() - t0) < 300) {
    if (SDO_read_statusword(&sw)) {
      if (!(sw & (1 << 10))) break;     // bit10 = 0, motion in progress
    }
    delay(10);
  }

  // Phase 2: wait for bit10 to set again (target reached)
  t0 = millis();
  uint8_t dot_div = 0;
  while ((millis() - t0) < timeout_ms) {
    if (SDO_read_statusword(&sw)) {
      if (sw & (1 << 10)) {              // target reached!
        Serial.println(F(" [reached]"));
        return true;
      }
    }
    if (++dot_div >= 5) { dot_div = 0; Serial.print('.'); }
    delay(20);
  }
  Serial.println(F(" [timeout]"));
  return false;
}

void printMenu() {
  Serial.println(F("\n======= [ Kinco FD1X5 Controller ] ======="));
  Serial.println(F("  en      : Enable motor (default 1000 RPM)"));
  Serial.println(F("  spd <n> : Set RPM speed (calibrated)"));
  Serial.println(F("  abs <n> : Move to absolute angle (deg)"));
  Serial.println(F("  z       : Zero current AS5600 position"));
  Serial.println(F("  h       : Show this help"));
  Serial.println(F("=========================================="));
}

/* ===== [5] Main ===== */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  encoder.begin(4);

  if (CAN.begin(CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("CAN Init Failed!"));
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  printMenu();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "en") {
      Serial.println(F(">> Enable sequence (default 1000 RPM)..."));
      SDO_write_u16(0x6040, 0x00, 0x0086); delay(50);
      SDO_write_u8 (0x6060, 0x00, 0x01);   delay(50);
      setProfileSpeedRpm(1000.0f);         delay(50);
      SDO_write_u16(0x6040, 0x00, 0x0006); delay(50);
      SDO_write_u16(0x6040, 0x00, 0x0007); delay(50);
      SDO_write_u16(0x6040, 0x00, 0x103F); delay(50);
      Serial.println(F(">> Motor online (1000 RPM)"));
    }
    else if (cmd.startsWith("spd ")) {
      float rpm = cmd.substring(4).toFloat();
      setProfileSpeedRpm(rpm);
    }
    else if (cmd.startsWith("abs ")) {
      float target = cmd.substring(4).toFloat();
      SDO_write_u32(0x607A, 0x00, (long)(target * INC_PER_DEG));
      waitForTargetReached();

      long rel = (encoder.rawAngle() - g_offsetPulse) * ENC_DIR;
      float actual = (rel / ENC_RES) * 360.0f * g_encCorrection;
      Serial.print(F(">> Move complete. Current angle: "));
      Serial.print(actual, 2); Serial.println(F(" deg"));
    }
    else if (cmd == "z") {
      g_offsetPulse = encoder.rawAngle();
      Serial.println(F(">> Zero set"));
    }
    else if (cmd == "h") {
      printMenu();
    }
  }
}
