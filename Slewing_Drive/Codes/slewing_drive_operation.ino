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

// [Calibrated] Coefficient that compensates for the
// observed error of: input 100 RPM -> output 93.13 RPM.
// 1 RPM = 17896.14 DEC
const float DEC_PER_RPM = 17896.14f;

// Mechanical angle constants
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

// Set Profile Velocity (0x6081) in RPM
void setProfileSpeedRpm(float rpm) {
  if (rpm < 0.1f) rpm = 0.1f;
  uint32_t decValue = (uint32_t)(rpm * DEC_PER_RPM);
  SDO_write_u32(0x6081, 0x00, decValue);
  Serial.print(F(">> Target speed set: ")); Serial.print(rpm);
  Serial.print(F(" RPM (DEC: ")); Serial.print(decValue); Serial.println(F(")"));
}

/* ===== [4] Motion logic & UI ===== */
void waitForStop() {
  Serial.print(F(" -> moving"));
  delay(500);
  long prev_p = encoder.rawAngle();
  unsigned long startT = millis();
  while (millis() - startT < 15000) {
    delay(200);
    long curr_p = encoder.rawAngle();
    if (abs(curr_p - prev_p) < 5) {       // motor settled
      delay(300);
      Serial.println(F(" [stopped]"));
      return;
    }
    prev_p = curr_p;
    Serial.print(".");
  }
  Serial.println(F(" [timeout]"));
}

void printMenu() {
  Serial.println(F("\n======= [ Kinco FD1X5 Controller ] ======="));
  Serial.println(F("  en      : Enable motor (06->07->103F + 1000 RPM default)"));
  Serial.println(F("  spd <n> : Set RPM speed (calibrated)"));
  Serial.println(F("  abs <n> : Move to absolute angle in degrees"));
  Serial.println(F("  z       : Set current position as zero"));
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
      Serial.println(F(">> Starting enable sequence (default 1000 RPM)..."));
      SDO_write_u16(0x6040, 0x00, 0x0086); delay(50);   // Fault Reset
      SDO_write_u8 (0x6060, 0x00, 0x01);   delay(50);   // Position Mode

      // Inject default speed = 1000 RPM
      setProfileSpeedRpm(1000.0f);         delay(50);

      // DS402 standard enable sequence
      SDO_write_u16(0x6040, 0x00, 0x0006); delay(50);   // Ready to switch on
      SDO_write_u16(0x6040, 0x00, 0x0007); delay(50);   // Switched on
      SDO_write_u16(0x6040, 0x00, 0x103F); delay(50);   // Enable + Immediate Change
      Serial.println(F(">> Motor online (default 1000 RPM applied)"));
    }
    else if (cmd.startsWith("spd ")) {
      float rpm = cmd.substring(4).toFloat();
      setProfileSpeedRpm(rpm);
    }
    else if (cmd.startsWith("abs ")) {
      float target = cmd.substring(4).toFloat();
      // Write target position (0x607A)
      SDO_write_u32(0x607A, 0x00, (long)(target * INC_PER_DEG));
      waitForStop();

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
