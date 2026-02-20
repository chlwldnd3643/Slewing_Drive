#include <Arduino.h>
#include "ACANFD_STM32.h"

// =====================
// HW (NUCLEO-G431RB + SN65HVD230)
// =====================
static constexpr PinName FDCAN_TX_PIN = PB_9;  // FDCAN1_TX
static constexpr PinName FDCAN_RX_PIN = PB_8;  // FDCAN1_RX

// =====================
// VESC CAN settings
// =====================
// VESC CAN은 Extended ID 사용 (29-bit)
// EID = (CommandID << 8) | VESC_ID  :contentReference[oaicite:5]{index=5}
static constexpr uint8_t  VESC_ID = 1;          // VESC Tool에서 동일하게 설정
static constexpr uint32_t VESC_CAN_BAUD = 500000; // 필요 시 250000으로 변경

// Timeout 방지를 위해 50Hz 권장 :contentReference[oaicite:6]{index=6}
static constexpr uint32_t SEND_PERIOD_MS = 20;

// =====================
// VESC simple command IDs (single-frame)
// =====================
// :contentReference[oaicite:7]{index=7}
enum VescCanCmd : uint8_t {
  CAN_PACKET_SET_DUTY          = 0,
  CAN_PACKET_SET_CURRENT       = 1,
  CAN_PACKET_SET_CURRENT_BRAKE = 2,
  CAN_PACKET_SET_RPM           = 3,
  CAN_PACKET_SET_POS           = 4,
  CAN_PACKET_STATUS            = 9,
  CAN_PACKET_STATUS_5          = 27,
};

// =====================
// Command state (what to keep sending)
// =====================
enum class ControlMode : uint8_t { DUTY, CURRENT, RPM, BRAKE, STOP };

static ControlMode gMode = ControlMode::STOP;
static float gDuty = 0.0f;      // -1.0 ~ 1.0
static float gCurrentA = 0.0f;  // A
static float gBrakeA = 0.0f;    // A
static int32_t gRpm = 0;        // rpm

static uint32_t gLastSendMs = 0;

// ---------------------
// helpers
// ---------------------
static inline uint32_t makeEID(uint8_t cmd, uint8_t vescId) {
  return (uint32_t(cmd) << 8) | uint32_t(vescId);
}

// big-endian int32 pack
static inline void pack_i32_be(uint8_t *b, int32_t v) {
  b[0] = (uint8_t)((v >> 24) & 0xFF);
  b[1] = (uint8_t)((v >> 16) & 0xFF);
  b[2] = (uint8_t)((v >>  8) & 0xFF);
  b[3] = (uint8_t)((v >>  0) & 0xFF);
}

static inline int32_t unpack_i32_be(const uint8_t *b) {
  return (int32_t)((uint32_t)b[0] << 24 |
                   (uint32_t)b[1] << 16 |
                   (uint32_t)b[2] <<  8 |
                   (uint32_t)b[3]);
}

static inline int16_t unpack_i16_be(const uint8_t *b) {
  return (int16_t)((uint16_t)b[0] << 8 | (uint16_t)b[1]);
}

// Send 4-byte simple command (int32 big-endian) :contentReference[oaicite:8]{index=8}
static void vescSendSimple4(uint8_t cmd, int32_t value_i32) {
  CANFDMessage tx;
  tx.id  = makeEID(cmd, VESC_ID);
  tx.ext = true;
  tx.rtr = false;
  tx.len = 4;
  pack_i32_be(tx.data, value_i32);

  if (!fdcan1.tryToSend(tx)) {
    Serial.println("VESC TX fail");
  }
}

// scaling rules :contentReference[oaicite:9]{index=9}
static void vescSetDuty(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  int32_t v = (int32_t)(duty * 100000.0f);   // scale 100000
  vescSendSimple4(CAN_PACKET_SET_DUTY, v);
}

static void vescSetCurrent(float currentA) {
  int32_t v = (int32_t)(currentA * 1000.0f); // scale 1000
  vescSendSimple4(CAN_PACKET_SET_CURRENT, v);
}

static void vescSetBrake(float brakeA) {
  int32_t v = (int32_t)(brakeA * 1000.0f);   // scale 1000
  vescSendSimple4(CAN_PACKET_SET_CURRENT_BRAKE, v);
}

static void vescSetRpm(int32_t rpm) {
  vescSendSimple4(CAN_PACKET_SET_RPM, rpm);  // scale 1
}

// ---------------------
// CAN init
// ---------------------
static void initCAN() {
  ACANFD_STM32_Settings settings(VESC_CAN_BAUD, DataBitRateFactor::x1);
  settings.mModuleMode = ACANFD_STM32_Settings::DISABLE_FD; // Classic CAN
  settings.mEnableRetransmission = true;

  settings.mTxPin = FDCAN_TX_PIN;
  settings.mRxPin = FDCAN_RX_PIN;

  // 필터: 일단 "모든 Extended 프레임" 수신 허용(디버그/테스트에 유리)
  ACANFD_STM32_StandardFilters stdFilters;
  ACANFD_STM32_ExtendedFilters extFilters;

  settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::FIFO0;
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  const uint32_t err = fdcan1.beginFD(settings, stdFilters, extFilters);
  if (err != 0) {
    Serial.print("FDCAN init fail: 0x");
    Serial.println(err, HEX);
    while (true) delay(100);
  }
}

// ---------------------
// Status decode (optional)
// STATUS and STATUS_5 formats :contentReference[oaicite:10]{index=10}
// ---------------------
static void handleVescStatus(const CANFDMessage &rx) {
  const uint8_t cmd = (uint8_t)((rx.id >> 8) & 0xFF);
  const uint8_t srcId = (uint8_t)(rx.id & 0xFF);

  if (!rx.ext || rx.len != 8) return;

  if (cmd == CAN_PACKET_STATUS) {
    // B0-3 ERPM (scale 1), B4-5 Current (scale 10), B6-7 Duty (scale 1000) :contentReference[oaicite:11]{index=11}
    int32_t erpm = unpack_i32_be(&rx.data[0]);
    int16_t cur10 = unpack_i16_be(&rx.data[4]);
    int16_t duty1000 = unpack_i16_be(&rx.data[6]);
    float curA = cur10 / 10.0f;
    float duty = duty1000 / 1000.0f;

    Serial.print("[VESC "); Serial.print(srcId);
    Serial.print("] STATUS ERPM="); Serial.print(erpm);
    Serial.print("  I="); Serial.print(curA);
    Serial.print("A  Duty="); Serial.println(duty);
  }

  if (cmd == CAN_PACKET_STATUS_5) {
    // B0-3 Tachometer (scale 6), B4-5 Vin (scale 10) :contentReference[oaicite:12]{index=12}
    int32_t tach = unpack_i32_be(&rx.data[0]);
    int16_t vin10 = unpack_i16_be(&rx.data[4]);
    float vin = vin10 / 10.0f;

    Serial.print("[VESC "); Serial.print(srcId);
    Serial.print("] STATUS_5 Tacho="); Serial.print(tach);
    Serial.print("  Vin="); Serial.print(vin);
    Serial.println("V");
  }
}

// ---------------------
// Serial command parser
// ---------------------
static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  d <duty>     e.g. d 0.10   (-1.0 ~ 1.0)");
  Serial.println("  i <A>        e.g. i 20     (motor current)");
  Serial.println("  r <rpm>      e.g. r 1500");
  Serial.println("  b <A>        e.g. b 5      (brake current)");
  Serial.println("  z            stop output");
}

static void parseSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "h" || line == "help") {
    printHelp();
    return;
  }

  if (line == "z") {
    gMode = ControlMode::STOP;
    gDuty = 0; gCurrentA = 0; gBrakeA = 0; gRpm = 0;
    Serial.println("STOP");
    return;
  }

  char c = line.charAt(0);
  float f = 0;
  int32_t n = 0;

  if (c == 'd') {
    f = line.substring(1).toFloat();
    gMode = ControlMode::DUTY;
    gDuty = f;
    Serial.print("Set DUTY="); Serial.println(gDuty);
  } else if (c == 'i') {
    f = line.substring(1).toFloat();
    gMode = ControlMode::CURRENT;
    gCurrentA = f;
    Serial.print("Set CURRENT="); Serial.print(gCurrentA); Serial.println("A");
  } else if (c == 'b') {
    f = line.substring(1).toFloat();
    gMode = ControlMode::BRAKE;
    gBrakeA = f;
    Serial.print("Set BRAKE="); Serial.print(gBrakeA); Serial.println("A");
  } else if (c == 'r') {
    n = (int32_t)line.substring(1).toInt();
    gMode = ControlMode::RPM;
    gRpm = n;
    Serial.print("Set RPM="); Serial.println(gRpm);
  } else {
    Serial.println("Unknown. type 'h'");
  }
}

// ---------------------
// Arduino setup/loop
// ---------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  initCAN();
  printHelp();

  Serial.print("CAN baud = ");
  Serial.println(VESC_CAN_BAUD);
  Serial.print("VESC_ID = ");
  Serial.println(VESC_ID);
}

void loop() {
  parseSerial();

  // 50Hz keep-sending to avoid CAN timeout :contentReference[oaicite:13]{index=13}
  const uint32_t now = millis();
  if (now - gLastSendMs >= SEND_PERIOD_MS) {
    switch (gMode) {
      case ControlMode::DUTY:    vescSetDuty(gDuty); break;
      case ControlMode::CURRENT: vescSetCurrent(gCurrentA); break;
      case ControlMode::RPM:     vescSetRpm(gRpm); break;
      case ControlMode::BRAKE:   vescSetBrake(gBrakeA); break;
      case ControlMode::STOP:
      default:
        // STOP은 안전하게 0 duty를 반복 송신
        vescSetDuty(0.0f);
        break;
    }
    gLastSendMs = now;
  }

  // Receive any VESC status frames and print
  CANFDMessage rx;
  while (fdcan1.available()) {
    fdcan1.receive(rx);
    // Extended ID에서 cmd/id를 풀어보고 status면 출력
    handleVescStatus(rx);
  }
}
