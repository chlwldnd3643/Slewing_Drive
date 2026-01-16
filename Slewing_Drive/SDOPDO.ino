#include <Arduino.h>
/* * STM32 전용: Software Interlock & PDO State Machine 버전 (V10)
 * KR 자율운항선박 지침: 브레이크 체결 시 모터 토크 인가 방지 (Interlock)
 * Controlword(0x6040) 조작을 통해 하드웨어 레벨의 안전을 보장합니다.
 */
#include <SPI.h>
#include "mcp2515_can.h"

/* ===== 하드웨어 핀 설정 ===== */
#define CAN1_CS  PA4  
#define CAN2_CS  PA15 
#define CAN_INT  PB0

#define PIN_POWER_SENSE     PB12 
#define PIN_BRAKE_RELAY     PA1  

mcp2515_can CAN1(CAN1_CS);
mcp2515_can CAN2(CAN2_CS);

/* ===== 제어 및 모니터링 파라미터 ===== */
const uint32_t CONTROL_PERIOD_US = 20000;   // 20ms
const uint32_t TIMEOUT_MS = 200;            
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;  
const uint8_t  BUS_ERROR_THRESHOLD = 96;

const long  ERROR_LIMIT_1_DEG = 27777;      
const uint16_t MAX_ERROR_TICKS = 50;        

const float MAX_VEL   = 8000.0f;    
const float MAX_ACCEL = 400.0f;     
const float STOP_MARGIN = 10.0f;    

/* ===== 시스템 상태 (volatile) ===== */
volatile long  steer_cmd = 0;       
volatile float profile_pos = 0;     
volatile float current_vel = 0;     
volatile long  actual_pos = 0;      

volatile bool  is_emergency = false;
volatile bool  is_power_lost = false;       
volatile bool  is_steering_fault = false;   
volatile uint16_t error_persistence_cnt = 0; 
volatile uint32_t last_steer_tick = 0;

// [신규] 드라이버 제어 상태 변수
volatile uint16_t control_word = 0x0006;    // 초기값: Disable Voltage (Safe)

// Dual CAN 상태
volatile uint32_t last_hb_can1 = 0, last_hb_can2 = 0;
volatile bool can1_healthy = false, can2_healthy = false;
volatile uint8_t can1_tec = 0, can1_rec = 0, can2_tec = 0, can2_rec = 0;
volatile uint8_t driver_state = 0;

/* ===== CANopen ID ===== */
#define NODE_ID 1
#define COB_ID_SYNC      0x80
#define COB_ID_RPDO1     (0x200 + NODE_ID) // Mapping: 6040(16bit) + 607A(32bit)
#define COB_ID_TPDO1     (0x180 + NODE_ID) 
#define COB_ID_HEARTBEAT (0x700 + NODE_ID)

HardwareTimer *MyTim;

/* ===== 전원 상실 인터럽트 서비스 루틴 (EXTI) ===== */
void PowerLoss_ISR() {
    digitalWrite(PIN_BRAKE_RELAY, LOW); 
    is_power_lost = true;
    is_emergency = true;
}

/* ===== [핵심] 브레이크 및 소프트웨어 인터락 로직 ===== */
void Update_Brake_And_Interlock() {
    // 1. 물리 브레이크 제어
    if (is_emergency || is_power_lost || is_steering_fault) {
        digitalWrite(PIN_BRAKE_RELAY, LOW); // 브레이크 잠금
        // 2. 소프트웨어 인터락: 브레이크가 잠기면 토크 해제 (Disable Operation)
        control_word = 0x0006; 
    } else {
        digitalWrite(PIN_BRAKE_RELAY, HIGH); // 브레이크 해제
        // 3. 브레이크가 풀린 상태에서만 운전 가능 (Enable Operation)
        control_word = 0x103F; 
    }
}

/* ===== 조향 오차 감시 로직 ===== */
void Monitor_Steering_Performance() {
    long current_error = abs((long)profile_pos - actual_pos);
    if (current_error > ERROR_LIMIT_1_DEG) {
        error_persistence_cnt++;
        if (error_persistence_cnt >= MAX_ERROR_TICKS) is_steering_fault = true;
    } else {
        if (error_persistence_cnt > 0) error_persistence_cnt--;
        if (error_persistence_cnt == 0) is_steering_fault = false;
    }
}

/* ===== Trapezoidal Motion Profile 연산 ===== */
void Update_Motion_Profile() {
    // [인터락] 브레이크가 잠겨있거나 토크가 해제된 상태면 프로필 업데이트 중단
    if (control_word != 0x103F) {
        current_vel = 0;
        return; 
    }

    float target = (float)steer_cmd;
    float distance = target - profile_pos;
    float abs_dist = abs(distance);

    if (abs_dist < STOP_MARGIN) {
        profile_pos = target;
        current_vel = 0;
        return;
    }

    float stopping_dist = (current_vel * current_vel) / (2.0f * MAX_ACCEL);

    if (abs_dist > stopping_dist) {
        if (distance > 0) {
            current_vel += MAX_ACCEL;
            if (current_vel > MAX_VEL) current_vel = MAX_VEL;
        } else {
            current_vel -= MAX_ACCEL;
            if (current_vel < -MAX_VEL) current_vel = -MAX_VEL;
        }
    } else {
        if (current_vel > 0) {
            current_vel -= MAX_ACCEL;
            if (current_vel < 0) current_vel = 0;
        } else if (current_vel < 0) {
            current_vel += MAX_ACCEL;
            if (current_vel > 0) current_vel = 0;
        }
    }
    profile_pos += current_vel;
}

void Check_Bus_Errors(mcp2515_can &can, volatile uint8_t &tec, volatile uint8_t &rec) {
    tec = can.errorCountTX();
    rec = can.errorCountRX();
}

/* ===== 타이머 인터럽트 서비스 루틴 (ISR) ===== */
void Control_ISR(void) {
    Check_Bus_Errors(CAN1, can1_tec, can1_rec);
    Check_Bus_Errors(CAN2, can2_tec, can2_rec);

    CAN1.sendMsgBuf(COB_ID_SYNC, 0, 0, NULL);
    CAN2.sendMsgBuf(COB_ID_SYNC, 0, 0, NULL);

    auto processReceive = [](mcp2515_can &can, volatile uint32_t &hb_tick) {
        while (can.checkReceive() == CAN_MSGAVAIL) {
            unsigned long id;
            uint8_t len, buf[8];
            can.readMsgBuf(&id, &len, buf);
            if (id == COB_ID_TPDO1) {
                actual_pos = ((long)buf[3] << 24) | ((long)buf[2] << 16) | ((long)buf[1] << 8) | (long)buf[0];
            } else if (id == COB_ID_HEARTBEAT) {
                driver_state = buf[0];
                hb_tick = millis();
            }
        }
    };
    processReceive(CAN1, last_hb_can1);
    processReceive(CAN2, last_hb_can2);

    // 1. 비상 상황 및 인터락 상태 업데이트
    can1_healthy = (millis() - last_hb_can1 < HEARTBEAT_TIMEOUT_MS) && (can1_tec < BUS_ERROR_THRESHOLD);
    can2_healthy = (millis() - last_hb_can2 < HEARTBEAT_TIMEOUT_MS) && (can2_tec < BUS_ERROR_THRESHOLD);
    bool handle_fail = (millis() - last_steer_tick > TIMEOUT_MS);
    
    if (handle_fail || (!can1_healthy && !can2_healthy) || is_steering_fault || is_power_lost) {
        is_emergency = true;
    } else {
        is_emergency = false;
    }

    // [신규] 브레이크 상태에 따른 Controlword 및 인터락 결정
    Update_Brake_And_Interlock();

    // 2. 모션 프로필 업데이트 (인터락 상태 반영됨)
    Update_Motion_Profile();
    Monitor_Steering_Performance();

    // 3. RPDO 전송 (Controlword 16bit + Target Position 32bit)
    // 데이터 구조: [Control L][Control H][Pos L][Pos M1][Pos M2][Pos H]
    long final_pos = (long)profile_pos;
    uint8_t d[6];
    d[0] = (uint8_t)(control_word & 0xFF);
    d[1] = (uint8_t)((control_word >> 8) & 0xFF);
    d[2] = (uint8_t)(final_pos & 0xFF);
    d[3] = (uint8_t)((final_pos >> 8) & 0xFF);
    d[4] = (uint8_t)((final_pos >> 16) & 0xFF);
    d[5] = (uint8_t)((final_pos >> 24) & 0xFF);
    
    CAN1.sendMsgBuf(COB_ID_RPDO1, 0, 6, d);
    CAN2.sendMsgBuf(COB_ID_RPDO1, 0, 6, d);
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(57600); 

    pinMode(PIN_BRAKE_RELAY, OUTPUT);
    digitalWrite(PIN_BRAKE_RELAY, LOW); 
    
    pinMode(PIN_POWER_SENSE, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_POWER_SENSE), PowerLoss_ISR, FALLING);

    if (CAN1.begin(CAN_250KBPS, MCP_16MHz) != 0 || CAN2.begin(CAN_250KBPS, MCP_16MHz) != 0) {
        while (1) Serial.println("Dual CAN Init Fail!");
    }
    CAN1.setMode(MCP_NORMAL);
    CAN2.setMode(MCP_NORMAL);

    MyTim = new HardwareTimer(TIM2); 
    MyTim->setOverflow(CONTROL_PERIOD_US, MICROSEC_FORMAT); 
    MyTim->attachInterrupt(Control_ISR); 
    MyTim->resume(); 

    Serial.println("V10: Software Interlock & State Machine Initialized");
}

void loop() {
    if (Serial1.available() > 0) {
        String input = Serial1.readStringUntil('\n');
        if (input.startsWith("S")) {
            steer_cmd = input.substring(1).toInt();
            last_steer_tick = millis();
        }
    }

    static uint32_t lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print("CtrlWord:0x"); Serial.print(control_word, HEX);
        Serial.print(" | Brake:"); Serial.print(digitalRead(PIN_BRAKE_RELAY) ? "OPEN" : "LOCKED");
        Serial.print(" | Pos:"); Serial.println((long)profile_pos);
    }
}
