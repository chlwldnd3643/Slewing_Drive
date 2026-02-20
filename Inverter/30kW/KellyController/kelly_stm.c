/* --- CAN ID 정의 --- */
// KLS 수신(VCU -> Controller): 일반적으로 0x0C010511 (Priority:3, PF:1, PS:05, SA:11)
static const uint32_t KELLY_CMD_ID = 0x0C010511U; 

// KLS 송신(Controller -> VCU): 프로토콜 매뉴얼 기준
static const uint32_t KELLY_ID_MSG1 = 0x0CF11E05U; // 속도/전류/전압 
static const uint32_t KELLY_ID_MSG2 = 0x0CF11F05U; // 스로틀/온도/상태 [cite: 782]

/* --- 제어 변수 --- */
uint8_t target_throttle = 0; // 0 ~ 255 (0V ~ 5V) [cite: 782]
uint8_t target_direction = 1; // 0:Neutral, 1:Forward, 2:Backward [cite: 782]

/**
 * @brief Kelly 컨트롤러에 동작 명령 전송 (100ms 주기로 호출 권장)
 */
void Kelly_SendCommand(uint8_t throttle, uint8_t direction) {
    uint8_t txData[8] = {0};
    
    txData[0] = throttle;    // Byte 0: 스로틀 명령 (0~255)
    txData[1] = direction;   // Byte 1: 방향 (1:전진, 2:후진)
    txData[2] = 0;           // 나머지 예약된 바이트
    
    can_tx_ext(KELLY_CMD_ID, txData, 8);
}

/**
 * @brief 수신 데이터 통합 처리
 */
void Kelly_Handle_Response(uint32_t id, uint8_t *data) {
    if (id == KELLY_ID_MSG1) {
        // RPM: 1rpm/bit 
        uint16_t rpm = (uint16_t)((data[1] << 8) | data[0]);
        // Current: 0.1A/bit 
        float current = ((data[3] << 8) | data[2]) / 10.0f;
        // Voltage: 0.1V/bit 
        float voltage = ((data[5] << 8) | data[4]) / 10.0f;
        
        printf("[MON] %u RPM | %.1fV | %.1fA\r\n", rpm, voltage, current);
    } 
    else if (id == KELLY_ID_MSG2) {
        // 온도 파싱: Offset 적용 [cite: 782]
        int8_t c_temp = (int8_t)data[1] - 40; // 컨트롤러 온도 [cite: 782]
        int8_t m_temp = (int8_t)data[2] - 30; // 모터 온도 [cite: 782]
        
        printf("[MON] T_Ctrl: %dC | T_Mot: %dC | Stat: 0x%02X\r\n", c_temp, m_temp, data[4]);
    }
}

/* --- 메인 루프 예시 --- */
uint32_t last_cmd_time = 0;

while (1) {
    // 1. 수신 처리
    uint32_t rid;
    uint8_t rdata[8];
    uint8_t rlen;
    while (can_rx_fifo0(&rid, rdata, &rlen)) {
        Kelly_Handle_Response(rid, rdata);
    }

    // 2. 100ms마다 제어 명령 전송 (Keep-alive 역할 포함)
    if (HAL_GetTick() - last_cmd_time >= 100) {
        // 안전을 위해 에러 상황 시 throttle을 0으로 설정하는 로직 권장
        Kelly_SendCommand(target_throttle, target_direction);
        last_cmd_time = HAL_GetTick();
    }
}
