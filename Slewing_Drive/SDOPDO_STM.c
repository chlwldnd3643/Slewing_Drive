/* main.c – Kinco FD 드라이브 + 핸들 보드 제어 (STM32 FDCAN 포팅 버전) */

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

/* ---- 기본 설정 및 전역 ---- */

// Kinco 드라이브 노드 ID
static uint8_t NODE_ID = 1;

// profile_speed 스케일
// 실측: 6081 = 1000(dec) → 약 0.06 rpm
// => 1 rpm 당 DEC ≈ 16666.7
static const float DEC_PER_RPM = 1000.0f / 0.06f;

// 제어 모드
typedef enum {
    MODE_SDO_CMD    = 0,   // SDO ABS/REL/SPD
    MODE_HANDLE     = 1,   // 핸들값으로 실시간 위치제어
    MODE_BOTH_DEBUG = 2    // 둘 다 활성
} ControlMode;

static ControlMode controlMode = MODE_SDO_CMD;

// 핸들 모드에서 RPDO를 진짜 쓸지 여부
static const bool USE_PDO_FOR_HANDLE = false;

// profile speed 관련
static float    g_profSpeedRpm = 200.0f;
static uint32_t g_profSpeedDec = 0;

// SDO 기반 타겟 포지션 기억
static int32_t g_targetPos = 0;

// 핸들 보드에서 받은 최종 포지션 (-2.5M ~ +2.5M)
static int32_t steer_cmd = 0;

// PC 시리얼 라인 버퍼
static char serialLine[64];
static uint8_t serialLineLen = 0;

// 핸들 시리얼 라인 버퍼 ("S123456\n" 형태)
static char steerLine[32];
static uint8_t steerLineLen = 0;

// FDCAN / UART 핸들 (Cube가 생성)
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart2;   // PC용
extern UART_HandleTypeDef huart3;   // 핸들용

/* 간단한 UART 출력 헬퍼 (PC) */
static void pc_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

static void pc_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

/* ---- CAN 송신 헬퍼 (확장 ID, 8바이트) ---- */
static bool CAN_SendExt(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (len < 8) {
        // 나머지는 0 채우기
        uint8_t tmp[8] = {0};
        memcpy(tmp, data, len);
        return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tmp) == HAL_OK);
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*)data) == HAL_OK);
}

/* ---- SDO write 유틸 ---- */

static bool SDO_write_u8(uint16_t idx, uint8_t sub, uint8_t val)
{
    uint8_t d[8] = {
        0x2F,
        (uint8_t)(idx & 0xFF),
        (uint8_t)(idx >> 8),
        sub,
        val, 0, 0, 0
    };
    return CAN_SendExt(0x600 + NODE_ID, d, 8);
}

static bool SDO_write_u16(uint16_t idx, uint8_t sub, uint16_t val)
{
    uint8_t d[8] = {
        0x2B,
        (uint8_t)(idx & 0xFF),
        (uint8_t)(idx >> 8),
        sub,
        (uint8_t)(val & 0xFF),
        (uint8_t)(val >> 8),
        0, 0
    };
    return CAN_SendExt(0x600 + NODE_ID, d, 8);
}

static bool SDO_write_u32(uint16_t idx, uint8_t sub, uint32_t val)
{
    uint8_t d[8] = {
        0x23,
        (uint8_t)(idx & 0xFF),
        (uint8_t)(idx >> 8),
        sub,
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF),
        (uint8_t)((val >> 16) & 0xFF),
        (uint8_t)((val >> 24) & 0xFF)
    };
    return CAN_SendExt(0x600 + NODE_ID, d, 8);
}

static bool SDO_write_i32(uint16_t idx, uint8_t sub, int32_t val)
{
    return SDO_write_u32(idx, sub, (uint32_t)val);
}

/* ---- PDO: 타겟 포지션 (607A) 송신 ----
 * 가정: RPDO1(0x1400 / 0x1600)이 607A:00, 32bit로 매핑,
 *      COB-ID = 0x200 + NODE_ID
 */
static bool PDO_write_target_position(int32_t pos)
{
    uint8_t d[8];
    d[0] = (uint8_t)( pos        & 0xFF);
    d[1] = (uint8_t)((pos >> 8 ) & 0xFF);
    d[2] = (uint8_t)((pos >> 16) & 0xFF);
    d[3] = (uint8_t)((pos >> 24) & 0xFF);
    d[4] = d[5] = d[6] = d[7] = 0;
    return CAN_SendExt(0x200 + NODE_ID, d, 8);
}

/* ---- profile_speed(0x6081) 설정 (입력: rpm) ---- */
static void set_profile_speed_rpm(float rpm)
{
    if (rpm < 0.0f) rpm = -rpm;
    if (rpm < 0.1f) rpm = 0.1f;

    float dec_f = rpm * DEC_PER_RPM;
    if (dec_f > 4.0e9f) dec_f = 4.0e9f;

    uint32_t dec = (uint32_t)(dec_f + 0.5f);

    g_profSpeedRpm = rpm;
    g_profSpeedDec = dec;

    bool ok = SDO_write_u32(0x6081, 0x00, g_profSpeedDec);

    pc_printf("[SPD] req rpm=%.2f -> 6081(dec)=%lu %s\r\n",
              rpm, (unsigned long)g_profSpeedDec, ok ? "(OK)" : "(FAIL)");
}

/* ---- (선택) RPDO1 → 607A 매핑 ---- */
static void configure_RPDO1_for_target_position(void)
{
    // 1) RPDO1 disable (COB-ID 상위 bit = 1)
    SDO_write_u32(0x1400, 0x01, 0x80000200UL + NODE_ID);
    HAL_Delay(10);

    // 2) 매핑 지우기
    SDO_write_u8(0x1600, 0x00, 0);
    HAL_Delay(10);

    // 3) 607A:00, 32bit 매핑
    SDO_write_u32(0x1600, 0x01, 0x607A0020UL);
    HAL_Delay(10);

    // 4) 항목 개수 = 1
    SDO_write_u8(0x1600, 0x00, 1);
    HAL_Delay(10);

    // 5) RPDO1 enable : COB-ID = 0x200 + NODE_ID
    SDO_write_u32(0x1400, 0x01, 0x200 + NODE_ID);
    HAL_Delay(10);

    pc_print("[RPDO] RPDO1 mapped to 0x607A:00, COB-ID=0x200+ID\r\n");
}

/* ---- 드라이브 Enable (Position Mode + 0x103F) ---- */
static bool enable_drive_position_mode(void)
{
    bool ok = true;

    // 1) Fault reset (0x86)
    ok &= SDO_write_u16(0x6040, 0x00, 0x0086);
    HAL_Delay(20);

    // 2) Mode of operation = 1 (Profile Position Mode)
    ok &= SDO_write_u8(0x6060, 0x00, 0x01);
    HAL_Delay(20);

    // 3) profile_speed 설정
    set_profile_speed_rpm(g_profSpeedRpm);
    HAL_Delay(20);

    // 4) Controlword = 0x103F
    ok &= SDO_write_u16(0x6040, 0x00, 0x103F);
    HAL_Delay(20);

    return ok;
}

/* ---- SDO 포지션 명령 ---- */
static void sdo_move_abs(int32_t pos)
{
    g_targetPos = pos;
    SDO_write_i32(0x607A, 0x00, g_targetPos);
    HAL_Delay(5);

    pc_printf("[SDO ABS] targetPos = %ld\r\n", (long)g_targetPos);
}

static void sdo_move_rel(int32_t delta)
{
    g_targetPos += delta;
    sdo_move_abs(g_targetPos);
}

/* ---- PC 시리얼 명령 처리 ----
   en          : enable (pos mode + 6081(rpm) + 0x103F)
   rst         : 0x6040 = 0x0086
   abs <pos>   : 절대 위치
   rel <delta> : 상대 위치
   spd <rpm>   : profile_speed 설정
   1/2/3       : 모드 변경
   h/help      : 도움말
*/
static void processSerial(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
        if (ch == '\r')
            continue;

        if (ch == '\n') {
            if (serialLineLen == 0) {
                // 빈 줄
                return;
            }
            serialLine[serialLineLen] = '\0';
            serialLineLen = 0;

            // 소문자로 변환
            for (char *p = serialLine; *p; ++p) {
                if (*p >= 'A' && *p <= 'Z') *p = *p - 'A' + 'a';
            }

            char *line = serialLine;

            // 모드 전환
            if (strcmp(line, "1") == 0) {
                controlMode = MODE_SDO_CMD;
                pc_print("[MODE] SDO CMD (ABS/REL/SPD)\r\n");
                return;
            }
            if (strcmp(line, "2") == 0) {
                controlMode = MODE_HANDLE;
                pc_print("[MODE] HANDLE (steering -> target position)\r\n");
                return;
            }
            if (strcmp(line, "3") == 0) {
                controlMode = MODE_BOTH_DEBUG;
                pc_print("[MODE] BOTH (SDO + HANDLE)\r\n");
                return;
            }
            if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0) {
                pc_print("=== HELP ===\r\n");
                pc_print(" en          : enable (pos mode + 6081(rpm) + 0x103F)\r\n");
                pc_print(" rst         : 0x6040 = 0x0086 (fault reset)\r\n");
                pc_print(" abs <pos>   : absolute position via SDO (inc)\r\n");
                pc_print(" rel <delta> : relative move via SDO (inc)\r\n");
                pc_print(" spd <rpm>   : profile_speed(0x6081) set by rpm\r\n");
                pc_print(" 1           : MODE_SDO_CMD\r\n");
                pc_print(" 2           : MODE_HANDLE\r\n");
                pc_print(" 3           : MODE_BOTH_DEBUG\r\n");
                return;
            }

            // enable / reset
            if (strcmp(line, "en") == 0) {
                bool ok = enable_drive_position_mode();
                pc_print(ok ? "[EN] OK\r\n" : "[EN] FAIL\r\n");
                return;
            }
            if (strcmp(line, "rst") == 0) {
                bool ok = SDO_write_u16(0x6040, 0x00, 0x0086);
                pc_print(ok ? "[RST] 0x86 OK\r\n" : "[RST] FAIL\r\n");
                return;
            }

            // spd <rpm>
            if (strncmp(line, "spd", 3) == 0) {
                char *arg = line + 3;
                while (*arg == ' ') arg++;
                float rpm = atof(arg);
                set_profile_speed_rpm(rpm);
                return;
            }

            // abs / rel (SDO 모드 또는 BOTH일 때만 처리)
            if (controlMode == MODE_SDO_CMD || controlMode == MODE_BOTH_DEBUG) {
                if (strncmp(line, "abs", 3) == 0) {
                    char *arg = line + 3;
                    while (*arg == ' ') arg++;
                    int32_t p = (int32_t)strtol(arg, NULL, 10);
                    sdo_move_abs(p);
                    return;
                }
                if (strncmp(line, "rel", 3) == 0) {
                    char *arg = line + 3;
                    while (*arg == ' ') arg++;
                    int32_t d = (int32_t)strtol(arg, NULL, 10);
                    sdo_move_rel(d);
                    return;
                }
            }

            pc_printf("[WARN] Unknown cmd: %s\r\n", line);
            return;
        } else {
            if (serialLineLen < sizeof(serialLine) - 1) {
                serialLine[serialLineLen++] = (char)ch;
            } else {
                serialLineLen = 0; // overflow 시 리셋
            }
        }
    }
}

/* ---- 핸들 보드로부터 S<long>\n 파싱 (USART3) ---- */
static void readSteerCommand(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart3, &ch, 1, 0) == HAL_OK) {
        if (ch == '\r') continue;

        if (ch == '\n') {
            if (steerLineLen == 0) return;

            steerLine[steerLineLen] = '\0';
            steerLineLen = 0;

            if (steerLine[0] == 'S' || steerLine[0] == 's') {
                long val = strtol(&steerLine[1], NULL, 10);
                if (val >  2500000L) val =  2500000L;
                if (val < -2500000L) val = -2500000L;
                steer_cmd = (int32_t)val;
            }
        } else {
            if (steerLineLen < sizeof(steerLine) - 1) {
                steerLine[steerLineLen++] = (char)ch;
            } else {
                steerLineLen = 0;
            }
        }
    }
}

/* ---- main ---- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_FDCAN1_Init();

    pc_print("=== Kinco FD1x5 Slewing Controller (STM32 port) ===\r\n");
    pc_print(" 1) after power, send 'en' for pos mode + 6081(rpm) + 0x103F\r\n");
    pc_print(" 2) MODE 1: abs/rel/spd (SDO)\r\n");
    pc_print(" 3) MODE 2: handle -> target position (-2.5M~+2.5M)\r\n");
    pc_print(" cmds: en, rst, abs, rel, spd, 1/2/3, h/help\r\n");

    if (USE_PDO_FOR_HANDLE) {
        configure_RPDO1_for_target_position();
    }

    controlMode = MODE_SDO_CMD;

    uint32_t lastHandlePrint = HAL_GetTick();

    while (1)
    {
        // 1) PC 명령 처리
        processSerial();

        // 2) 핸들 모드라면 핸들 값 읽어와서 포지션 명령
        if (controlMode == MODE_HANDLE || controlMode == MODE_BOTH_DEBUG) {
            readSteerCommand();

            int32_t pos = steer_cmd;

            if (USE_PDO_FOR_HANDLE) {
                PDO_write_target_position(pos);
            } else {
                // SDO 스트리밍 방식 (항상 움직이게)
                SDO_write_i32(0x607A, 0x00, pos);
                // 필요 시 0x6040 0x103F를 주기적으로 넣어도 됨
                // SDO_write_u16(0x6040, 0x00, 0x103F);
            }

            uint32_t now = HAL_GetTick();
            if (now - lastHandlePrint > 200) {
                pc_printf("[HANDLE] targetPos=%ld\r\n", (long)pos);
                lastHandlePrint = now;
            }
        }

        HAL_Delay(10);
    }
}
