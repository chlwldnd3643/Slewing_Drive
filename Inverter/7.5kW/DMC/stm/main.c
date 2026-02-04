/* main.c */

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// === 원래 Arduino 전역 변수들 포팅 ===

// sendOne
uint8_t start_value1 = 0x1;
uint8_t mode_value1  = 0x2;  // 제어 모드

uint8_t one_value1   = 0x02; // 출력 전압 상위 바이트 (51.3V → 513 → 0x0201)
uint8_t two_value1   = 0x01; // 출력 전압 하위 바이트

uint8_t three_value1 = 0x06; // 입력 전류 설정
uint8_t four_value1  = 0x40;

uint8_t five_value1  = 0x01;
uint8_t six_value1   = 0x00;
uint8_t seven_value1 = 0x00;

// sendTwo
uint8_t zero_value2  = 0x00;

uint8_t one_value2   = 0x01; // 입력 전압 상위 바이트 (25.8V → 258 → 0x0102)
uint8_t two_value2   = 0x90; // 입력 전압 하위 바이트

uint8_t three_value2 = 0x06; // 출력 전류 설정
uint8_t four_value2  = 0x40;

uint8_t five_value2  = 0x00;
uint8_t six_value2   = 0x00;
uint8_t seven_value2 = 0x00;

// receive
float outV = 0.0f;
float outA = 0.0f;
float inV  = 0.0f;
float inA  = 0.0f;

uint8_t temp = 0;
uint8_t disability = 0;
uint8_t condition = 0;
uint8_t error_code = 0;

uint8_t life = 10;
uint8_t lifeCheck = 0;

// 기타 상태
bool systemStarted = false;
bool contin = false;
uint32_t lastSendTime  = 0;
const  uint32_t sendInterval  = 80;    // ms

uint32_t lastPrintTime = 0;
const  uint32_t printInterval = 1000; // 1초마다 출력

uint8_t condition_0 = 0;

// 외부 핸들
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart2;

// === 함수 프로토타입 ===
void waitForStart(void);
void stop_control(void);
void sendOne(void);
void sendTwo(void);
void receive_can(void);
void checkStartupState(void);
void CAN_SendExt(uint32_t id, uint8_t *data, uint8_t len);


// === 메인 함수 ===
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();

  // Arduino: Serial.begin(115200)
  const char hello[] = "STM32 IBDC control starting...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello, sizeof(hello)-1, HAL_MAX_DELAY);

  // Arduino: waitForStart();
  waitForStart();

  // Arduino: while (CAN_OK != CAN.begin(CAN_250KBPS)) { ... }
  // 여기서는 FDCAN은 이미 MX_FDCAN1_Init에서 초기화/Start 완료되었다고 가정.
  // 오류 시 Error_Handler 호출되어 멈춤.

  const char canok[] = "CAN BUS init OK\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)canok, sizeof(canok)-1, HAL_MAX_DELAY);

  // === Arduino setup()의 초기 CAN 시퀀스 ===

  uint8_t data11[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_SendExt(0x18FF1A01, data11, 8);
  HAL_Delay(150);

  uint8_t data12[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  CAN_SendExt(0x18FF1A01, data12, 8);
  HAL_Delay(8000);

  uint8_t data100[8] = {0x21, 0x02, 0x01, 0x06, 0x40, 0x01, 0x00, 0x00};
  CAN_SendExt(0x18FF1A01, data100, 8);
  HAL_Delay(100);

  // 실행 상태 체크 (1초)
  uint32_t startTime = HAL_GetTick();
  while (!contin && (HAL_GetTick() - startTime < 1000))
  {
    checkStartupState();
    HAL_Delay(20);
  }

  if (!contin)
  {
    stop_control();
  }

  lastSendTime  = HAL_GetTick();
  lastPrintTime = HAL_GetTick();

  // === Arduino loop() ===
  while (1)
  {
    // PC에서 'x' 입력하면 stop_control() 호출 (간단한 stop 명령)
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
      if (ch == 'x' || ch == 'X')
      {
        stop_control();
      }
    }

    uint32_t now = HAL_GetTick();
    if (now - lastSendTime > sendInterval)
    {
      sendOne();
      HAL_Delay(10);
      sendTwo();
      lastSendTime = now;
    }

    receive_can();

    HAL_Delay(10); // 너무 바쁘지 않게 약간 쉬어줌 (Arduino의 delay(90)에 해당)
  }
}


// === CAN 송신 함수 (MCP2515 → FDCAN) ===
void CAN_SendExt(uint32_t id, uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 항상 8바이트 사용
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
}


// === Arduino: sendOne() ===
void sendOne(void)
{
  uint8_t zero_value1 = (mode_value1 << 4) | (start_value1 & 0x0F);
  uint8_t data1[8] = {
    zero_value1,
    one_value1, two_value1,
    three_value1, four_value1,
    five_value1, six_value1, seven_value1
  };

  CAN_SendExt(0x18FF1A01, data1, 8);
}

// === Arduino: sendTwo() ===
void sendTwo(void)
{
  uint8_t data2[8] = {
    zero_value2,
    one_value2, two_value2,
    three_value2, four_value2,
    five_value2, six_value2, seven_value2
  };

  CAN_SendExt(0x18FF1A02, data2, 8);
}


// === Arduino: receive() ===
void receive_can(void)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t buf[8];

  // FIFO0에 메시지가 있는 동안 처리
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
    {
      return;
    }

    uint32_t canId = RxHeader.Identifier;

    switch (canId)
    {
      case 0x18FF1587:
        outV = (buf[0] << 8 | buf[1]) * 0.1f;
        outA = (buf[2] << 8 | buf[3]) * 0.1f - 1500.0f;
        inV  = (buf[4] << 8 | buf[5]) * 0.1f;
        inA  = (buf[6] << 8 | buf[7]) * 0.1f - 1500.0f;

        if (HAL_GetTick() - lastPrintTime > printInterval)
        {
          char msg[128];
          snprintf(msg, sizeof(msg),
                   "출력전압: %.1fV, 출력전류: %.1fA, 입력전압: %.1fV, 입력전류: %.1fA\r\n",
                   outV, outA, inV, inA);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
          lastPrintTime = HAL_GetTick();
        }
        break;

      case 0x18FF2587:
        temp       = buf[0];
        condition  = buf[1] & 0x0F;
        disability = (buf[1] & 0xF0) >> 4;
        error_code = buf[2];
        life       = buf[7];

        if (condition != 0x01)
        {
          char msg[64];
          snprintf(msg, sizeof(msg), "DCDC condition: %u\r\n", condition);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        if ((int)(temp - 40) > 65)
        {
          const char warn[] = "temperature warning!\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)warn, sizeof(warn)-1, HAL_MAX_DELAY);
          stop_control();
        }

        if (disability != 0x00)
        {
          char msg[64];
          snprintf(msg, sizeof(msg), "Disability: 0x%02X\r\n", disability);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        if (error_code != 0x00)
        {
          char msg[64];
          snprintf(msg, sizeof(msg), "Error Code: 0x%02X\r\n", error_code);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        if (life != lifeCheck)
        {
          lifeCheck = life;
        }
        else
        {
          const char warn[] = "life warning!\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)warn, sizeof(warn)-1, HAL_MAX_DELAY);
        }
        break;

      default:
        // 필요하면 디버깅용으로 다른 ID들도 찍을 수 있음
        break;
    }
  }
}


// === Arduino: checkStartupState() ===
void checkStartupState(void)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t buf[8];

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
    return;

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
    return;

  uint32_t canId = RxHeader.Identifier;

  if (canId == 0x18FF2587)
  {
    condition_0 = buf[1] & 0x0F;
    if (condition_0 == 0x1)
    {
      const char msg[] = "실행 상태 감지됨.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
      contin = true;
    }
    else
    {
      const char msg[] = "실행 상태 감지되지 않음.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

      // 필요하면 데이터도 HEX로 찍기
      char bufmsg[64];
      for (int i = 0; i < 8; i++)
      {
        snprintf(bufmsg, sizeof(bufmsg), "%02X ", buf[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)bufmsg, strlen(bufmsg), HAL_MAX_DELAY);
      }
      const char crlf[] = "\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf)-1, HAL_MAX_DELAY);
    }
  }
  else
  {
    // 수신 ID, 데이터 찍고 싶으면 여기서 출력
  }
}


// === Arduino: waitForStart() ===
void waitForStart(void)
{
  const char msg[] = "If you want to start, please press 'S'!!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

  uint8_t ch;
  while (!systemStarted)
  {
    if (HAL_UART_Receive(&huart2, &ch, 1, 100) == HAL_OK)
    {
      if (ch == 'S' || ch == 's')
      {
        systemStarted = true;
        const char started[] = "시스템 시작\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)started, sizeof(started)-1, HAL_MAX_DELAY);
      }
    }
  }
}


// === Arduino: stop_control() ===
void stop_control(void)
{
  uint8_t data3[8] = {0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00};
  CAN_SendExt(0x18FF1A01, data3, 8);
  HAL_Delay(50);

  uint8_t data4[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  CAN_SendExt(0x18FF1A02, data4, 8);
  HAL_Delay(200);

  uint8_t data5[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  CAN_SendExt(0x18FF1A01, data5, 8);
  HAL_Delay(200);

  const char msg[] = "stopping the system..\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

  HAL_Delay(500);

  // AVR의 wdt_reset에 대응: STM32는 소프트웨어 리셋 사용
  NVIC_SystemReset();
}
