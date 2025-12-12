/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (DCDC CAN Monitor & Control)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_DCDC_NODE_ID_CMD1   0x18FF1A01u
#define CAN_DCDC_NODE_ID_CMD2   0x18FF1A02u
#define CAN_DCDC_NODE_ID_MEAS   0x18FF1587u
#define CAN_DCDC_NODE_ID_STATE  0x18FF2587u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* BSP Nucleo UART 핸들 (COM1 = ST-LINK VCP) */
extern UART_HandleTypeDef hcom_uart[COMn];

/* FDCAN 핸들 */
extern FDCAN_HandleTypeDef hfdcan1;

/* ===== 아두이노 코드에서 가져온 전역 변수들 ===== */

/* sendOne */
uint8_t start_value1 = 0x1;
uint8_t mode_value1  = 0x2;

uint8_t one_value1   = 0x02;  // 출력 전압 상위 바이트 (예: 51.3V → 513 → 0x0201)
uint8_t two_value1   = 0x01;  // 출력 전압 하위 바이트

uint8_t three_value1 = 0x06;  // 입력 전류 설정
uint8_t four_value1  = 0x40;

uint8_t five_value1  = 0x01;
uint8_t six_value1   = 0x00;
uint8_t seven_value1 = 0x00;

/* sendTwo */
uint8_t zero_value2  = 0x00;

uint8_t one_value2   = 0x01;  // 입력 전압 상위 바이트 (25.8V → 258 → 0x0102)
uint8_t two_value2   = 0x90;  // 입력 전압 하위 바이트

uint8_t three_value2 = 0x06;  // 출력 전류 설정 (예: 10A → 1600 → 0x0640)
uint8_t four_value2  = 0x40;

uint8_t five_value2  = 0x00;
uint8_t six_value2   = 0x00;
uint8_t seven_value2 = 0x00;

/* receive 계산 결과 */
float outV = 0.0f;
float outA = 0.0f;
float inV  = 0.0f;
float inA  = 0.0f;

uint8_t temp       = 0;
uint8_t disability = 0;
uint8_t condition  = 0;
uint8_t error_code = 0;

uint8_t life      = 10;
uint8_t lifeCheck = 0;

/* 기타 상태 플래그 */
bool systemStarted = false;
bool contin        = false;

uint32_t lastSendTime  = 0;
const uint32_t sendInterval  = 80;    /* ms */

uint32_t lastPrintTime = 0;
const uint32_t printInterval = 1000;  /* ms */

uint8_t condition_0 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* UART Helper */
static void pc_print(const char *s);
static void pc_printf(const char *fmt, ...);
static int  pc_get_char_nonblock(uint8_t *ch);

/* CAN Helper */
static void CAN_Start_AllFilters(void);
static void CAN_SendExt(uint32_t id, uint8_t data[8]);

/* DCDC Control */
static void waitForStart(void);
static void sendOne(void);
static void sendTwo(void);
static void receive_can(void);
static void checkStartupState(void);
static void stop_control(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();      /* ADC는 지금 사용하진 않지만 기존 설정 유지 */

  /* USER CODE BEGIN 2 */

  /* LED / 버튼 / COM1 초기화 */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* FDCAN1 시작 및 필터 설정 (모든 ID 수신) */
  CAN_Start_AllFilters();

  pc_print("\r\n==== DCDC CAN Monitor (STM32) ====\r\n");

  /* PC에서 'S' 또는 's' 입력 시까지 대기 */
  waitForStart();

  /* DCDC 시동 시퀀스 (아두이노 코드와 동일) */

  /* CAN init 비슷한 느낌: HAL_FDCAN_Start는 이미 CAN_Start_AllFilters에서 호출 */

  /* 1) 0x18FF1A01, [03 00 00 00 00 00 00 00] */
  {
    uint8_t data11[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data11);
  }
  HAL_Delay(150);

  /* 2) 0x18FF1A01, [00 00 00 00 00 01 00 00] */
  {
    uint8_t data12[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data12);
  }
  HAL_Delay(8000);

  /* 3) 0x18FF1A01, [21 02 01 06 40 01 00 00] */
  {
    uint8_t data100[8] = {0x21, 0x02, 0x01, 0x06, 0x40, 0x01, 0x00, 0x00};
    CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data100);
  }
  HAL_Delay(100);

  /* DCDC가 실행 상태(조건 == 0x1) 가 될 때까지 최대 1초 기다림 */
  uint32_t startTime = HAL_GetTick();
  contin = false;
  while (!contin && (HAL_GetTick() - startTime < 1000U))
  {
    checkStartupState();
    HAL_Delay(20);
  }

  if (!contin)
  {
    pc_print("시동 실패, stop_control 실행\r\n");
    stop_control();  /* 이 함수 안에서 리셋 */
  }

  pc_print("시동 성공, 메인 루프 진입\r\n");

  lastSendTime  = HAL_GetTick();
  lastPrintTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* PC에서 'x' 또는 'X' 보내면 stop_control */
    uint8_t ch;
    if (pc_get_char_nonblock(&ch))
    {
      if (ch == 'x' || ch == 'X')
      {
        stop_control();
      }
    }

    /* 80ms 마다 제어 프레임 송신 */
    if ((now - lastSendTime) >= sendInterval)
    {
      sendOne();
      HAL_Delay(10);
      sendTwo();
      lastSendTime = now;

      BSP_LED_Toggle(LED_GREEN);  /* 살아있음 표시 */
    }

    /* 수신 프레임 처리 */
    receive_can();

    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* ===== Helper 구현부 ===================================================== */

static void pc_print(const char *s)
{
  HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

static void pc_printf(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}

/* 한 바이트 비차단 수신: 데이터 있으면 1 리턴, 없으면 0 */
static int pc_get_char_nonblock(uint8_t *ch)
{
  if (HAL_UART_Receive(&hcom_uart[COM1], ch, 1, 0) == HAL_OK)
  {
    return 1;
  }
  return 0;
}

/* 모든 ID 수신하도록 FDCAN1 시작 */
static void CAN_Start_AllFilters(void)
{
  /* Global Filter: 매칭 안 되는 ID도 FIFO0로 수신 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                               FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_REJECT_REMOTE,
                               FDCAN_REJECT_REMOTE);

  /* Extended ID 전체 범위 필터 (0x00000000 ~ 0x1FFFFFFF) */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x00000000U;
  sFilterConfig.FilterID2    = 0x1FFFFFFFU;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    pc_print("FDCAN Start 실패\r\n");
    Error_Handler();
  }
}

/* 확장 ID 8바이트 데이터 송신 */
static void CAN_SendExt(uint32_t id, uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef txHeader;
  txHeader.Identifier          = id;
  txHeader.IdType              = FDCAN_EXTENDED_ID;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  txHeader.DataLength          = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0;

  /* TX FIFO에 여유가 생길 때까지 대기 */
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
  {
  }

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

/* PC에서 'S' 또는 's' 입력 받을 때까지 대기 */
static void waitForStart(void)
{
  pc_print("If you want to start, please press 'S'!!\r\n");

  while (!systemStarted)
  {
    uint8_t ch;
    if (pc_get_char_nonblock(&ch))
    {
      if (ch == 'S' || ch == 's')
      {
        systemStarted = true;
        pc_print("시스템 시작\r\n");
        break;
      }
    }
    HAL_Delay(100);
  }
}

/* 아두이노 sendOne() 포팅 */
static void sendOne(void)
{
  uint8_t zero_value1 = (uint8_t)((mode_value1 << 4) | (start_value1 & 0x0F));
  uint8_t data1[8] = {
      zero_value1,
      one_value1,
      two_value1,
      three_value1,
      four_value1,
      five_value1,
      six_value1,
      seven_value1};

  CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data1);
}

/* 아두이노 sendTwo() 포팅 */
static void sendTwo(void)
{
  uint8_t data2[8] = {
      zero_value2,
      one_value2,
      two_value2,
      three_value2,
      four_value2,
      five_value2,
      six_value2,
      seven_value2};

  CAN_SendExt(CAN_DCDC_NODE_ID_CMD2, data2);
}

/* 아두이노 receive() 포팅 */
static void receive_can(void)
{
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t buf[8];

  /* FIFO0에 데이터 없으면 return */
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
  {
    return;
  }

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, buf) != HAL_OK)
  {
    return;
  }

  uint32_t canId = rxHeader.Identifier;

  switch (canId)
  {
    case CAN_DCDC_NODE_ID_MEAS:
    {
      /* 출력/입력 전압/전류 계산 */
      outV = (float)((buf[0] << 8) | buf[1]) * 0.1f;
      outA = (float)((buf[2] << 8) | buf[3]) * 0.1f - 1500.0f;
      inV  = (float)((buf[4] << 8) | buf[5]) * 0.1f;
      inA  = (float)((buf[6] << 8) | buf[7]) * 0.1f - 1500.0f;

      uint32_t now = HAL_GetTick();
      if ((now - lastPrintTime) >= printInterval)
      {
        pc_printf("출력전압: %.1f V, 출력전류: %.1f A, 입력전압: %.1f V, 입력전류: %.1f A\r\n",
                  outV, outA, inV, inA);
        lastPrintTime = now;
      }
      break;
    }

    case CAN_DCDC_NODE_ID_STATE:
    {
      temp       = buf[0];
      condition  = (uint8_t)(buf[1] & 0x0F);
      disability = (uint8_t)((buf[1] & 0xF0) >> 4);
      error_code = buf[2];
      life       = buf[7];

      if (condition != 0x01)
      {
        pc_printf("DCDC condition: 0x%02X\r\n", condition);
      }

      if ((int8_t)(temp - 40) > 65)
      {
        pc_print("temperature warning!\r\n");
        stop_control();
      }

      if (disability != 0x00)
      {
        pc_printf("Disability: 0x%02X\r\n", disability);
      }

      if (error_code != 0x00)
      {
        pc_printf("Error Code: 0x%02X\r\n", error_code);
      }

      if (life != lifeCheck)
      {
        lifeCheck = life;
      }
      else
      {
        pc_print("life warning!\r\n");
      }
      break;
    }

    default:
      /* 필요하면 디버그용으로 ID/데이터 출력 */
      break;
  }
}

/* 아두이노 checkStartupState() 포팅 */
static void checkStartupState(void)
{
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t buf[8];

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
  {
    return;
  }

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, buf) != HAL_OK)
  {
    return;
  }

  uint32_t canId = rxHeader.Identifier;

  if (canId == CAN_DCDC_NODE_ID_STATE)
  {
    condition_0 = (uint8_t)(buf[1] & 0x0F);
    if (condition_0 == 0x1)
    {
      pc_print("실행 상태 감지됨.\r\n");
      contin = true;
    }
    else
    {
      pc_print("실행 상태 감지되지 않음.\r\n데이터: ");
      for (int i = 0; i < 8; i++)
      {
        pc_printf("%02X ", buf[i]);
      }
      pc_print("\r\n");
    }
  }
  else
  {
    pc_printf("수신 ID: 0x%08lX\r\n데이터: ", (unsigned long)canId);
    for (int i = 0; i < 8; i++)
    {
      pc_printf("%02X ", buf[i]);
    }
    pc_print("\r\n");
  }
}

/* 아두이노 stop_control() 포팅 */
static void stop_control(void)
{
  uint8_t data3[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data3);
  HAL_Delay(50);

  uint8_t data4[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_SendExt(CAN_DCDC_NODE_ID_CMD2, data4);
  HAL_Delay(200);

  uint8_t data5[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_SendExt(CAN_DCDC_NODE_ID_CMD1, data5);
  HAL_Delay(200);

  pc_print("stopping the system..\r\n");
  HAL_Delay(500);

  /* AVR wdt 대신 STM32 소프트 리셋 */
  NVIC_SystemReset();
}

/* ===== 이하 Clock / Error_Handler 는 기존 그대로 사용 ===== */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN            = 10;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 사용 시 필요하면 구현 */
}
#endif /* USE_FULL_ASSERT */
