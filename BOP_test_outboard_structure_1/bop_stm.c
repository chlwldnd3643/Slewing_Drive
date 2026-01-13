/* main.c  STM32G431  Relay Command Console  Active Low */

#include "main.h"
#include "gpio.h"
#include "stm32g4xx_nucleo.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

/* BSP COM */
COM_InitTypeDef BspCOMInit;
extern UART_HandleTypeDef hcom_uart[COMn];

/* Active Low relay logic */
#define RELAY_ON  GPIO_PIN_RESET
#define RELAY_OFF GPIO_PIN_SET

/* Pin mapping, arbitrary */
#define PUMP_GPIO_Port           GPIOB
#define PUMP_Pin                 GPIO_PIN_0

#define PUMP_SEA_GPIO_Port       GPIOB
#define PUMP_SEA_Pin             GPIO_PIN_1

#define HEATER_GPIO_Port         GPIOB
#define HEATER_Pin               GPIO_PIN_2

#define CBV_SEA_PWR_GPIO_Port    GPIOB
#define CBV_SEA_PWR_Pin          GPIO_PIN_10

#define CBV_SEA_GPIO_Port        GPIOB
#define CBV_SEA_Pin              GPIO_PIN_11

#define O2_COMP_GPIO_Port        GPIOB
#define O2_COMP_Pin              GPIO_PIN_12

#define H2_IN_GPIO_Port          GPIOB
#define H2_IN_Pin                GPIO_PIN_13

#define H2_OUT_GPIO_Port         GPIOB
#define H2_OUT_Pin               GPIO_PIN_14

static bool systemStarted = false;

/* command line buffer */
static char cmd_buf[32];
static uint8_t cmd_len = 0;

static void pc_print(const char *s);
static void pc_printf(const char *fmt, ...);
static int  pc_get_char_nonblock(uint8_t *ch);

static void relays_init_state(void);
static void handle_command(const char *cmd);
static void waitForStart(void);

void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  BSP_LED_Init(LED_GREEN);

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  pc_print("\r\n==== Relay Console (STM32G431) ====\r\n");
  pc_print("Logic: ACTIVE LOW (LOW = ON)\r\n");

  relays_init_state();
  waitForStart();

  pc_print("\r\nCommands:\r\n");
  pc_print("p1/p0    pump on/off\r\n");
  pc_print("ps1/ps0  pump_sea on/off\r\n");
  pc_print("h1/h0    heater on/off\r\n");
  pc_print("csp1/csp0  cbv_sea_power on/off\r\n");
  pc_print("cs1/cs0    cbv_sea on/off\r\n");
  pc_print("c1/c0    oxygen_compressor on/off\r\n");
  pc_print("iv1/iv0  hydrogen_in_valve on/off\r\n");
  pc_print("ov1/ov0  hydrogen_out_valve on/off\r\n");
  pc_print("stop     all off then reset\r\n\r\n");

  while (1)
  {
    uint8_t ch;
    if (pc_get_char_nonblock(&ch))
    {
      if (ch == '\r')
      {
      }
      else if (ch == '\n')
      {
        cmd_buf[cmd_len] = 0;
        if (cmd_len > 0) handle_command(cmd_buf);
        cmd_len = 0;
      }
      else
      {
        if (cmd_len < (sizeof(cmd_buf) - 1))
          cmd_buf[cmd_len++] = (char)ch;
        else
          cmd_len = 0;
      }
    }

    HAL_Delay(5);
  }
}

static void pc_print(const char *s)
{
  HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void pc_printf(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}

static int pc_get_char_nonblock(uint8_t *ch)
{
  return (HAL_UART_Receive(&hcom_uart[COM1], ch, 1, 0) == HAL_OK) ? 1 : 0;
}

static void relays_init_state(void)
{
  HAL_GPIO_WritePin(PUMP_GPIO_Port,        PUMP_Pin,        RELAY_OFF);
  HAL_GPIO_WritePin(PUMP_SEA_GPIO_Port,    PUMP_SEA_Pin,    RELAY_OFF);
  HAL_GPIO_WritePin(HEATER_GPIO_Port,      HEATER_Pin,      RELAY_OFF);
  HAL_GPIO_WritePin(CBV_SEA_PWR_GPIO_Port, CBV_SEA_PWR_Pin, RELAY_OFF);
  HAL_GPIO_WritePin(CBV_SEA_GPIO_Port,     CBV_SEA_Pin,     RELAY_OFF);
  HAL_GPIO_WritePin(O2_COMP_GPIO_Port,     O2_COMP_Pin,     RELAY_OFF);
  HAL_GPIO_WritePin(H2_IN_GPIO_Port,       H2_IN_Pin,       RELAY_OFF);
  HAL_GPIO_WritePin(H2_OUT_GPIO_Port,      H2_OUT_Pin,      RELAY_OFF);
}

static void waitForStart(void)
{
  pc_print("Press 'S' to start\r\n");
  while (!systemStarted)
  {
    uint8_t ch;
    if (pc_get_char_nonblock(&ch))
    {
      if (ch == 'S' || ch == 's')
      {
        systemStarted = true;
        pc_print("Start\r\n");
      }
    }
    HAL_Delay(50);
  }
}

static void handle_command(const char *cmd)
{
  if (!strcmp(cmd, "stop"))
  {
    pc_print("stopping\r\n");
    relays_init_state();
    HAL_Delay(100);
    NVIC_SystemReset();
  }
  else if (!strcmp(cmd, "p1"))  HAL_GPIO_WritePin(PUMP_GPIO_Port,     PUMP_Pin,     RELAY_ON);
  else if (!strcmp(cmd, "p0"))  HAL_GPIO_WritePin(PUMP_GPIO_Port,     PUMP_Pin,     RELAY_OFF);

  else if (!strcmp(cmd, "ps1")) HAL_GPIO_WritePin(PUMP_SEA_GPIO_Port, PUMP_SEA_Pin, RELAY_ON);
  else if (!strcmp(cmd, "ps0")) HAL_GPIO_WritePin(PUMP_SEA_GPIO_Port, PUMP_SEA_Pin, RELAY_OFF);

  else if (!strcmp(cmd, "h1"))  HAL_GPIO_WritePin(HEATER_GPIO_Port,   HEATER_Pin,   RELAY_ON);
  else if (!strcmp(cmd, "h0"))  HAL_GPIO_WritePin(HEATER_GPIO_Port,   HEATER_Pin,   RELAY_OFF);

  else if (!strcmp(cmd, "csp1")) HAL_GPIO_WritePin(CBV_SEA_PWR_GPIO_Port, CBV_SEA_PWR_Pin, RELAY_ON);
  else if (!strcmp(cmd, "csp0")) HAL_GPIO_WritePin(CBV_SEA_PWR_GPIO_Port, CBV_SEA_PWR_Pin, RELAY_OFF);

  else if (!strcmp(cmd, "cs1"))  HAL_GPIO_WritePin(CBV_SEA_GPIO_Port, CBV_SEA_Pin, RELAY_ON);
  else if (!strcmp(cmd, "cs0"))  HAL_GPIO_WritePin(CBV_SEA_GPIO_Port, CBV_SEA_Pin, RELAY_OFF);

  else if (!strcmp(cmd, "c1"))   HAL_GPIO_WritePin(O2_COMP_GPIO_Port, O2_COMP_Pin, RELAY_ON);
  else if (!strcmp(cmd, "c0"))   HAL_GPIO_WritePin(O2_COMP_GPIO_Port, O2_COMP_Pin, RELAY_OFF);

  else if (!strcmp(cmd, "iv1"))  HAL_GPIO_WritePin(H2_IN_GPIO_Port,   H2_IN_Pin,   RELAY_ON);
  else if (!strcmp(cmd, "iv0"))  HAL_GPIO_WritePin(H2_IN_GPIO_Port,   H2_IN_Pin,   RELAY_OFF);

  else if (!strcmp(cmd, "ov1"))  HAL_GPIO_WritePin(H2_OUT_GPIO_Port,  H2_OUT_Pin,  RELAY_ON);
  else if (!strcmp(cmd, "ov0"))  HAL_GPIO_WritePin(H2_OUT_GPIO_Port,  H2_OUT_Pin,  RELAY_OFF);

  else
  {
    pc_print("Unknown command\r\n");
  }
}

/* Clock and Error handlers: keep your existing implementation */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
