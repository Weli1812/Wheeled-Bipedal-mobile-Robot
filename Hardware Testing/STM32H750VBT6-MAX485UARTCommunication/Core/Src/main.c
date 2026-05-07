/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SteadyWin GIM6010-48 RS485 Motor Control
  *                   Protocol : SteadyWin GIM Driver Protocol Spec rev2.2
  *                   MCU      : STM32H750VBT6 @ 480 MHz
  *                   UART     : USART1, 921600 baud, RS485 hardware DE
  *                   LED      : PE3 — blinks every ~3 s in main loop,
  *                              rapid blink = stuck in Error_Handler (fault)
  ******************************************************************************
  *
  * RS485 Frame Structure (SteadyWin GIM Protocol rev2.2):
  *
  *   [ HEADER 4 bytes ][ PAYLOAD 8 bytes ]  = 12 bytes total
  *
  *   Header:
  *     Byte 0 : Motor ID
  *     Byte 1 : Frame Type  0x00 = normal, 0x01 = ACK
  *     Byte 2 : Payload len always 0x08
  *     Byte 3 : CRC = Byte0 XOR Byte1 XOR Byte2
  *
  *   Payload (8 bytes, LSB first):
  *     Byte 0 : COMMAND
  *     Byte 1-7 : command-specific data
  *
  * Command Table:
  *   0x91  Start Motor
  *   0x92  Stop Motor
  *   0x93  Torque Control
  *   0x94  Speed Control
  *   0x95  Position Control
  *   0x97  Stop Control
  *   0xA1  Modify Parameter
  *   0xA2  Retrieve Parameter
  *   0xB1  Get Version
  *   0xB2  Get Fault
  *   0xB3  Acknowledge Fault
  *   0xB4  Retrieve Indicator
  *   0x81  Reset Configuration
  *   0x82  Refresh Configuration
  *   0x83  Modify Configuration
  *   0x84  Retrieve Configuration
  *
  * Fault Codes (Get Fault response Byte1):
  *   0x00  No Fault
  *   0x01  FoC Frequency Too High
  *   0x02  Over Voltage
  *   0x04  Under Voltage
  *   0x08  Over Temperature
  *   0x10  Start Failure
  *   0x40  Over Current
  *   0x80  Software Exception
  *
  ******************************************************************************
  *
  * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  * ACTION REQUIRED BEFORE FLASHING:
  *
  * 1. Set HSE_FREQ_MHZ to your board's actual crystal frequency.
  *    Wrong HSE = wrong baud rate = motor never responds.
  *    Check PCB silkscreen, schematic, or crystal body marking.
  *
  * 2. Set USART1_TX/RX/DE port+pin defines to match your wiring:
  *      MAX485 DI  <- STM32 USART1 TX
  *      MAX485 RO  -> STM32 USART1 RX
  *      MAX485 DE  <- STM32 USART1 DE  (tie MAX485 RE low or bridge to DE)
  *    Check schematic or use continuity meter from STM32 pads to MAX485.
  * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * USER CONFIGURATION — EDIT THESE TO MATCH YOUR HARDWARE
 * ============================================================================ */

/* --- HSE crystal frequency in MHz ---
 * Common values: 8, 12, 16, 24, 25
 * This drives the PLL multiplier table below.                               */
#define HSE_FREQ_MHZ        25      /* <-- SET THIS                           */

/* --- USART1 GPIO pin assignments ---
 * Common USART1 TX/RX alternate function pins on STM32H750:
 *   PA9  / PA10   AF7   (most common default)
 *   PB6  / PB7    AF7
 *   PB14 / PB15   AF4
 *
 * Common DE pin options:
 *   PA12          AF8   (most common for RS485 DE)
 *   PB0           AF4
 *
 * Change the four PORT/PIN defines below to match your schematic.           */
#define USART1_TX_PORT      GPIOA
#define USART1_TX_PIN       GPIO_PIN_9
#define USART1_TX_AF        GPIO_AF7_USART1

#define USART1_RX_PORT      GPIOA
#define USART1_RX_PIN       GPIO_PIN_10
#define USART1_RX_AF        GPIO_AF7_USART1

#define USART1_DE_PORT      GPIOA
#define USART1_DE_PIN       GPIO_PIN_12
#define USART1_DE_AF        GPIO_AF7_USART1   /* PA12=AF7 for USART1_DE on H7 series */

/* --- Motor ID ---
 * 0 = broadcast — all motors on bus respond, no reply expected.
 * Use 0 first to confirm comms, then switch to your motor's actual ID.      */
#define MOTOR_ID            0       /* broadcast until comms confirmed        */

/* ============================================================================
 * PLL LOOKUP TABLE — auto-selected from HSE_FREQ_MHZ
 * All combinations target SYSCLK = 480 MHz  (VCO = 960 MHz, PLLP = 2)
 * ============================================================================ */
#if   HSE_FREQ_MHZ == 25
  #define PLL_M  5
  #define PLL_N  192    /* 25/5  * 192 = 960 MHz */
#elif HSE_FREQ_MHZ == 24
  #define PLL_M  4
  #define PLL_N  160    /* 24/4  * 160 = 960 MHz */
#elif HSE_FREQ_MHZ == 16
  #define PLL_M  2
  #define PLL_N  120    /* 16/2  * 120 = 960 MHz */
#elif HSE_FREQ_MHZ == 12
  #define PLL_M  3
  #define PLL_N  240    /* 12/3  * 240 = 960 MHz */
#elif HSE_FREQ_MHZ == 8
  #define PLL_M  2
  #define PLL_N  240    /* 8/2   * 240 = 960 MHz */
#else
  #error "Unsupported HSE_FREQ_MHZ — add your value to the PLL lookup table above."
#endif

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */
#define RS485_TX_TIMEOUT_MS  50
#define RS485_RX_TIMEOUT_MS  30
#define PAYLOAD_LEN          8
#define FRAME_LEN            12    /* 4-byte header + 8-byte payload          */

/* Fault bit masks */
#define FAULT_NONE           0x00
#define FAULT_FOC_FREQ       0x01
#define FAULT_OVER_VOLTAGE   0x02
#define FAULT_UNDER_VOLTAGE  0x04
#define FAULT_OVER_TEMP      0x08
#define FAULT_START_FAIL     0x10
#define FAULT_OVER_CURRENT   0x40
#define FAULT_SW_EXCEPTION   0x80

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */
UART_HandleTypeDef huart1;
static uint8_t rx_buf[FRAME_LEN];

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* ============================================================================
 * RS485 LOW-LEVEL FRAMING
 * ============================================================================ */

/**
  * @brief  Build and send a 12-byte SteadyWin RS485 frame.
  *
  *   Frame: [ motor_id | 0x00 | 0x08 | XOR-CRC ] [ 8-byte payload ]
  *
  * @param  motor_id  Target motor ID (0 = broadcast)
  * @param  payload   8 bytes: payload[0] = COMMAND, payload[1..7] = data
  */
static void RS485_Send(uint8_t motor_id, const uint8_t *payload)
{
    uint8_t frame[FRAME_LEN];
    frame[0] = motor_id;
    frame[1] = 0x00;
    frame[2] = PAYLOAD_LEN;
    frame[3] = frame[0] ^ frame[1] ^ frame[2];   /* XOR CRC of header bytes  */
    memcpy(&frame[4], payload, PAYLOAD_LEN);
    HAL_UART_Transmit(&huart1, frame, FRAME_LEN, RS485_TX_TIMEOUT_MS);
}

/**
  * @brief  Send a command and optionally receive the 12-byte response.
  * @param  motor_id  Target motor ID
  * @param  payload   8-byte command payload
  * @param  response  12-byte buffer for response, or NULL to skip
  */
static HAL_StatusTypeDef RS485_SendReceive(uint8_t motor_id,
                                           const uint8_t *payload,
                                           uint8_t *response)
{
    RS485_Send(motor_id, payload);
    if (response == NULL) return HAL_OK;
    HAL_Delay(2);
    return HAL_UART_Receive(&huart1, response, FRAME_LEN, RS485_RX_TIMEOUT_MS);
}

/* ============================================================================
 * MOTOR COMMANDS
 * ============================================================================ */

/**
  * @brief  Acknowledge Fault (0xB3).
  *         MUST be sent to unlock the driver after any fault is latched.
  *         The driver will silently ignore ALL other commands while faulted.
  */
void Motor_AckFault(uint8_t id)
{
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0xB3;
    RS485_Send(id, payload);
    HAL_Delay(50);
}

/**
  * @brief  Start Motor (0x91).
  *         Enables the drive stage. Motor will now accept control commands.
  */
void Motor_Start(uint8_t id)
{
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x91;
    RS485_Send(id, payload);
    HAL_Delay(100);
}

/**
  * @brief  Stop Motor (0x92).
  *         Disables the drive stage and de-energises windings.
  */
void Motor_Stop(uint8_t id)
{
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x92;
    RS485_Send(id, payload);
}

/**
  * @brief  Stop Control (0x97).
  *         Aborts the current motion profile. Motor stays in running state.
  */
void Motor_StopControl(uint8_t id)
{
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x97;
    RS485_Send(id, payload);
}

/**
  * @brief  Get Fault status (0xB2).
  * @retval Fault byte (FAULT_xxx bitmask), or 0xFF on comms error.
  *
  * Response layout (rx_buf):
  *   [0..3]  response header
  *   [4]     0xB2  (command echo)
  *   [5]     RES   (result code)
  *   [6]     FaultNo  <-- returned value
  */
uint8_t Motor_GetFault(uint8_t id)
{
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0xB2;
    if (RS485_SendReceive(id, payload, rx_buf) != HAL_OK) return 0xFF;
    return rx_buf[6];
}

/**
  * @brief  Position Control (0x95).
  *
  *  Payload:
  *    [0]    0x95
  *    [1..4] target position, IEEE 754 float, radians, LSB first
  *    [5..7] duration, 24-bit unsigned int, milliseconds, LSB first
  *
  * @param  id           Motor ID
  * @param  position_rad Target output-shaft position in radians
  * @param  duration_ms  Time to reach target in ms  (0 = max speed)
  */
void Motor_SetPosition(uint8_t id, float position_rad, uint32_t duration_ms)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    uint8_t payload[PAYLOAD_LEN];
    payload[0] = 0x95;

    uint8_t *fp = (uint8_t *)&position_rad;
    payload[1] = fp[0];
    payload[2] = fp[1];
    payload[3] = fp[2];
    payload[4] = fp[3];

    payload[5] = (uint8_t)((duration_ms >>  0) & 0xFF);
    payload[6] = (uint8_t)((duration_ms >>  8) & 0xFF);
    payload[7] = (uint8_t)((duration_ms >> 16) & 0xFF);

    RS485_Send(id, payload);

    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

/**
  * @brief  Speed Control (0x94).
  *
  *  Payload:
  *    [0]    0x94
  *    [1..4] target speed, IEEE 754 float, RPM, LSB first  (negative = reverse)
  *    [5..7] duration, 24-bit unsigned int, ms, LSB first  (0 = run until next cmd)
  */
void Motor_SetSpeed(uint8_t id, float speed_rpm, uint32_t duration_ms)
{
    uint8_t payload[PAYLOAD_LEN];
    payload[0] = 0x94;

    uint8_t *fp = (uint8_t *)&speed_rpm;
    payload[1] = fp[0];
    payload[2] = fp[1];
    payload[3] = fp[2];
    payload[4] = fp[3];

    payload[5] = (uint8_t)((duration_ms >>  0) & 0xFF);
    payload[6] = (uint8_t)((duration_ms >>  8) & 0xFF);
    payload[7] = (uint8_t)((duration_ms >> 16) & 0xFF);

    RS485_Send(id, payload);
}

/**
  * @brief  Torque Control (0x93).
  *
  *  Payload:
  *    [0]    0x93
  *    [1..4] target torque, IEEE 754 float, N.m, LSB first  (negative = reverse)
  *    [5..7] duration, 24-bit unsigned int, ms, LSB first
  */
void Motor_SetTorque(uint8_t id, float torque_nm, uint32_t duration_ms)
{
    uint8_t payload[PAYLOAD_LEN];
    payload[0] = 0x93;

    uint8_t *fp = (uint8_t *)&torque_nm;
    payload[1] = fp[0];
    payload[2] = fp[1];
    payload[3] = fp[2];
    payload[4] = fp[3];

    payload[5] = (uint8_t)((duration_ms >>  0) & 0xFF);
    payload[6] = (uint8_t)((duration_ms >>  8) & 0xFF);
    payload[7] = (uint8_t)((duration_ms >> 16) & 0xFF);

    RS485_Send(id, payload);
}

/* ============================================================================
 * HELPERS
 * ============================================================================ */

static inline float Deg2Rad(float deg)
{
    return deg * (float)M_PI / 180.0f;
}

/**
  * @brief  Full motor init sequence.
  *
  *         Sends AckFault twice then Start with generous delays.
  *         Call once, after a 1000 ms boot wait, before any control commands.
  *
  *         The double AckFault is intentional — some units need the fault
  *         frame seen twice before the latch clears reliably over RS485.
  */
void Motor_Init(uint8_t id)
{
    Motor_AckFault(id);
    HAL_Delay(200);
    Motor_AckFault(id);
    HAL_Delay(200);
    Motor_Start(id);
    HAL_Delay(200);
}

/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void)
{
+    MPU_Config();
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* Let the motor driver board finish its power-on self-test */
    HAL_Delay(1000);

    Motor_Init(MOTOR_ID);

    /* Fault check — skip when broadcasting (ID 0 gives no response frame) */
#if MOTOR_ID != 0
    uint8_t fault = Motor_GetFault(MOTOR_ID);
    if (fault != FAULT_NONE && fault != 0xFF)
    {
        Error_Handler();  /* PE3 rapid blink = fault still latched */
    }
#endif

    /* Oscillate between 0° and +90° */
    while (1)
    {
        Motor_SetPosition(MOTOR_ID, Deg2Rad(90.0f), 2000);
        HAL_Delay(3000);

        Motor_SetPosition(MOTOR_ID, Deg2Rad(0.0f), 2000);
        HAL_Delay(3000);
    }
}

/* ============================================================================
 * PERIPHERAL INITIALISATION
 * ============================================================================ */

/**
  * @brief  USART1 — 921600 baud, RS485 hardware DE.
  *
  *         Explicitly configures TX, RX, and DE GPIO alternate functions using
  *         the pin defines at the top of this file.  This replaces whatever
  *         CubeMX may have generated and guarantees the mapping is correct.
  *
  *         Wiring checklist if motor still does not respond:
  *           [ ] MAX485 RE pin tied LOW (or bridged to DE)
  *           [ ] RS485 A/B pair not swapped between MAX485 and motor connector
  *           [ ] 120 Ω termination resistor across A–B at each end of the bus
  *           [ ] Motor ID confirmed with Motor Wizard (Type-C port on motor)
  *           [ ] HSE_FREQ_MHZ set correctly above
  */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();

    /* Enable clocks for all GPIO banks (safe to call if already enabled) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* TX */
    GPIO_InitStruct.Pin       = USART1_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_TX_AF;
    HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

    /* RX — pull-up keeps line defined when bus is idle */
    GPIO_InitStruct.Pin       = USART1_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_RX_AF;
    HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

    /* DE — pull-down keeps MAX485 in receive mode when STM32 is not driving */
    GPIO_InitStruct.Pin       = USART1_DE_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_DE_AF;
    HAL_GPIO_Init(USART1_DE_PORT, &GPIO_InitStruct);

    /* USART1 peripheral */
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 921600;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_UARTEx_DisableFifoMode(&huart1);
}

/**
  * @brief  GPIO init — PE3 status LED only.
  *         USART1 TX/RX/DE pins are configured inside MX_USART1_UART_Init().
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
  * @brief  System clock — 480 MHz via HSE + PLL.
  *         PLL_M and PLL_N are resolved at compile time from HSE_FREQ_MHZ.
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType     = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState           = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState       = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource      = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM           = PLL_M;
    RCC_OscInitStruct.PLL.PLLN           = PLL_N;
    RCC_OscInitStruct.PLL.PLLP           = 2;
    RCC_OscInitStruct.PLL.PLLQ           = 2;
    RCC_OscInitStruct.PLL.PLLR           = 2;
    RCC_OscInitStruct.PLL.PLLRGE        = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL     = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN      = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType       = RCC_CLOCKTYPE_HCLK    | RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2
                                      | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }

    HAL_RCC_EnableCSS();
}

/**
  * @brief  MPU — STM32H7 cache coherency safety region.
  */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x00000000UL;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Error Handler.
  *
  *         PE3 rapid blink (~5 Hz) = code halted here, fault still latched.
  *         This is visually distinct from the normal 3-second main loop blink.
  *
  *         To identify the exact fault:
  *           Connect Motor Wizard (Windows app) to the motor via its Type-C
  *           USB port and read the fault register directly from the driver.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
        /* Busy-loop delay (~100 ms @ 480 MHz) — HAL_Delay unavailable with IRQ off */
        for (volatile uint32_t i = 0; i < 4800000UL; i++);
    }
}
