/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SteadyWin GIM6010-48 RS485 Motor Control (Speed Mode)
  * Protocol : V3.03b0 (Standard RS485 Frame)
  * MCU      : STM32H750VBT6 @ 480 MHz
  * UART     : USART1, 115200 baud, RS485 hardware DE
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>

/* ============================================================================
 * USER CONFIGURATION
 * ============================================================================ */
#define HSE_FREQ_MHZ        25
#define USART1_TX_PORT      GPIOA
#define USART1_TX_PIN       GPIO_PIN_9
#define USART1_TX_AF        GPIO_AF7_USART1

#define USART1_RX_PORT      GPIOA
#define USART1_RX_PIN       GPIO_PIN_10
#define USART1_RX_AF        GPIO_AF7_USART1

#define USART1_DE_PORT      GPIOA
#define USART1_DE_PIN       GPIO_PIN_12
#define USART1_DE_AF        GPIO_AF7_USART1

#define MOTOR_ID            1
#define COUNTS_PER_REV      16384.0f
#define RATED_SPEED_RPM     150.0f

/* ============================================================================
 * PLL LOOKUP TABLE (Targeting 480MHz)
 * ============================================================================ */
#if   HSE_FREQ_MHZ == 25
  #define PLL_M  5
  #define PLL_N  192
#elif HSE_FREQ_MHZ == 24
  #define PLL_M  4
  #define PLL_N  160
#elif HSE_FREQ_MHZ == 16
  #define PLL_M  2
  #define PLL_N  120
#elif HSE_FREQ_MHZ == 12
  #define PLL_M  3
  #define PLL_N  240
#elif HSE_FREQ_MHZ == 8
  #define PLL_M  2
  #define PLL_N  240
#else
  #error "Unsupported HSE_FREQ_MHZ"
#endif

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */
UART_HandleTypeDef huart1;
static uint8_t packet_seq = 0;
volatile float current_rpm = 0.0f;

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

/* ============================================================================
 * V3.03 PROTOCOL IMPLEMENTATION
 * ============================================================================ */

uint16_t CRC16_Modbus(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief Builds and transmits a V3.03 compliant frame, and prevents bus collisions.
  */
static void RS485_SendCmd(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t frame[256];

    // Build Header
    frame[0] = 0xAE;
    frame[1] = packet_seq++;
    frame[2] = id;
    frame[3] = cmd;
    frame[4] = len;

    // Copy Payload
    if (len > 0 && data != NULL) {
        memcpy(&frame[5], data, len);
    }

    // Calculate CRC
    uint16_t crc = CRC16_Modbus(frame, 5 + len);
    frame[5 + len] = (uint8_t)(crc & 0xFF);
    frame[6 + len] = (uint8_t)((crc >> 8) & 0xFF);

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

    // Transmit to Motor
    HAL_UART_Transmit(&huart1, frame, 7 + len, 50);

    /* * CRITICAL FIX: Absorb the motor's automatic ACK.
     * Most commands cause the motor to reply. E.g., 0x21 replies with 29 bytes.
     * We MUST wait for and clear these bytes from the RS485 bus line, otherwise
     * our very next transmission will collide with the motor's ongoing reply.
     */
    uint8_t ack_dummy[32];
    HAL_UART_Receive(&huart1, ack_dummy, sizeof(ack_dummy), 15); // 15ms is enough for 29 bytes @ 115200
}

/* ============================================================================
 * MOTOR COMMANDS
 * ============================================================================ */

void Motor_ClearFault(uint8_t id)
{
    RS485_SendCmd(id, 0x0F, NULL, 0);
    HAL_Delay(10);
}

/**
  * @brief Command 0x21: Speed Control
  */
void Motor_SetSpeed(uint8_t id, float speed_rpm, float accel_rpm_s)
{
    uint8_t payload[8];

    // Datasheet format: 0.01 Rpm & 0.01 Rpm/s
    int32_t  speed_unit = (int32_t)(speed_rpm * 100.0f);
    uint32_t accel_unit = (uint32_t)(accel_rpm_s * 100.0f);

    // Target Speed (4 bytes, Signed 32-bit, Little Endian)
    payload[0] = (uint8_t)((speed_unit >>  0) & 0xFF);
    payload[1] = (uint8_t)((speed_unit >>  8) & 0xFF);
    payload[2] = (uint8_t)((speed_unit >> 16) & 0xFF);
    payload[3] = (uint8_t)((speed_unit >> 24) & 0xFF);

    // Acceleration (4 bytes, Unsigned 32-bit, Little Endian)
    payload[4] = (uint8_t)((accel_unit >>  0) & 0xFF);
    payload[5] = (uint8_t)((accel_unit >>  8) & 0xFF);
    payload[6] = (uint8_t)((accel_unit >> 16) & 0xFF);
    payload[7] = (uint8_t)((accel_unit >> 24) & 0xFF);

    RS485_SendCmd(id, 0x21, payload, 8);
}

float Motor_ReadRPM(uint8_t id)
{
    uint8_t frame[7];
    frame[0] = 0xAE;
    frame[1] = packet_seq++;
    frame[2] = id;
    frame[3] = 0x0B;
    frame[4] = 0x00;

    uint16_t crc = CRC16_Modbus(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

    HAL_UART_Transmit(&huart1, frame, 7, 50);

    uint8_t rx_buf[29];
    if (HAL_UART_Receive(&huart1, rx_buf, 29, 20) == HAL_OK)
    {
        if (rx_buf[0] == 0xAC && rx_buf[3] == 0x0B)
        {
            int32_t speed_raw;
            memcpy(&speed_raw, &rx_buf[11], 4);

            return (float)speed_raw / 100.0f;
        }
    }

    return current_rpm;
}

/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void)
{
    MPU_Config();
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    HAL_Delay(1500);

    Motor_ClearFault(MOTOR_ID);
    HAL_Delay(100);

    while (1)
    {
        // 1. Move Forward (Using 0.0f acceleration defaults to hardware maximum)
        Motor_SetSpeed(MOTOR_ID, RATED_SPEED_RPM, 0.0f);

        // 2. Continously ask for RPM. The bus collision is now resolved.
        for(int i = 0; i < 30; i++) {
            current_rpm = Motor_ReadRPM(MOTOR_ID);
            HAL_Delay(100);
        }

        // 3. Move Reverse
        Motor_SetSpeed(MOTOR_ID, -RATED_SPEED_RPM, 0.0f);

        for(int i = 0; i < 30; i++) {
            current_rpm = Motor_ReadRPM(MOTOR_ID);
            HAL_Delay(100);
        }
    }
}

/* ============================================================================
 * PERIPHERAL INITIALISATION
 * ============================================================================ */

static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TX */
    GPIO_InitStruct.Pin       = USART1_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_TX_AF;
    HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

    /* RX */
    GPIO_InitStruct.Pin       = USART1_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_RX_AF;
    HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

    /* DE */
    GPIO_InitStruct.Pin       = USART1_DE_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART1_DE_AF;
    HAL_GPIO_Init(USART1_DE_PORT, &GPIO_InitStruct);

    /* USART1 Configured to 115200 Baud as per datasheet */
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
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

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
        for (volatile uint32_t i = 0; i < 4800000UL; i++);
    }
}
