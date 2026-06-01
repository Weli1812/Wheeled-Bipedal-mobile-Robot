/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ReferenceGeneratorxLQRCombinedxPWMGenerator.h" // Auto-generated Simulink header
#include <string.h> // Required for memcpy
#include <math.h>   // Required for fabs
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Packet layout (must match Mixedmodel.ino exactly):
//   [0]     0xAA  sync byte 0
//   [1]     0x55  sync byte 1
//   [2..5]  x_robot   (float32 LE)
//   [6..9]  y_robot   (float32 LE)
//   [10..13] phi      (float32 LE, radians, pitch offset already subtracted)
//   [14..17] s        (float32 LE, arc-length metres)
//   [18..21] theta    (float32 LE, yaw radians, tared at boot)
//   [22..25] v        (float32 LE, forward velocity m/s)
//   [26..29] omega    (float32 LE, yaw rate rad/s, bias corrected)
//   [30..33] phi_dot  (float32 LE, pitch rate rad/s, bias corrected)
//   [34]    XOR checksum over bytes [2..33]
#define PACKET_SIZE    35    // 2 sync + 32 payload + 1 checksum
#define PAYLOAD_START   2    // index of first payload byte
#define PAYLOAD_BYTES  32    // 8 floats × 4 bytes
#define CHECKSUM_IDX   34    // index of checksum byte

// Ring buffer: 4 × PACKET_SIZE gives 20 ms headroom at 200 Hz.
// Must be a power-of-two multiple of PACKET_SIZE for clean wrapping;
// 4×35 = 140 bytes is fine since we use modulo arithmetic.
#define RX_BUFFER_SIZE 140
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
// ==========================================================================
// DMA RX RING BUFFER — placed at fixed AXI SRAM address
// ==========================================================================
// STM32H7 has a split-cache architecture: the CPU's D-Cache and the DMA bus
// master access RAM independently.  If the buffer were in cached RAM the CPU
// could read stale cache lines while DMA has already written new bytes.
//
// Solution: map the buffer to the AXI SRAM bank (0x24000000) and configure
// the MPU to mark that region as Non-Cacheable (see MPU_Config()).  The CPU
// then always reads directly from the physical RAM — guaranteed coherency.
//
// *** LINKER WARNING ***
// This pointer is hardcoded to 0x24000000.  The STM32CubeIDE linker script
// places the default heap/stack in DTCM (0x20000000), so there is no
// collision in the default configuration.  If you ever add a custom RAM
// section at 0x24000000 in the linker script, a collision WILL occur.
// A safer long-term approach is to declare the buffer with
//   __attribute__((section(".dma_buffers")))
// and add a dedicated no-cache section in the linker script.
uint8_t * const uart_rx_buffer = (uint8_t*)0x24000000;
uint16_t dma_tail = 0;   // Ring-buffer CPU read pointer

// data_ready: set to 1 when a valid packet has been parsed and rtU is fresh.
// Cleared immediately at the top of the control execution block.
volatile uint8_t  data_ready = 0;
// last_packet_time: HAL tick (ms) when the last valid packet was accepted.
// Used by the 100 ms communication watchdog.
volatile uint32_t last_packet_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MPU_Config(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 1. Error Recovery Callback (Catches Overrun/Framing Errors safely)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 1. Clear all UART error flags (ORE, NE, FE, PE)
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    // 2. Restart the Circular DMA
    HAL_UART_Receive_DMA(huart, uart_rx_buffer, RX_BUFFER_SIZE);
  }
}
/* USER CODE END 0 */

/**
  * @brief The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Caches */
  SCB_EnableICache();
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // -------------------------------------------------------------------------
  // STEP 1 — SAFETY: disable motors immediately at startup
  // -------------------------------------------------------------------------
  // PE3 drives the motor-driver EN pin (active HIGH = motors enabled).
  // Hold LOW for 5 s so the robot can be placed upright before the control
  // loop starts commanding PWM.  This delay runs BEFORE any UART activity.
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(5000);   // 5-second safe-placement window

  // STEP 2 — Initialise the Simulink-generated LQR controller
  ReferenceGeneratorxLQRCombinedxPWMGenerator_initialize();

  // STEP 3 — Give the ESP32 time to boot, then arm the DMA listener
  // The ESP32 takes ~200-500 ms from power-on before it starts transmitting.
  // Starting DMA here (after the 5 s delay) means the ESP32 is already
  // transmitting and the DMA buffer fills from the first available packet.
  HAL_Delay(500);
  HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUFFER_SIZE);

  // -------------------------------------------------------------------------
  // STEP 4 — BLOCKING WAIT for first valid packet (self-healing)
  // -------------------------------------------------------------------------
  // The robot must not move until a valid state has been received from the
  // ESP32.  This loop polls the DMA ring buffer.  If no packet arrives within
  // 200 ms it aborts and restarts DMA (guards against a silent DMA hang).
  uint32_t wait_start = HAL_GetTick();

  while (data_ready == 0)
  {
    HAL_Delay(10);   // yield — prevents tight-loop lockup of HAL tick

    // --- Ring-buffer head calculation ---
    // DMA NDTR counts DOWN from RX_BUFFER_SIZE to 0 (then reloads in circular
    // mode).  head = bytes written = RX_BUFFER_SIZE − NDTR.
    uint16_t dma_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    uint16_t available = (dma_head >= dma_tail)
                         ? (dma_head - dma_tail)
                         : (RX_BUFFER_SIZE - dma_tail + dma_head);

    while (available >= PACKET_SIZE)
    {
      uint8_t b0 = uart_rx_buffer[dma_tail];
      uint8_t b1 = uart_rx_buffer[(dma_tail + 1) % RX_BUFFER_SIZE];

      if (b0 == 0xAA && b1 == 0x55)
      {
        // Possible packet start — extract a local copy for CRC check
        uint8_t packet[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++)
          packet[i] = uart_rx_buffer[(dma_tail + i) % RX_BUFFER_SIZE];

        // XOR checksum over payload bytes [PAYLOAD_START .. CHECKSUM_IDX-1]
        uint8_t calc_crc = 0;
        for (int i = PAYLOAD_START; i < CHECKSUM_IDX; i++)
          calc_crc ^= packet[i];

        if (calc_crc == packet[CHECKSUM_IDX])
        {
          // Valid packet — unpack the 8 float32 state values
          float temp_states[8];
          memcpy(temp_states, &packet[PAYLOAD_START], PAYLOAD_BYTES);

          // Map to Simulink external inputs (same order as ESP32 payload[])
          rtU.x_robot = (double)temp_states[0];  // world X (m)
          rtU.y_robot = (double)temp_states[1];  // world Y (m)
          rtU.phi     = (double)temp_states[2];  // pitch (rad)
          rtU.s       = (double)temp_states[3];  // arc-length (m)
          rtU.theta   = (double)temp_states[4];  // yaw (rad)
          rtU.v       = (double)temp_states[5];  // forward velocity (m/s)
          rtU.omega   = (double)temp_states[6];  // yaw rate (rad/s)
          rtU.phi_dot = (double)temp_states[7];  // pitch rate (rad/s)

          data_ready = 1;
          dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
          available -= PACKET_SIZE;
          continue;   // parse any additional packets in the buffer
        }
        else
        {
          // Header matched but CRC failed — this is a corrupted packet.
          // Skip past the entire expected packet boundary so the next parse
          // attempt starts where the *next* packet should begin.
          // (Sliding by 1 would re-examine 34 already-bad bytes.)
          dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
          available = (available >= PACKET_SIZE) ? available - PACKET_SIZE : 0;
        }
      }
      else
      {
        // No valid header at this position — slide forward 1 byte to search
        dma_tail = (dma_tail + 1) % RX_BUFFER_SIZE;
        available--;
      }
    }

    // Self-healing: if DMA silently stalled, abort and restart every 200 ms
    if (HAL_GetTick() - wait_start > 200)
    {
      HAL_UART_AbortReceive(&huart1);
      dma_tail = 0;   // reset tail — new DMA starts writing from index 0
      HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUFFER_SIZE);
      wait_start = HAL_GetTick();
    }
  }

  // -------------------------------------------------------------------------
  // STEP 5 — First valid packet received: arm the system
  // -------------------------------------------------------------------------
  data_ready = 0;                         // clear flag; main loop re-sets it
  last_packet_time = HAL_GetTick();       // seed watchdog timestamp

  // Enable motors now that the ESP32 is confirmed alive and rtU is populated
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  // Start PWM outputs
  // PWM frequency = TIMx_CLK / ((Prescaler+1) * Period)
  //               = 240 MHz / (6 * 4000) = 10 kHz
  // Timer range 0..4000; 0 = 0 % duty, 4000 = 100 % duty.
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Right motor RPWM  — PA9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // Right motor LPWM  — PA10
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // Left  motor RPWM  — PC6
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // Left  motor LPWM  — PC7

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // =========================================================================
    // BLOCK A — Parse DMA ring buffer for incoming packets
    // =========================================================================
    uint16_t dma_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    uint16_t available = (dma_head >= dma_tail)
                         ? (dma_head - dma_tail)
                         : (RX_BUFFER_SIZE - dma_tail + dma_head);

    while (available >= PACKET_SIZE)
    {
      uint8_t b0 = uart_rx_buffer[dma_tail];
      uint8_t b1 = uart_rx_buffer[(dma_tail + 1) % RX_BUFFER_SIZE];

      if (b0 == 0xAA && b1 == 0x55)
      {
        // Valid header found — extract a local copy for CRC check
        uint8_t packet[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++)
          packet[i] = uart_rx_buffer[(dma_tail + i) % RX_BUFFER_SIZE];

        uint8_t calc_crc = 0;
        for (int i = PAYLOAD_START; i < CHECKSUM_IDX; i++)
          calc_crc ^= packet[i];

        if (calc_crc == packet[CHECKSUM_IDX])
        {
          // CRC OK — unpack the 8 float32 state values into Simulink inputs
          float temp_states[8];
          memcpy(temp_states, &packet[PAYLOAD_START], PAYLOAD_BYTES);

          rtU.x_robot = (double)temp_states[0];  // world X (m)
          rtU.y_robot = (double)temp_states[1];  // world Y (m)
          rtU.phi     = (double)temp_states[2];  // pitch (rad)
          rtU.s       = (double)temp_states[3];  // arc-length (m)
          rtU.theta   = (double)temp_states[4];  // yaw (rad)
          rtU.v       = (double)temp_states[5];  // forward velocity (m/s)
          rtU.omega   = (double)temp_states[6];  // yaw rate (rad/s)
          rtU.phi_dot = (double)temp_states[7];  // pitch rate (rad/s)

          data_ready = 1;
          dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
          available -= PACKET_SIZE;
          continue;   // immediately try to parse the next packet
        }
        else
        {
          // Header matched but CRC failed — corrupted packet.
          // Advance by a full PACKET_SIZE: the next legitimate packet starts
          // PACKET_SIZE bytes after the current (bad) header, not just 1 byte.
          // Sliding by 1 would cause 34 redundant re-parse iterations per error.
          dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
          available = (available >= PACKET_SIZE) ? available - PACKET_SIZE : 0;
        }
      }
      else
      {
        // No sync bytes at current position — slide 1 byte to search for header
        dma_tail = (dma_tail + 1) % RX_BUFFER_SIZE;
        available--;
      }
    }

    // =========================================================================
    // BLOCK B — Execute control step on fresh data
    // =========================================================================
    if (data_ready == 1)
    {
      data_ready = 0;
      last_packet_time = HAL_GetTick();   // reset watchdog

      // -----------------------------------------------------------------------
      // FALL DETECTION — disable motors if pitch exceeds safe range
      // -----------------------------------------------------------------------
      // 0.7 rad ≈ 40°.  Beyond this the LQR cannot recover balance.
      // Disabling the motor driver (PE3 LOW) immediately cuts all torque and
      // prevents the motor driver from latching in an overcurrent state.
      if (fabs(rtU.phi) > 0.7)
      {
        // Robot has fallen — kill motors
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
      }
      else
      {
        // Robot is upright — enable motors and run one LQR step
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

        // Run Simulink-generated LQR + path-following + PWM generation
        ReferenceGeneratorxLQRCombinedxPWMGenerator_step();

        // Apply PWM outputs to timers
        // RPWM_x > 0 drives motor forward; LPWM_x > 0 drives motor reverse.
        // Both are always ≥ 0 (split-rail PWM scheme).
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)rtY.RPWM_R);  // Right fwd
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)rtY.LPWM_R);  // Right rev
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)rtY.LPWM_L);  // Left  fwd
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)rtY.RPWM_L);  // Left  rev
      }
    }
    else
    {
      // -----------------------------------------------------------------------
      // COMMUNICATION WATCHDOG — 100 ms silence → disable motors
      // -----------------------------------------------------------------------
      // At 200 Hz, a packet arrives every 5 ms.  100 ms = 20 missed packets,
      // which is a reliable indicator that the ESP32 has crashed or the UART
      // link has failed.  Disabling motors is the safe response.
      if (HAL_GetTick() - last_packet_time > 100)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 5;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** * Sets AXI SRAM (0x24000000) specifically to NOT CACHEABLE
   * Allows the rest of the 4GB space to benefit from global CPU cache.
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x24000000;
  // CONFLICT RESOLUTION: Increased to 256B because your new RX_BUFFER_SIZE is 140 bytes
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief Period elapsed callback in non blocking mode
  * @note This function is called when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

/**

* @brief Reports the name of the source file and the source line number

* where the assert_param error has occurred.

* @param file: pointer to the source file

name

* @param line: assert_param error line

source number

* @retval None

*/

void assert_failed(uint8_t *file, uint32_t line)

{

/* USER CODE BEGIN 6 */

/* User can add his own implementation

to report the file name and line number,

ex: printf("Wrong parameters value: file

%s on line %d\r\n", file, line) */

/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
