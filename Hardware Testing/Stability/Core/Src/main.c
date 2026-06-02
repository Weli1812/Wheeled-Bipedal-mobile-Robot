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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Simulink.h" // Auto-generated Simulink header
#include <string.h>   // Required for memcpy
#include <math.h>     // Required for fabs
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 252  // Fits exactly 4 packets of 63 bytes
#define PACKET_SIZE 63      // 2 Header + 60 Payload (15 floats) + 1 Checksum
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
// 1. STM32H7 CRITICAL: Force buffer to AXI SRAM so DMA and CPU Cache don't fight
uint8_t * const uart_rx_buffer = (uint8_t*)0x24000000;
uint16_t dma_tail = 0; // Ring buffer read pointer

volatile uint8_t data_ready = 0;
volatile uint32_t last_packet_time = 0;

// --- Dynamic Tuning Variables ---
volatile float esp_wheel_speed_r = 0.0f; // NEW: Right wheel speed from ESP
volatile float esp_wheel_speed_l = 0.0f; // NEW: Left wheel speed from ESP
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
    dma_tail = 0;
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

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. SAFETY: Override CubeMX init and explicitly pull PE3 LOW to disable motors
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(5000);

  // 2. Initialize the Simulink controller
  Simulink_initialize();

  // 3. Wait for ESP32 to boot, then kick off the DMA listener
  HAL_Delay(500);
  HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUFFER_SIZE);

  // 4. BLOCKING WAIT with self-healing: restart DMA periodically if no packet arrives
  uint32_t wait_start = HAL_GetTick();

  while(data_ready == 0)
  {
      HAL_Delay(10); // Prevent tight loop lockup

      uint16_t dma_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      uint16_t available = (dma_head >= dma_tail) ? (dma_head - dma_tail) : (RX_BUFFER_SIZE - dma_tail + dma_head);

      while(available >= PACKET_SIZE)
      {
          uint8_t b1 = uart_rx_buffer[dma_tail];
          uint8_t b2 = uart_rx_buffer[(dma_tail + 1) % RX_BUFFER_SIZE];

          if (b1 == 0xAA && b2 == 0x55)
          {
              uint8_t packet[PACKET_SIZE];
              for(int i = 0; i < PACKET_SIZE; i++)
              {
                  packet[i] = uart_rx_buffer[(dma_tail + i) % RX_BUFFER_SIZE];
              }

              // Calculate XOR Checksum over the 60 bytes of payload (indices 2 through 61)
                            uint8_t calc_crc = 0;
                            for(int i = 2; i < 62; i++)
                            {
                                calc_crc ^= packet[i];
                            }

                            // Check against byte 62 (the 63rd byte)
                            if (calc_crc == packet[62])
                            {
                            	float payload[15]; // Holds 15 floats
                            	    memcpy(payload, &packet[2], 60);

                            	    for(int i = 0; i < 6; i++) {
                            	        rtU.state_x[i] = (double)payload[i];
                            	        // Indices 6 through 11 (K_gains) are now ignored
                            	    }
                            	    // Index 12 (KI_PHI) is now ignored

                            	    esp_wheel_speed_r = payload[13]; // Dynamically updated right wheel speed
                            	    esp_wheel_speed_l = payload[14]; // Dynamically updated left wheel speed

                                data_ready = 1;
                                dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
                                available -= PACKET_SIZE;
                                continue;
                            }
          }
          // Bad header or CRC: slide window by 1 byte
          dma_tail = (dma_tail + 1) % RX_BUFFER_SIZE;
          available--;
      }

      // Self-healing block: Every 200ms, abort and restart DMA in case it silently died
      if (HAL_GetTick() - wait_start > 200)
      {
          HAL_UART_AbortReceive(&huart1);
          dma_tail = 0; // CRITICAL: Reset circular buffer tail to match new DMA Head
          HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUFFER_SIZE);
          wait_start = HAL_GetTick();
      }
  }

  // ESP32 IS CONNECTED! Clear the flag from the first packet
  data_ready = 0;
  last_packet_time = HAL_GetTick(); // Initialize watchdog timestamp

  // 5. ENABLE MOTORS: Pull PE3 HIGH now that we have valid state data
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  // 6. Start the Hardware Timers for PWM Output
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Right Motor RPWM (PA9)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Right Motor LPWM (PA10)
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Left Motor RPWM (PC6)
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Left Motor LPWM (PC7)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. Process Ring Buffer for new packets
      uint16_t dma_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      uint16_t available = (dma_head >= dma_tail) ? (dma_head - dma_tail) : (RX_BUFFER_SIZE - dma_tail + dma_head);

      while(available >= PACKET_SIZE)
      {
          uint8_t b1 = uart_rx_buffer[dma_tail];
          uint8_t b2 = uart_rx_buffer[(dma_tail + 1) % RX_BUFFER_SIZE];

          if (b1 == 0xAA && b2 == 0x55)
          {
              uint8_t packet[PACKET_SIZE];
              for(int i = 0; i < PACKET_SIZE; i++)
              {
                  packet[i] = uart_rx_buffer[(dma_tail + i) % RX_BUFFER_SIZE];
              }

              // Calculate XOR Checksum over the 60 bytes of payload (indices 2 through 61)
              uint8_t calc_crc = 0;
              for(int i = 2; i < 62; i++)
              {
                  calc_crc ^= packet[i];
              }

              // Check against byte 62 (the 63rd byte)
              if (calc_crc == packet[62])
              {
                  float payload[15]; // Holds 15 floats
                  memcpy(payload, &packet[2], 60);

                  for(int i = 0; i < 6; i++) {
                                        rtU.state_x[i] = (double)payload[i];
                                        // Indices 6 through 11 (K_gains) are now ignored
                                    }
                                    // Index 12 (KI_PHI) is now ignored
                  esp_wheel_speed_r = payload[13]; // Dynamically updated wheel speed
                  esp_wheel_speed_l = payload[14]; // Dynamically updated wheel speed

                  data_ready = 1;
                  dma_tail = (dma_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
                  available -= PACKET_SIZE;
                  continue;
              }
          }
          dma_tail = (dma_tail + 1) % RX_BUFFER_SIZE;
          available--;
      }

      // 2. Control Loop Execution
      // ... (Keep your existing data_ready == 1 control loop logic below here)
      if (data_ready == 1)
      {
          // Acknowledge the flag and update timestamp
          data_ready = 0;
          last_packet_time = HAL_GetTick();

          // -------------------------------------------------------------
          // FALL DETECTION: Check if pitch is outside controllable range
          // -------------------------------------------------------------
          if (fabs(rtU.state_x[0]) > 0.52)
          {
              // ROBOT HAS FALLEN: Disable motor drivers globally
              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

              // Force duty cycles to 0
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
              Simulink_reset_integrator();
          }
                    else
                    {
                        // ROBOT IS UPRIGHT: Re-enable motors and balance
                        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

                        // 1. Run the LQR Control Matrix calculation
                        Simulink_step();

                        // 2. Apply generated PWM to hardware timers dynamically
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)rtY.RPWM_R);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)rtY.RPWM_R1);
                        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)rtY.RPWM_R3);
                        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)rtY.RPWM_R2);

                    }
      }
      else
      {
          // Failsafe Watchdog: If no packet received for > 100ms, shutdown motors
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

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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

  HAL_RCC_EnableCSS();
}

/**
  * @brief TIM1 Initialization Function
  */
void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM8 Initialization Function
  */
void MX_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
  HAL_TIM_MspPostInit(&htim8);
}

/**
  * @brief USART1 Initialization Function
  */
void MX_USART1_UART_Init(void)
{
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
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
}

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
  // SIZE MUST BE LARGER THAN 220 BYTES TO ENCAPSULATE NEW RING BUFFER
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
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
}

/**
  * @brief This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
