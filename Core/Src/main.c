/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
//#include "usart.h"
//#include "gpio.h"
//#include "rtc.h"
//#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <time.h>
/* USER CODE END Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
uint16_t cell_voltages[TOTAL_CELLS]; // Cell voltages (in mV)
int16_t pack_current_1;              // Pack 1 current (in mA, positive for discharge)
int16_t pack_current_2;              // Pack 2 current (in mA, positive for discharge)
int16_t temperature_1;               // Temperature from Pack 1 (in °C)
int16_t temperature_2;               // Temperature from Pack 2 (in °C)
float soc = INITIAL_SOC;             // State of Charge (in %)
float soh = INITIAL_SOH;             // State of Health (in %)
KalmanFilter soc_kf;                 // Kalman Filter for SOC
KalmanFilter soh_kf;                 // Kalman Filter for SOH
/* Logging variables */
static uint32_t next_slot = 0;       // Tracks the next slot to write to
static uint8_t log_buffer[LOG_ENTRY_SIZE]; // Temporary buffer for log entries
/* Battery capacity tracking */
static float coulomb_count = (INITIAL_SOC / 100.0) * NOMINAL_CAPACITY; // Initial coulomb count in mAh
static float initial_capacity = NOMINAL_CAPACITY; // Initial capacity for SOH calculation
static float actual_capacity = NOMINAL_CAPACITY;  // Actual capacity for SOH calculation
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_Init(void);
/* USER CODE BEGIN PFP */
void Flash_Erase(uint32_t sector);
void Log_Error(const char *message);
void Log_Read_All(void);
void Log_Init(void);
void Update_SOC_SOH(void);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */
  // Turn off the LED at the start
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  // Initialize PWM for heaters (start with 0% duty cycle)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Start PWM on Channel 3 (HEATER2)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Start PWM on Channel 4 (HEATER1)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 0% duty cycle for HEATER2
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // 0% duty cycle for HEATER1

  // Set the initial RTC time to a known UTC value (e.g., 2025-03-28 12:00:00)
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  sTime.Hours = 12;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sDate.Year = 25; // 2025 - 2000
  sDate.Month = 3;
  sDate.Date = 28;
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  // Initialize the logging system
  Log_Init();
  Log_Error("System started");

  // Initialize BQ76920 on I2C1 (Pack 1)
  if (BQ76920_Init(&hi2c1) != HAL_OK)
  {
      Log_Error("BQ76920 (I2C1) initialization failed");
      Error_Handler();
  }

  // Initialize BQ76920 on I2C2 (Pack 2)
  if (BQ76920_Init(&hi2c2) != HAL_OK)
  {
      Log_Error("BQ76920 (I2C2) initialization failed");
      Error_Handler();
  }

  // Initialize Kalman Filters
  KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0); // Q=0.01, R=1.0
  KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0);

  // Initialize PID for heaters
  PID_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_log_read = 0; // Track the last time we read logs
  while (1)
  {
    // Step 1: Read data from BQ76920 on I2C1 (Pack 1, cells 1-3)
    if (BQ76920_ReadVoltages(&hi2c1, cell_voltages, 0) != HAL_OK)
    {
      Log_Error("Error reading BQ76920 (I2C1) voltages");
      for (uint8_t i = 0; i < NUM_CELLS_PER_IC; i++)
      {
          cell_voltages[i] = 0;
      }
    }
    if (BQ76920_ReadCurrent(&hi2c1, &pack_current_1) != HAL_OK)
    {
      Log_Error("Error reading BQ76920 (I2C1) current");
    }
    if (Temperature_Read(&hi2c1, &temperature_1) != HAL_OK)
    {
      Log_Error("Error reading temperature (I2C1)");
    }

    // Step 2: Read data from BQ76920 on I2C2 (Pack 2, cells 4-6)
    if (BQ76920_ReadVoltages(&hi2c2, cell_voltages, NUM_CELLS_PER_IC) != HAL_OK)
    {
      Log_Error("Error reading BQ76920 (I2C2) voltages");
      for (uint8_t i = NUM_CELLS_PER_IC; i < TOTAL_CELLS; i++)
      {
          cell_voltages[i] = 0;
      }
    }
    if (BQ76920_ReadCurrent(&hi2c2, &pack_current_2) != HAL_OK)
    {
      Log_Error("Error reading BQ76920 (I2C2) current");
    }
    if (Temperature_Read(&hi2c2, &temperature_2) != HAL_OK)
    {
      Log_Error("Error reading temperature (I2C2)");
    }

    // Step 3: Check for overvoltage/undervoltage protection
    uint8_t ov_flag_1, uv_flag_1, ov_flag_2, uv_flag_2;
    BQ76920_CheckProtection(&hi2c1, cell_voltages, 0, &ov_flag_1, &uv_flag_1);
    BQ76920_CheckProtection(&hi2c2, cell_voltages, NUM_CELLS_PER_IC, &ov_flag_2, &uv_flag_2);
    if (ov_flag_1 || ov_flag_2)
    {
      Log_Error("Overvoltage detected");
    }
    if (uv_flag_1 || uv_flag_2)
    {
      Log_Error("Undervoltage detected");
    }

    // Step 4: Balance cells
    if (BQ76920_BalanceCells(&hi2c1, cell_voltages, 0) != HAL_OK)
    {
      Log_Error("Error balancing cells (I2C1)");
    }
    if (BQ76920_BalanceCells(&hi2c2, cell_voltages, NUM_CELLS_PER_IC) != HAL_OK)
    {
      Log_Error("Error balancing cells (I2C2)");
    }

    // Step 5: Control the heaters using PID and PWM
    int16_t min_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2;
    PID_Control(min_temp);

    // Step 6: Update SOC and SOH
    Update_SOC_SOH();

    // Step 7: Log the voltages, current, temperature, SOC, and SOH to flash
    char message[MESSAGE_SIZE];
    snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick());
    for (uint8_t i = 0; i < TOTAL_CELLS; i++)
    {
      char cell_data[16];
      snprintf(cell_data, sizeof(cell_data), "Cell%d: %dmV ", i + 1, cell_voltages[i]);
      strcat(message, cell_data);
    }
    char temp_data[32];
    sprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC SOC: %.1f%% SOH: %.1f%%", pack_current_1, pack_current_2, temperature_1, temperature_2, soc, soh);
    strcat(message, temp_data);
    Log_Error(message);

    // Step 8: Periodically read and send all logs over RS485 (every 10 seconds)
    if (HAL_GetTick() - last_log_read >= 10000)
    {
      Log_Read_All();
      last_log_read = HAL_GetTick();
    }

    // Wait for 1 second before the next iteration
    HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;  // HSE = 8 MHz, PLLM = 1 -> 8 MHz input to PLL
  RCC_OscInitStruct.PLL.PLLN = 20; // 8 MHz * 20 = 160 MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; // 160 MHz / 7 = ~22.86 MHz (not used for SYSCLK)
  RCC_OscInitStruct.PLL.PLLQ = 2;  // 160 MHz / 2 = 80 MHz (not used for SYSCLK)
  RCC_OscInitStruct.PLL.PLLR = 2;  // 160 MHz / 2 = 80 MHz (used for SYSCLK)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // HCLK = 80 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // APB1 = 40 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // APB2 = 80 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  /* USER CODE BEGIN I2C3_Init 0 */
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */
  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00201D2B;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  /* USER CODE END I2C3_Init 2 */
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS4852_DE_Pin */
  GPIO_InitStruct.Pin = RS4852_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS4852_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT2_Pin */
  GPIO_InitStruct.Pin = BOOT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALERT2_Pin */
  GPIO_InitStruct.Pin = ALERT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT_Pin ALERT_Pin */
  GPIO_InitStruct.Pin = BOOT_Pin|ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Helper function to unlock flash for writing
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef Flash_Unlock(void) {
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR);
    return HAL_OK;
}

/**
  * @brief  Helper function to lock flash after writing
  * @retval None
  */
static void Flash_Lock(void) {
    HAL_FLASH_Lock();
}

/**
  * @brief  Helper function to write data to flash
  * @param  address: Flash address to write to
  * @param  data: Data to write
  * @param  size: Size of data in bytes
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef Flash_Write(uint32_t address, const uint8_t *data, uint32_t size) {
    if (Flash_Unlock() != HAL_OK) {
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < size; i += 8) { // Increment by 8 bytes (64-bit double-word)
        uint64_t double_word = 0;
        memcpy(&double_word, &data[i], 8); // Copy 8 bytes into a 64-bit variable
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, double_word) != HAL_OK) {
            Flash_Lock();
            return HAL_ERROR;
        }
    }

    Flash_Lock();
    return HAL_OK;
}

/**
  * @brief  Helper function to read data from flash
  * @param  address: Flash address to read from
  * @param  data: Buffer to store the read data
  * @param  size: Size of data to read in bytes
  * @retval None
  */
static void Flash_Read(uint32_t address, uint8_t *data, uint32_t size) {
    memcpy(data, (uint8_t *)address, size);
}

/**
  * @brief  Erases a specified page in Flash memory.
  * @param  page: The page to erase
  * @retval None
  */
void Flash_Erase(uint32_t page)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;  // Use Bank 1 (address 0x080E0000 is in Bank 1)
    erase_init.Page = page;           // Page number (e.g., 448 for 0x080E0000)
    erase_init.NbPages = 1;           // Erase 1 page

    if (Flash_Unlock() != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        Error_Handler();
    }

    Flash_Lock();
}

/**
  * @brief  Gets the current UTC timestamp from the RTC
  * @retval uint64_t: Unix timestamp (seconds since epoch)
  */
static uint64_t Get_UTCTimestamp(void) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    struct tm time_struct = {0};
    time_struct.tm_year = sDate.Year + 2000 - 1900;
    time_struct.tm_mon = sDate.Month - 1;
    time_struct.tm_mday = sDate.Date;
    time_struct.tm_hour = sTime.Hours;
    time_struct.tm_min = sTime.Minutes;
    time_struct.tm_sec = sTime.Seconds;

    return (uint64_t)mktime(&time_struct);
}

/**
  * @brief  Initializes the logging system
  * @retval None
  */
void Log_Init(void) {
    Flash_Read(NEXT_SLOT_ADDR, (uint8_t *)&next_slot, sizeof(next_slot));
    if (next_slot == 0xFFFFFFFF) {
        Flash_Erase(FLASH_LOG_PAGE);
        next_slot = 0;
        Flash_Write(NEXT_SLOT_ADDR, (uint8_t *)&next_slot, sizeof(next_slot));
    }
}

/**
  * @brief  Logs a message to flash with a UTC timestamp
  * @param  message: The message to log
  * @retval None
  */
void Log_Error(const char *message)
{
    memset(log_buffer, 0, LOG_ENTRY_SIZE);
    uint64_t timestamp = Get_UTCTimestamp();
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE);
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message, MESSAGE_SIZE - 1);

    uint32_t slot_addr = LOG_START_ADDR + (next_slot * LOG_ENTRY_SIZE);
    if (Flash_Write(slot_addr, log_buffer, LOG_ENTRY_SIZE) != HAL_OK) {
        HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);
        HAL_USART_Transmit(&husart2, (uint8_t *)"Flash write failed\n", 19, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
        return;
    }

    next_slot = (next_slot + 1) % NUM_LOG_ENTRIES;
    if (next_slot == 0) {
        Flash_Erase(FLASH_LOG_PAGE);
    }
    Flash_Write(NEXT_SLOT_ADDR, (uint8_t *)&next_slot, sizeof(next_slot));
}

/**
  * @brief  Reads all logs from flash and sends them over RS485
  * @retval None
  */
void Log_Read_All(void)
{
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);

    for (uint32_t i = 0; i < NUM_LOG_ENTRIES; i++) {
        uint32_t slot_addr = LOG_START_ADDR + (i * LOG_ENTRY_SIZE);
        Flash_Read(slot_addr, log_buffer, LOG_ENTRY_SIZE);

        uint64_t timestamp;
        memcpy(&timestamp, log_buffer, TIMESTAMP_SIZE);
        if (timestamp == 0xFFFFFFFFFFFFFFFFULL) continue;

        char log_message[128];
        snprintf(log_message, sizeof(log_message), "[%llu] %s\n", timestamp, (char *)(log_buffer + TIMESTAMP_SIZE));
        HAL_USART_Transmit(&husart2, (uint8_t *)log_message, strlen(log_message), HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Updates SOC and SOH using coulomb counting and Kalman filtering
  * @retval None
  */
void Update_SOC_SOH(void)
{
    int16_t avg_current = (pack_current_1 + pack_current_2) / 2;
    coulomb_count -= (float)avg_current * LOOP_TIME / 3600.0; // Convert mAs to mAh

    if (coulomb_count < 0) coulomb_count = 0;
    if (coulomb_count > NOMINAL_CAPACITY) coulomb_count = NOMINAL_CAPACITY;

    float raw_soc = (coulomb_count / NOMINAL_CAPACITY) * 100.0;
    KalmanFilter_Update(&soc_kf, raw_soc);
    soc = soc_kf.state;

    static uint32_t cycle_count = 0;
    cycle_count++;
    if (cycle_count % 100 == 0) {
        actual_capacity *= 0.995; // 0.5% degradation per cycle
        soh = (actual_capacity / initial_capacity) * 100.0;
        KalmanFilter_Update(&soh_kf, soh);
        soh = soh_kf.state;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  char message[MESSAGE_SIZE];
  snprintf(message, sizeof(message), "Assert failed: %s, line %lu", file, line);
  Log_Error(message);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
