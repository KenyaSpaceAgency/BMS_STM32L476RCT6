//
// Include header files needed for the project
#include "main.h"             // Main project header with TelemetryData, GPIO pins, and constants
#include "flash.h"            // Flash memory functions for storing telemetry
#include "BMS_Service.h"      // BMS service functions for battery management

// Declare global hardware handles (initialized by HAL functions)
ADC_HandleTypeDef hadc1;      // ADC for PCB temperature (internal STM32 sensor)
I2C_HandleTypeDef hi2c1;      // I2C1 (PB6/PB7) for first BQ76920 and TMP100 sensors
I2C_HandleTypeDef hi2c2;      // I2C2 (PB10/PB11) for second BQ76920
I2C_HandleTypeDef hi2c3;      // I2C3 (PC0/PC1) for EPS sync counter
DMA_HandleTypeDef hdma_i2c3_rx; // DMA for I2C3 receive
DMA_HandleTypeDef hdma_i2c3_tx; // DMA for I2C3 transmit
TIM_HandleTypeDef htim2;      // Timer 2 for system timing
UART_HandleTypeDef huart1;    // UART1 (PA9/PA10) for error logging
UART_HandleTypeDef huart2;    // UART2 (PA2/PA3) for additional communication
DMA_HandleTypeDef hdma_usart2_rx; // DMA for UART2 receive
DMA_HandleTypeDef hdma_usart2_tx; // DMA for UART2 transmit

// Declare initialization functions for hardware peripherals
void SystemClock_Config(void);     // Configures the system clock
static void MX_GPIO_Init(void);    // Initializes GPIO pins (LED, heaters, boot, alerts)
static void MX_DMA_Init(void);     // Initializes DMA for I2C and UART
static void MX_I2C3_Init(void);    // Initializes I2C3 for EPS sync
static void MX_USART2_UART_Init(void); // Initializes UART2
static void MX_I2C1_Init(void);    // Initializes I2C1 for BQ76920 and TMP100
static void MX_I2C2_Init(void);    // Initializes I2C2 for second BQ76920
static void MX_USART1_UART_Init(void); // Initializes UART1 for logging
static void MX_ADC1_Init(void);    // Initializes ADC for temperature
static void MX_TIM2_Init(void);    // Initializes Timer 2 for timing

// Declare error logging function (defined later)
void Log_Error(const char *format, ...);

// Initialize global BMS instances for two BQ76920 chips
BQ76920_t bms_instance1 = {0};    // First BQ76920 instance (I2C1)
BQ76920_t bms_instance2 = {0};    // Second BQ76920 instance (I2C2)
TelemetryData telemetry = {0};     // Global telemetry structure for all battery data

// Function: main
// Inputs:
//   - None (void)
// Output:
//   - Returns int (required by C standard, but never returns in embedded systems)
// Significance:
//   - The entry point of the program, initializes hardware, sets up BMS, and runs the main loop
//     to monitor and manage the battery pack.
int main(void) {
    // Initialize the STM32 HAL library (sets up interrupts, clocks, etc.)
    HAL_Init();
    // Configure the system clock (8 MHz HSE)
    SystemClock_Config();
    // Initialize GPIO pins (LED, heaters, boot, alerts)
    MX_GPIO_Init();
    // Initialize DMA for I2C3 and UART2
    MX_DMA_Init();
    // Initialize I2C3 for EPS sync counter
    MX_I2C3_Init();
    // Initialize UART2 for additional communication
    MX_USART2_UART_Init();
    // Initialize I2C1 for first BQ76920 and TMP100
    MX_I2C1_Init();
    // Initialize I2C2 for second BQ76920
    MX_I2C2_Init();
    // Initialize UART1 for error logging
    MX_USART1_UART_Init();
    // Initialize ADC for PCB temperature
    MX_ADC1_Init();
    // Initialize Timer 2 for system timing
    MX_TIM2_Init();

    // Start Timer 2 to begin counting (used for system ticks)
    HAL_TIM_Base_Start(&htim2);

    // Set up first BQ76920 instance with I2C1 and boot pin
    bms_instance1.i2cHandle = &hi2c1;         // Link to I2C1 handle
    bms_instance1.bootPort = BOOT1_GPIO_Port; // Boot pin port (GPIOB)
    bms_instance1.bootPin = BOOT1_Pin;        // Boot pin (PB4)
    // Set up second BQ76920 instance with I2C2 and boot pin
    bms_instance2.i2cHandle = &hi2c2;         // Link to I2C2 handle
    bms_instance2.bootPort = BOOT2_GPIO_Port; // Boot pin port (GPIOC)
    bms_instance2.bootPin = BOOT2_Pin;        // Boot pin (PC7)

    // Initialize the BMS (sets up BQ76920 chips and loads telemetry)
    BMS_Service_Init();
    // Initialize the temperature system (TMP100 sensors and PID controller)
    Temperature_Init();  // Initialize TMP100 sensors and PID system

    // Store the current system tick (milliseconds) for timing
    uint32_t last_time = HAL_GetTick();
    // Flag to track low-power mode (1=low power, 0=normal)
    uint8_t low_power_mode = 0;

    // Main loop: runs forever, managing the BMS
    while (1) {
        // Get the current system tick (milliseconds)
        uint32_t current_time = HAL_GetTick();
        // Calculate elapsed time in seconds
        uint32_t delta_time = (current_time - last_time) / 1000;
    	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        // If BMS is online (no I2C errors)
        if (telemetry.bms_online) {
            // Check and handle alerts from BQ76920 chips
            BMS_Service_HandleAlerts();
            // Read measurements (voltages, currents, temperatures)
            BMS_Service_ReadMeasurements();
            // Process data (SOC, SOH, balancing)
            BMS_Service_ProcessData();
            // Update time-based counters
            BMS_Service_UpdateCounters(delta_time);
            // Check for low-power conditions (SOC < 5%)
            BMS_Service_HandleLowPowerCondition(&low_power_mode);
            // Save telemetry to flash if needed
            BMS_Service_HandleFlashStorage();
        }

        // If in low-power mode, try to wake BQ76920 chips
        if (low_power_mode) {
            BMS_Service_HandleLowPowerMode(&low_power_mode);
        }

        // Update last_time for the next loop
        last_time = current_time;
        // Wait for the voltage read interval (defined in BMS_Service.h)
        HAL_Delay(1000);
    }
}

// Function: SystemClock_Config
// Inputs:
//   - None (void)
// Output:
//   - None (void), configures the system clock
// Significance:
//   - Sets up the STM32’s clock to 8 MHz using the HSE oscillator, critical for
//     accurate timing of all peripherals (Reference Manual, Section 6, page 188).
void SystemClock_Config(void)
{
    // Declare structure for oscillator configuration
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    // Declare structure for clock configuration
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    // Set voltage scaling to maximum performance (Scale 1)
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Use HSE oscillator (8 MHz external crystal)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    // Enable HSE oscillator
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    // Disable PLL (not used in this configuration)
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    // Apply oscillator configuration
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Configure clocks: system, AHB, APB1, APB2
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    // Use HSE as system clock source
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    // Set AHB clock divider to 1 (8 MHz)
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    // Set APB1 clock divider to 1 (8 MHz)
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    // Set APB2 clock divider to 1 (8 MHz)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    // Apply clock configuration with zero latency (for 8 MHz)
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_ADC1_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes ADC1 for PCB temperature
// Significance:
//   - Sets up ADC1 to read the STM32’s internal temperature sensor, used in
//     BMS_Service.c for telemetry.pcb_temperature (Reference Manual, Section 15, page 439).
static void MX_ADC1_Init(void)
{
    // Declare structure for multi-mode ADC configuration
    ADC_MultiModeTypeDef multimode = {0};
    // Declare structure for ADC channel configuration
    ADC_ChannelConfTypeDef sConfig = {0};
    // Set ADC1 instance (ADC1 peripheral)
    hadc1.Instance = ADC1;
    // Use asynchronous clock with no prescaler
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    // Use 12-bit resolution for accurate readings
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    // Align data to the right
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    // Disable scan mode (single channel)
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    // Use single conversion end-of-conversion flag
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    // Disable auto-wait mode
    hadc1.Init.LowPowerAutoWait = DISABLE;
    // Use single conversion mode (not continuous)
    hadc1.Init.ContinuousConvMode = DISABLE;
    // Use one conversion per trigger
    hadc1.Init.NbrOfConversion = 1;
    // Disable discontinuous mode
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    // Use software trigger (manual start)
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    // No external trigger edge
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    // Disable DMA for ADC
    hadc1.Init.DMAContinuousRequests = DISABLE;
    // Preserve data on overrun
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    // Disable oversampling
    hadc1.Init.OversamplingMode = DISABLE;
    // Initialize ADC1 with these settings
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Set ADC to independent mode (no multi-ADC)
    multimode.Mode = ADC_MODE_INDEPENDENT;
    // Configure multi-mode
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Configure ADC channel for internal temperature sensor
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    // Set as first (and only) channel
    sConfig.Rank = ADC_REGULAR_RANK_1;
    // Use 2.5 cycles sampling time
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    // Use single-ended input
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    // No offset correction
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    // Set offset to 0
    sConfig.Offset = 0;
    // Configure ADC channel
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_I2C1_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes I2C1 for BQ76920 and TMP100
// Significance:
//   - Sets up I2C1 (PB6/PB7) for communication with the first BQ76920 and both TMP100 sensors
//     (BQ76920 datasheet, Section 8.5.23, page 33).
static void MX_I2C1_Init(void)
{
    // Set I2C1 instance (I2C1 peripheral)
    hi2c1.Instance = I2C1;
    // Set timing for 100 kHz I2C (calculated for 8 MHz clock)
    hi2c1.Init.Timing = 0x00201D2B;
    // No own address (STM32 is master)
    hi2c1.Init.OwnAddress1 = 0;
    // Use 7-bit addressing mode
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // Disable dual addressing
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // No second address
    hi2c1.Init.OwnAddress2 = 0;
    // No address mask
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    // Disable general call
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // Allow clock stretching
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // Initialize I2C1
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Enable analog filter for noise reduction
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Disable digital filter
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_I2C2_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes I2C2 for second BQ76920
// Significance:
//   - Sets up I2C2 (PB10/PB11) for communication with the second BQ76920 chip.
static void MX_I2C2_Init(void)
{
    // Set I2C2 instance
    hi2c2.Instance = I2C2;
    // Set timing for 100 kHz I2C
    hi2c2.Init.Timing = 0x00201D2B;
    // No own address
    hi2c2.Init.OwnAddress1 = 0;
    // Use 7-bit addressing
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // Disable dual addressing
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // No second address
    hi2c2.Init.OwnAddress2 = 0;
    // No address mask
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    // Disable general call
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // Allow clock stretching
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // Initialize I2C2
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Enable analog filter
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Disable digital filter
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_I2C3_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes I2C3 for EPS sync counter
// Significance:
//   - Sets up I2C3 (PC0/PC1) for communication with the EPS Power Distribution system,
//     intended for DMA-based sync counter reception.
static void MX_I2C3_Init(void)
{
    // Set I2C3 instance
    hi2c3.Instance = I2C3;
    // Set timing for 100 kHz I2C
    hi2c3.Init.Timing = 0x00201D2B;
    // No own address
    hi2c3.Init.OwnAddress1 = 0;
    // Use 7-bit addressing
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // Disable dual addressing
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // No second address
    hi2c3.Init.OwnAddress2 = 0;
    // No address mask
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    // Disable general call
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // Allow clock stretching
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // Initialize I2C3
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Enable analog filter
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Disable digital filter
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_TIM2_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes Timer 2 for system timing
// Significance:
//   - Sets up Timer 2 to generate a 1ms tick, used for system timing and delays
//     (Reference Manual, Section 27, page 846).
static void MX_TIM2_Init(void)
{
    // Declare structure for clock source configuration
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    // Declare structure for master configuration
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    // Set Timer 2 instance
    htim2.Instance = TIM2;
    // Prescaler for 1ms ticks (8 MHz / 8000 = 1 kHz)
    htim2.Init.Prescaler = 7999;
    // Count up mode
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    // Maximum period (65535)
    htim2.Init.Period = 0xFFFF;
    // No clock division
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // Disable auto-reload preload
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // Initialize Timer 2
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // Use internal clock source
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    // Configure clock source
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
    // No master output trigger
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    // Disable master-slave mode
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    // Configure master synchronization
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_USART1_UART_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes UART1 for error logging
// Significance:
//   - Sets up UART1 (PA9/PA10) for logging errors to a serial terminal
//     (Reference Manual, Section 36, page 1235).
static void MX_USART1_UART_Init(void)
{
    // Set USART1 instance
    huart1.Instance = USART1;
    // Set baud rate to 115200
    huart1.Init.BaudRate = 115200;
    // Use 8-bit word length
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    // Use 1 stop bit
    huart1.Init.StopBits = UART_STOPBITS_1;
    // No parity
    huart1.Init.Parity = UART_PARITY_NONE;
    // Enable transmit and receive
    huart1.Init.Mode = UART_MODE_TX_RX;
    // No hardware flow control
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // Use 16x oversampling
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    // Disable one-bit sampling
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // No advanced features
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    // Initialize UART1
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_USART2_UART_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes UART2 for communication
// Significance:
//   - Sets up UART2 (PA2/PA3) with RS485 mode, potentially for EPS communication.
static void MX_USART2_UART_Init(void)
{
    // Set USART2 instance
    huart2.Instance = USART2;
    // Set baud rate to 115200
    huart2.Init.BaudRate = 115200;
    // Use 8-bit word length
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    // Use 1 stop bit
    huart2.Init.StopBits = UART_STOPBITS_1;
    // No parity
    huart2.Init.Parity = UART_PARITY_NONE;
    // Enable transmit and receive
    huart2.Init.Mode = UART_MODE_TX_RX;
    // No hardware flow control
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // Use 16x oversampling
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    // Disable one-bit sampling
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // No advanced features
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    // Initialize UART2 with RS485 mode
    if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        // Call error handler on failure
        Error_Handler();
    }
}

// Function: MX_DMA_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes DMA for I2C3 and UART2
// Significance:
//   - Sets up DMA channels for efficient data transfer (e.g., I2C3 sync counter).
static void MX_DMA_Init(void)
{
    // Enable DMA1 clock
    __HAL_RCC_DMA1_CLK_ENABLE();
    // Set priority for DMA1 Channel 2 (I2C3 RX)
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    // Enable DMA1 Channel 2 interrupt
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    // Set priority for DMA1 Channel 3 (I2C3 TX)
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    // Enable DMA1 Channel 3 interrupt
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    // Set priority for DMA1 Channel 6 (USART2 RX)
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    // Enable DMA1 Channel 6 interrupt
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    // Set priority for DMA1 Channel 7 (USART2 TX)
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    // Enable DMA1 Channel 7 interrupt
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

// Function: MX_GPIO_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes GPIO pins
// Significance:
//   - Sets up GPIO pins for LED, heaters, boot, and alert signals (Reference Manual, Section 8, page 246).
static void MX_GPIO_Init(void)
{
    // Declare structure for GPIO configuration
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Enable clock for GPIOC (LED, BOOT2)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // Enable clock for GPIOH (not used here)
    __HAL_RCC_GPIOH_CLK_ENABLE();
    // Enable clock for GPIOA (ALERT2)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // Enable clock for GPIOB (BOOT1, HEATER1, HEATER2, ALERT1)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // Initialize LED and BOOT2 pins to low (off)
    HAL_GPIO_WritePin(GPIOC, LED_Pin|BOOT2_Pin, GPIO_PIN_RESET);
    // Initialize BOOT1, HEATER1, HEATER2 pins to low
    HAL_GPIO_WritePin(GPIOB, BOOT1_Pin|HEATER2_Pin|HEATER1_Pin, GPIO_PIN_RESET);
    // Configure LED and BOOT2 as output pins
    GPIO_InitStruct.Pin = LED_Pin|BOOT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;        // No pull-up/pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    // Configure ALERT2 as interrupt input (rising edge)
    GPIO_InitStruct.Pin = ALERT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ALERT2_GPIO_Port, &GPIO_InitStruct);
    // Configure BOOT1, HEATER1, HEATER2 as output pins
    GPIO_InitStruct.Pin = BOOT1_Pin|HEATER2_Pin|HEATER1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // Configure ALERT1 as interrupt input (rising edge)
    GPIO_InitStruct.Pin = ALERT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ALERT1_GPIO_Port, &GPIO_InitStruct);
    // Set interrupt priority for EXTI9_5 (ALERT1, PB5)
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    // Enable EXTI9_5 interrupt
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    // Set interrupt priority for EXTI15_10 (ALERT2, PA12)
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    // Enable EXTI15_10 interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Function: HAL_GPIO_EXTI_Callback
// Inputs:
//   - GPIO_Pin: A uint16_t, the GPIO pin that triggered the interrupt
// Output:
//   - None (void), updates telemetry with alert status
// Significance:
//   - Handles interrupts from ALERT1 and ALERT2 pins, updating BMS alerts when triggered
//     (BQ76920 datasheet, Section 8.5.1, page 24).
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Check if interrupt was from ALERT1 (PB5) or ALERT2 (PA12)
    if (GPIO_Pin == ALERT1_Pin || GPIO_Pin == ALERT2_Pin) {
        // Read alerts from first BQ76920
        readAlert(&bms_instance1);
        // Read alerts from second BQ76920
        readAlert(&bms_instance2);
        // Check I2C1 communication status
        telemetry.i2c_comm_error_ic1 = (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
        // Check I2C2 communication status
        telemetry.i2c_comm_error_ic2 = (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
        // Set BMS online if no I2C errors
        telemetry.bms_online = !(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2);
        // Update overcurrent/short-circuit alert for first BQ76920
        telemetry.ovrd_alert_ic1 = getAlert(&bms_instance1, 5);
        // Update overcurrent/short-circuit alert for second BQ76920
        telemetry.ovrd_alert_ic2 = getAlert(&bms_instance2, 5);
        // Update device-ready alert for first BQ76920
        telemetry.device_xready_ic1 = getAlert(&bms_instance1, 6);
        // Update device-ready alert for second BQ76920
        telemetry.device_xready_ic2 = getAlert(&bms_instance2, 6);
        // Update load detection alert for first BQ76920
        telemetry.load_present_ic1 = getAlert(&bms_instance1, 7);
        // Update load detection alert for second BQ76920
        telemetry.load_present_ic2 = getAlert(&bms_instance2, 7);
        // Log an alert event
        Log_Error("Alert triggered on BMS");
    }
}

// Function: Log_Error
// Inputs:
//   - format: A string (const char*), the error message format
//   - ...: Variable arguments for formatting (like printf)
// Output:
//   - None (void), sends error message over UART1
// Significance:
//   - Logs error messages to a serial terminal for debugging, used throughout the project.
void Log_Error(const char *format, ...) {
    // Declare buffer to hold formatted message
    char buffer[128];
    // Declare variable argument list
    va_list args;
    // Initialize argument list
    va_start(args, format);
    // Format message into buffer
    vsnprintf(buffer, sizeof(buffer), format, args);
    // Clean up argument list
    va_end(args);
    // Send message over UART1
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Function: Error_Handler
// Inputs:
//   - None (void)
// Output:
//   - None (void), enters an infinite loop
// Significance:
//   - Called on critical hardware errors (e.g., ADC, I2C failure) to stop the system safely.
void Error_Handler(void)
{
    // Disable all interrupts
    __disable_irq();
    // Enter infinite loop to halt execution
    while (1)
    {
    }
}

// Function: assert_failed
// Inputs:
//   - file: A pointer to a string (uint8_t*), the file where the assertion failed
//   - line: A uint32_t, the line number of the failure
// Output:
//   - None (void), does nothing in this implementation
// Significance:
//   - Placeholder for debugging assertions, currently empty but can be expanded for error logging.
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif //I JUST WANT The led pin to toggle on and off for 1 sec
