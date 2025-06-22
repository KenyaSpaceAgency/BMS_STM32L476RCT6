/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Battery Management System (BMS)
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
  * @brief  Implements the Battery Management System (BMS) for a CubeSat, managing battery monitoring,
  *         charging, protection, and communication with the On-Board Computer (OBC).
  * @note   - Runs on an STM32 microcontroller (e.g., STM32L4 series).
  *         - Monitors a 4S2P lithium-ion battery pack using two BQ76920 ICs for redundancy.
  *         - Communicates with the OBC via SSP (Simple Serial Protocol) over RS485.
  *         - Includes features like coulomb counting, Kalman filtering for SOC/SOH, CC-CV charging,
  *           cell balancing, temperature monitoring, and firmware updates.
  * @context In a CubeSat, the BMS ensures reliable power management by monitoring battery health,
  *          protecting against faults (e.g., overvoltage, overtemperature), controlling charging/discharging,
  *          and communicating status to the OBC. This file is the core of the EPS (Electrical Power System),
  *          integrating multiple subsystems for safe operation in space.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
/* @brief  Private variables for the BMS, storing state and configuration data.
 * @note   These variables are used to track battery status, BMS modes, and system state.
 * @context Declared as global to maintain state across functions in the BMS loop, reflecting
 *          the real-time monitoring and control requirements of a CubeSat EPS.
 */
// Array to store voltage readings for the 4 cells monitored by the first BQ76920 IC (Cells 1-4)
uint16_t group_voltages_1[NUM_GROUPS_PER_IC];
// Array to store voltage readings for the 4 cells monitored by the second BQ76920 IC (Cells 1-4, for redundancy)
uint16_t group_voltages_2[NUM_GROUPS_PER_IC];
// Variable to store the pack current measured by the first BQ76920 IC (in mA)
int16_t pack_current_1;
// Variable to store the pack current measured by the second BQ76920 IC (in mA)
int16_t pack_current_2;
// Variable to store the temperature reading from the first NTC sensor (in °C)
int16_t temperature_1;
// Variable to store the temperature reading from the second NTC sensor (in °C)
int16_t temperature_2;
// Variable to store the PCB temperature measured by the STM32’s internal sensor (in °C)
int16_t pcb_temperature;
// Variable to store the State of Charge (SOC) of the battery, initialized to a predefined value (in %)
float soc = INITIAL_SOC;
// Variable to store the State of Health (SOH) of the battery, initialized to a predefined value (in %)
float soh = INITIAL_SOH;
// Kalman filter structure for SOC estimation, used to smooth noisy SOC measurements
KalmanFilter soc_kf;
// Kalman filter structure for SOH estimation, used to smooth noisy SOH measurements
KalmanFilter soh_kf;

// Static variable to track the next available slot for logging errors in flash memory
static uint32_t next_slot = 0;
// Static buffer to store a single log entry before writing it to flash memory
static uint8_t log_buffer[LOG_ENTRY_SIZE];

// Static variable to track the cumulative charge (in mAh) for coulomb counting
static float coulomb_count;
// Static variable to store the initial capacity of the battery pack (in mAh)
static float initial_capacity;
// Static variable to store the actual capacity of the battery pack, updated based on SOC (in mAh)
static float actual_capacity;

// Static buffer for transmitting SSP frames to the OBC
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
// Static buffer for receiving SSP frames from the OBC
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];

// Static variable to store the current BMS operating mode (e.g., charging, discharging, sleep, fault)
static BMS_ModeTypeDef bms_mode = MODE_DISCHARGING;
// Static flag to enable or disable charging (0 = disabled, 1 = enabled)
static uint8_t charge_enabled = 0;
// Static flag to enable or disable discharging (0 = disabled, 1 = enabled), initialized to enabled
static uint8_t discharge_enabled = 1;
// Static flag to indicate if the battery should be charged immediately (e.g., when SOC is low)
static uint8_t charge_immediately = 0;
// Static flag to indicate if the BMS is online and operational
static uint8_t bms_online = 0;
// Static variable to store error flags for various fault conditions (e.g., overvoltage, overtemperature)
static uint32_t error_flags = 0;

// Static variable to count the number of completed charge cycles
static uint32_t charge_cycle_count = 0;
// Static variable to track the total time spent charging (in seconds)
static uint32_t total_charge_time = 0;
// Static variable to track the total time spent discharging (in seconds)
static uint32_t total_discharge_time = 0;
// Static variable to track the total operating time of the BMS (in seconds)
static uint32_t total_operating_time = 0;
// Static flag to indicate if a charging cycle has started (used to track cycle completion)
static uint8_t charging_started = 0;

// Static variable to store the timestamp when charging starts (in milliseconds), used for CC-CV charging
static uint32_t charge_start_time = 0;
// Static flag to indicate if the BMS is in Constant Voltage (CV) mode during charging (0 = CC, 1 = CV)
static uint8_t in_cv_mode = 0;

// Static flag to enable or disable mission termination, set by Ground Control Station (GCS) via SFP command
static uint8_t mission_termination_enabled = 0;

// Static variable to store the balancing mask for the first BQ76920 IC (bit 0 = Cell 1, bit 1 = Cell 2, etc.)
static uint8_t balancing_mask_1 = 0;
// Static variable to store the balancing mask for the second BQ76920 IC (bit 0 = Cell 1, bit 1 = Cell 2, etc.)
static uint8_t balancing_mask_2 = 0;
// Static flag to indicate if cell balancing is active (1 = active, 0 = inactive)
static uint8_t balancing_active = 0;

// Static array to store the state of 16 power lines (PWRL0 to PWRL15), controlled by SON/SOF commands (0 = OFF, 1 = ON)
static uint8_t power_lines[16] = {0};

// Static variable to store the timestamp when the BMS starts blinking the debug LED (in milliseconds)
static uint32_t startup_blink_start = 0;
// Static variable to store the timestamp of the last LED toggle (in milliseconds)
static uint32_t last_blink_toggle = 0;
// Define the duration for the startup LED blinking sequence (5 seconds)
#define STARTUP_BLINK_DURATION 5000
// Define the interval for toggling the LED during blinking (200 ms on/off)
#define BLINK_INTERVAL 200

// Structure to store battery configuration parameters
BatteryConfig battery_config = {
    .nominal_capacity = 4000.0f,      // Nominal capacity of the 4S2P pack (2 cells in parallel, 2000 mAh each, in mAh)
    .ov_threshold = 4200,             // Overvoltage threshold per cell (4.2V, in mV)
    .uv_threshold = 2800,             // Undervoltage threshold per cell (2.8V, in mV)
    .occ_threshold = 5000,            // Overcurrent threshold for charging (5000 mA)
    .ocd_threshold = 5000,            // Overcurrent threshold for discharging (5000 mA)
    .overtemp_threshold = 60,         // Overtemperature threshold (60°C)
    .undertemp_threshold = -20,       // Undertemperature threshold (-20°C)
    .soc_low_threshold = 20.0f,       // Low SOC threshold for triggering immediate charging (20%)
    .max_charge_time = 3600,          // Maximum allowed charging time (3600 seconds)
    .cv_threshold = 4200              // Voltage threshold for switching to CV mode (4200 mV)
};

// Static flag to indicate if the system is in firmware update mode (0 = normal, 1 = update)
static uint8_t firmware_update_mode = 0;
// Define the size of each firmware update packet (128 bytes)
#define FIRMWARE_UPDATE_PACKET_SIZE 128
// Define the timeout for firmware update operations (10 seconds)
#define FIRMWARE_UPDATE_TIMEOUT 10000
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* @brief  Function prototypes for private BMS functions.
 * @note   These functions handle core BMS operations like logging, state updates, and communication.
 * @context Declared here to be used throughout the BMS implementation, enabling modular design
 *          and separation of concerns in the CubeSat EPS.
 */
void Flash_Erase(uint32_t page);
void Log_Error(const char *format, ...);
void Log_Read_All(void);
void Log_Init(void);
void Update_SOC_SOH(void);
void SSP_SendStatus(void);
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame);
void Update_BMS_Mode(void);
int16_t Read_Internal_Temperature(void);
HAL_StatusTypeDef ChargeBattery(void);
void Bootloader_Check(void);
void Bootloader_FirmwareUpdate(void);
void JumpToApplication(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Erases a specified page in the STM32’s flash memory.
  * @param  page: The page number to erase.
  * @retval None (modifies flash memory in-place).
  * @note   - Uses HAL flash functions to unlock, erase, and relock the flash memory.
  *         - Ensures atomicity by locking the flash after erasure.
  *         - Page size depends on the STM32 model (typically 2 KB for STM32L4).
  * @context Called by `Log_Init` to clear the log region and by `Bootloader_FirmwareUpdate`
  *          to erase the application region before writing new firmware. Flash erasure is
  *          critical for maintaining persistent logs and enabling firmware updates in the CubeSat.
  * @integration The STM32’s flash memory is used for both logging (diagnostic data) and firmware
  *              storage, ensuring non-volatile storage of critical information in a space environment.
  * @debug  - If erasure fails, check the page number and ensure the flash is not write-protected.
  *         - Verify that `page_error` is checked for detailed error information if needed.
  */
void Flash_Erase(uint32_t page)
{
    FLASH_EraseInitTypeDef erase_init; // Structure to configure flash erase operation
    uint32_t page_error;               // Variable to store error information during erasure

    // Configure erase operation for a single page
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page;
    erase_init.NbPages = 1;

    // Unlock flash, erase page, and relock for atomicity
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
}

/**
  * @brief  Logs an error message to flash memory with a timestamp.
  * @param  format: Format string for the error message (printf-style).
  * @param  ...: Variable arguments to format the message.
  * @retval None (writes log entry to flash memory).
  * @note   - Uses `HAL_GetTick` for timestamping (in milliseconds) instead of RTC for simplicity.
  *         - Stores logs in a circular buffer in flash (`LOG_START_ADDR` to `LOG_START_ADDR + NUM_LOG_ENTRIES * LOG_ENTRY_SIZE`).
  *         - Each log entry includes a timestamp (`TIMESTAMP_SIZE` bytes) and message (`MESSAGE_SIZE` bytes).
  *         - Flash writes are done in 8-byte chunks (double words) to align with STM32 flash requirements.
  * @context Called throughout the BMS (e.g., `Update_BMS_Mode`, `SSP_ProcessReceivedFrame`) to
  *          log errors and system events for diagnostics. Logs are read and sent to the OBC every
  *          10 seconds (`Log_Read_All`) for analysis, critical for debugging in space.
  * @integration The STM32 uses its flash memory as a non-volatile log store, ensuring diagnostic
  *              data persists across power cycles. The circular buffer prevents flash wear by
  *              distributing writes, and `next_slot` is stored in flash for persistence.
  * @debug  - If logs are not written, check flash write permissions and ensure `LOG_START_ADDR`
  *           is correctly aligned.
  *         - If messages are truncated, verify `MESSAGE_SIZE` is sufficient.
  *         - Use `Log_Read_All` to retrieve logs and analyze system behavior.
  */
void Log_Error(const char *format, ...)
{
    // Retrieve current time and date from RTC (though not used for timestamp in this implementation)
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Use system tick count as timestamp (in milliseconds) for simplicity
    uint64_t timestamp = HAL_GetTick();
    char message_buffer[MESSAGE_SIZE]; // Buffer to format the error message

    // Format the message using variable arguments, capped to prevent overflow
    va_list args;
    va_start(args, format);
    vsnprintf(message_buffer, MESSAGE_SIZE, format, args);
    va_end(args);

    // Prepare log entry: timestamp followed by message
    memset(log_buffer, 0, LOG_ENTRY_SIZE); // Clear buffer
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE); // Copy timestamp
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message_buffer, MESSAGE_SIZE - 1); // Copy message

    // Calculate flash address for the current log slot
    uint32_t address = LOG_START_ADDR + (next_slot * LOG_ENTRY_SIZE);
    // Write log entry to flash in 8-byte chunks
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < LOG_ENTRY_SIZE; i += 8) {
        uint64_t data = *(uint64_t *)(log_buffer + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data);
    }
    HAL_FLASH_Lock();

    // Update the next slot in the circular buffer and store it in flash
    next_slot = (next_slot + 1) % NUM_LOG_ENTRIES;
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot);
    HAL_FLASH_Lock();
}

/**
  * @brief  Reads all logs from flash memory and sends them over RS485 to the OBC.
  * @retval None (transmits logs via UART).
  * @note   - Reads `NUM_LOG_ENTRIES` logs from flash starting at `LOG_START_ADDR`.
  *         - Each log entry includes a timestamp and message, formatted as a string.
  *         - Transmits logs using UART1 (RS485) with a blocking call (`HAL_MAX_DELAY`).
  * @context Called every 10 seconds in the main loop (`main`) to send diagnostic logs to the OBC.
  *          Logs provide critical insights into BMS operation (e.g., faults, mode changes) for
  *          ground station analysis, essential for CubeSat mission success.
  * @integration The STM32 uses UART1 (configured in `MX_USART1_UART_Init`) for RS485 communication.
  *              The logs are transmitted as ASCII strings, ensuring compatibility with OBC logging
  *              systems. The blocking nature ensures all logs are sent before proceeding.
  * @debug  - If logs are not received by the OBC, check UART1 configuration (115200 baud, 8N1),
  *           RS485 bus termination, and ensure the DE pin is properly controlled (not shown here).
  *         - Verify that `NUM_LOG_ENTRIES` and `LOG_START_ADDR` are correctly defined.
  */
void Log_Read_All(void)
{
    char buffer[128]; // Buffer to format log entries as strings
    // Iterate through all log entries in flash
    for (uint32_t i = 0; i < NUM_LOG_ENTRIES; i++) {
        uint32_t address = LOG_START_ADDR + (i * LOG_ENTRY_SIZE); // Calculate log entry address
        uint64_t timestamp = *(uint64_t *)address;                // Read timestamp
        char *message = (char *)(address + TIMESTAMP_SIZE);       // Read message
        // Format log entry as "Log <index>: Time=<timestamp>, Msg=<message>"
        snprintf(buffer, sizeof(buffer), "Log %lu: Time=%llu, Msg=%s\r\n", i, timestamp, message);
        // Transmit log entry over UART1 (RS485)
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

/**
  * @brief  Initializes the logging system by setting up the flash memory log region.
  * @retval None (modifies flash memory and `next_slot`).
  * @note   - Loads the `next_slot` value from flash (`NEXT_SLOT_ADDR`) to resume logging.
  *         - If `next_slot` is invalid (>= `NUM_LOG_ENTRIES`), erases the log page and resets it.
  *         - Ensures the log region is ready for writing new entries.
  * @context Called during system initialization in `main` to prepare the flash-based logging system.
  *          Logging is critical for diagnostics in a CubeSat, where real-time debugging is not possible.
  * @integration The STM32’s flash memory is used to store logs persistently, ensuring diagnostic data
  *              survives power cycles. The circular buffer approach (`next_slot`) minimizes flash wear.
  * @debug  - If initialization fails, check `NEXT_SLOT_ADDR` alignment and flash write permissions.
  *         - Ensure `NUM_LOG_ENTRIES` and `LOG_ENTRY_SIZE` are correctly defined in `main.h`.
  */
void Log_Init(void)
{
    // Load the next slot index from flash
    next_slot = *(uint32_t *)NEXT_SLOT_ADDR;
    // If the slot index is invalid, erase the log region and reset
    if (next_slot >= NUM_LOG_ENTRIES) {
        Flash_Erase(FLASH_LOG_PAGE); // Erase the log page
        next_slot = 0;               // Reset slot index
        HAL_FLASH_Unlock();
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot); // Store new slot index
        HAL_FLASH_Lock();
    }
}

/**
  * @brief  Updates the State of Charge (SOC) and State of Health (SOH) of the battery.
  * @retval None (updates global `soc` and `soh` variables).
  * @note   - Uses coulomb counting to estimate SOC by integrating the average current over time.
  *         - Applies a Kalman filter to reduce noise in SOC and SOH estimates.
  *         - Updates SOH when the battery is fully charged (SOC >= 100%).
  *         - Clamps SOC and SOH to the range [0, 100].
  * @context Called in the main loop (`main`) to continuously update SOC and SOH, which are critical
  *          for battery management in a CubeSat. SOC determines charging needs (`charge_immediately`),
  *          and SOH indicates battery degradation over time.
  * @integration The STM32 uses the Kalman filter (`kalman_filter.c`) to smooth noisy measurements from
  *              coulomb counting, ensuring accurate SOC/SOH estimates. These values are sent to the OBC
  *              via SSP (`SSP_SendStatus`) for monitoring.
  * @debug  - If SOC/SOH values are erratic, check `pack_current_1` and `pack_current_2` for accuracy
  *           (via BQ76920 readings) and verify Kalman filter parameters (`soc_kf`, `soh_kf`).
  *         - Ensure `LOOP_TIME` matches the actual loop duration to avoid integration errors.
  */
void Update_SOC_SOH(void)
{
    // Coulomb counting: Integrate average current over LOOP_TIME to estimate charge
    // Average current from both BQ76920 ICs, scaled to mAh (current in mA, time in seconds)
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0;
    // Calculate raw SOC as a percentage of nominal capacity
    float soc_measured = (coulomb_count / battery_config.nominal_capacity) * 100.0;
    // Apply Kalman filter to smooth SOC estimate
    soc = KalmanFilter_Update(&soc_kf, soc_measured);
    // Clamp SOC to valid range [0, 100]
    if (soc > 100.0) soc = 100.0;
    if (soc < 0.0) soc = 0.0;

    // Update SOH when the battery is fully charged (SOC >= 100%)
    if (soc >= 100.0) {
        actual_capacity = coulomb_count; // Update actual capacity based on coulomb count
        // Calculate SOH as a percentage of initial capacity
        soh = (actual_capacity / initial_capacity) * 100.0;
        // Apply Kalman filter to smooth SOH estimate
        soh = KalmanFilter_Update(&soh_kf, soh);
        // Clamp SOH to valid range [0, 100]
        if (soh > 100.0) soh = 100.0;
        if (soh < 0.0) soh = 0.0;
    }
}

/**
  * @brief  Updates the BMS operating mode and charge/discharge status based on system conditions.
  * @retval None (updates global `bms_mode`, `charge_enabled`, and `discharge_enabled`).
  * @note   - Handles fault states (e.g., overvoltage, overtemperature) by entering MODE_FAULT and
  *           disabling charging/discharging as needed.
  *         - Attempts recovery from faults with timeouts and reinitialization (e.g., for DEVICE_XREADY).
  *         - In normal operation, determines mode (charging, discharging, sleep) based on current and SOC.
  *         - Applies charge/discharge settings to both BQ76920 ICs for redundancy.
  * @context Called in the main loop (`main`) to manage the BMS’s operating state, ensuring safety and
  *          proper operation in a CubeSat. Fault handling protects the battery, while normal mode
  *          logic optimizes power usage based on load and SOC.
  * @integration The STM32 uses this function to manage the BMS state machine, interfacing with the
  *              BQ76920 ICs via I2C (`BQ76920_SetChargeEnable`) and logging faults for diagnostics.
  *              The state is communicated to the OBC via SSP (`SSP_SendStatus`).
  * @debug  - If the BMS enters MODE_FAULT unexpectedly, check `error_flags` and trace the source
  *           (e.g., voltage, current, temperature readings).
  *         - If recovery fails, verify BQ76920 initialization (`BQ76920_Init`) and I2C communication.
  *         - Ensure timeouts (`FAULT_TIMEOUT`, `TEMP_FAULT_TIMEOUT`) are appropriate for the system.
  */
void Update_BMS_Mode(void)
{
    // Static variables for fault state management
    static uint32_t fault_start_time = 0;      // Timestamp when fault mode is entered
    static uint8_t in_fault_mode = 0;          // Flag indicating if the BMS is in fault mode
    static uint8_t recovery_attempts = 0;      // Counter for recovery attempts (e.g., for DEVICE_XREADY)
    static const uint8_t MAX_RECOVERY_ATTEMPTS = 3; // Maximum recovery attempts before reset
    static const uint32_t FAULT_TIMEOUT = 30000;    // Timeout for most faults (30 seconds)
    static const uint32_t TEMP_FAULT_TIMEOUT = 60000; // Timeout for temperature faults (60 seconds)
    static const uint32_t COOLDOWN_PERIOD = 10000;    // Cooldown period before checking fault clearance (10 seconds)
    static const uint32_t RECOVERY_DELAY = 5000;      // Delay before attempting recovery (5 seconds)

    // Check for any fault conditions
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OCC | ERROR_OCD | ERROR_SCD | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY | ERROR_DEVICE_XREADY | ERROR_OVRD_ALERT))
    {
        // Enter fault mode if not already in it
        if (!in_fault_mode)
        {
            fault_start_time = HAL_GetTick(); // Record fault entry time
            in_fault_mode = 1;                // Set fault mode flag
        }

        bms_mode = MODE_FAULT; // Set BMS mode to fault

        // Handle overvoltage fault
        if (error_flags & ERROR_OVERVOLTAGE)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 1; // Allow discharging
            Log_Error("Protective action: Disabled charging due to overvoltage");

            // Check if all cells are below the overvoltage threshold to clear the fault
            uint8_t all_below_threshold = 1;
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                if (group_voltages_1[i] > battery_config.ov_threshold || group_voltages_2[i] > battery_config.ov_threshold)
                {
                    all_below_threshold = 0;
                    break;
                }
            }
            if (all_below_threshold)
            {
                Log_Error("Overvoltage fault cleared");
                error_flags &= ~ERROR_OVERVOLTAGE; // Clear fault flag
                in_fault_mode = 0;                // Exit fault mode
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                Log_Error("Overvoltage fault persists, triggering system reset");
                HAL_NVIC_SystemReset(); // Reset system if fault persists
            }
        }
        // Handle undervoltage fault
        else if (error_flags & ERROR_UNDERVOLTAGE)
        {
            charge_enabled = 1;  // Allow charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled discharging due to undervoltage");

            // Check if all cells are above the undervoltage threshold to clear the fault
            uint8_t all_above_threshold = 1;
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                if (group_voltages_1[i] < battery_config.uv_threshold || group_voltages_2[i] < battery_config.uv_threshold)
                {
                    all_above_threshold = 0;
                    break;
                }
            }
            if (all_above_threshold)
            {
                Log_Error("Undervoltage fault cleared");
                error_flags &= ~ERROR_UNDERVOLTAGE; // Clear fault flag
                in_fault_mode = 0;                 // Exit fault mode
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                Log_Error("Undervoltage fault persists, triggering system reset");
                HAL_NVIC_SystemReset(); // Reset system if fault persists
            }
        }
        // Handle overcurrent during charging (OCC)
        else if (error_flags & ERROR_OCC)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 1; // Allow discharging
            Log_Error("Protective action: Disabled charging due to overcurrent charge");

            // Wait for cooldown, then check if current is safe (current should be non-negative)
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                int16_t total_current = (pack_current_1 + pack_current_2) / 2;
                if (total_current >= 0)
                {
                    Log_Error("Overcurrent charge fault cleared");
                    error_flags &= ~ERROR_OCC; // Clear fault flag
                    in_fault_mode = 0;        // Exit fault mode
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Overcurrent charge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset(); // Reset system if fault persists
                }
            }
        }
        // Handle overcurrent during discharging (OCD)
        else if (error_flags & ERROR_OCD)
        {
            charge_enabled = 1;  // Allow charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled discharging due to overcurrent discharge");

            // Wait for cooldown, then check if current is safe (current should be non-positive)
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                int16_t total_current = (pack_current_1 + pack_current_2) / 2;
                if (total_current <= 0)
                {
                    Log_Error("Overcurrent discharge fault cleared");
                    error_flags &= ~ERROR_OCD; // Clear fault flag
                    in_fault_mode = 0;        // Exit fault mode
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Overcurrent discharge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset(); // Reset system if fault persists
                }
            }
        }
        // Handle short-circuit discharge (SCD)
        else if (error_flags & ERROR_SCD)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled charging and discharging due to short-circuit discharge");

            // Check if the short-circuit condition is cleared after timeout
            if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                uint8_t status1, status2;
                uint8_t scd_cleared = 1;
                // Check SCD bit (bit 3) in BQ76920 status registers
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 3)))
                {
                    scd_cleared = 0; // SCD still active on IC1
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 3)))
                {
                    scd_cleared = 0; // SCD still active on IC2
                }
                if (scd_cleared)
                {
                    Log_Error("Short-circuit discharge fault cleared");
                    error_flags &= ~ERROR_SCD; // Clear fault flag
                    in_fault_mode = 0;        // Exit fault mode
                }
                else
                {
                    Log_Error("Short-circuit discharge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset(); // Reset system if fault persists
                }
            }
        }
        // Handle overtemperature fault
        else if (error_flags & ERROR_OVERTEMP)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled charging and discharging due to overtemperature");

            // Check if temperatures are safe with a 10°C hysteresis
            if (temperature_1 < (battery_config.overtemp_threshold - 10) && temperature_2 < (battery_config.overtemp_threshold - 10) && pcb_temperature < (battery_config.overtemp_threshold - 10))
            {
                Log_Error("Overtemperature fault cleared");
                error_flags &= ~ERROR_OVERTEMP; // Clear fault flag
                in_fault_mode = 0;             // Exit fault mode
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT)
            {
                Log_Error("Overtemperature fault persists, triggering system reset");
                HAL_NVIC_SystemReset(); // Reset system if fault persists
            }
        }
        // Handle undertemperature fault
        else if (error_flags & ERROR_UNDERTEMP)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 1; // Allow discharging
            Log_Error("Protective action: Disabled charging due to undertemperature");

            // Check if temperatures are safe with a 10°C hysteresis
            if (temperature_1 > (battery_config.undertemp_threshold + 10) && temperature_2 > (battery_config.undertemp_threshold + 10))
            {
                Log_Error("Undertemperature fault cleared");
                error_flags &= ~ERROR_UNDERTEMP; // Clear fault flag
                in_fault_mode = 0;              // Exit fault mode
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT)
            {
                Log_Error("Undertemperature fault persists, triggering system reset");
                HAL_NVIC_SystemReset(); // Reset system if fault persists
            }
        }
        // Handle redundancy discrepancy between BQ76920 ICs
        else if (error_flags & ERROR_DISCREPANCY)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled charging and discharging due to redundancy discrepancy");

            // Attempt reinitialization after delay
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY)
            {
                Log_Error("Attempting to reinitialize BQ76920 ICs to resolve discrepancy");
                if (BQ76920_Init(&hi2c1) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)");
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)");
                }

                // Recheck redundancy after reinitialization
                uint8_t discrepancy_flag = 0;
                BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
                if (!discrepancy_flag)
                {
                    Log_Error("Redundancy discrepancy fault cleared");
                    error_flags &= ~ERROR_DISCREPANCY; // Clear fault flag
                    in_fault_mode = 0;                // Exit fault mode
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Redundancy discrepancy fault persists, triggering system reset");
                    HAL_NVIC_SystemReset(); // Reset system if fault persists
                }
            }
        }
        // Handle DEVICE_XREADY fault (BQ76920 initialization issue)
        else if (error_flags & ERROR_DEVICE_XREADY)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled charging and discharging due to DEVICE_XREADY");

            // Attempt reinitialization with limited retries
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY)
            {
                recovery_attempts++;
                Log_Error("Attempting to reinitialize BQ76920 ICs (attempt %d)", recovery_attempts);
                uint8_t init_success = 1;
                if (BQ76920_Init(&hi2c1) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)");
                    init_success = 0;
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)");
                    init_success = 0;
                }

                if (init_success)
                {
                    Log_Error("DEVICE_XREADY fault cleared");
                    error_flags &= ~ERROR_DEVICE_XREADY; // Clear fault flag
                    in_fault_mode = 0;                  // Exit fault mode
                    recovery_attempts = 0;              // Reset attempts
                }
                else if (recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    Log_Error("Failed to recover from DEVICE_XREADY after %d attempts, triggering system reset", MAX_RECOVERY_ATTEMPTS);
                    HAL_NVIC_SystemReset(); // Reset system if recovery fails
                }
            }
        }
        // Handle OVRD_ALERT fault (general alert from BQ76920)
        else if (error_flags & ERROR_OVRD_ALERT)
        {
            charge_enabled = 0;  // Disable charging
            discharge_enabled = 0; // Disable discharging
            Log_Error("Protective action: Disabled charging and discharging due to OVRD_ALERT");

            // Check if the alert condition is cleared after cooldown
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                uint8_t status1, status2;
                uint8_t alert_cleared = 1;
                // Check OVRD_ALERT bit (bit 6) in BQ76920 status registers
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 6)))
                {
                    alert_cleared = 0; // Alert still active on IC1
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 6)))
                {
                    alert_cleared = 0; // Alert still active on IC2
                }
                if (alert_cleared)
                {
                    Log_Error("OVRD_ALERT fault cleared");
                    error_flags &= ~ERROR_OVRD_ALERT; // Clear fault flag
                    in_fault_mode = 0;               // Exit fault mode
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("OVRD_ALERT fault persists, triggering system reset");
                    HAL_NVIC_SystemReset(); // Reset system if fault persists
                }
            }
        }

        // Apply charge/discharge settings to both BQ76920 ICs for redundancy
        BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled);
        BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled);
        return; // Exit function while in fault mode
    }

    // Normal operation: Reset fault state
    in_fault_mode = 0;
    recovery_attempts = 0;

    // Trigger immediate charging if SOC is below threshold
    charge_immediately = (soc < battery_config.soc_low_threshold) ? 1 : 0;

    // Determine operating mode based on average current
    int16_t total_current = (pack_current_1 + pack_current_2) / 2;
    if (total_current < 0) { // Negative current indicates charging
        bms_mode = MODE_CHARGING;
        charge_enabled = 1;
        discharge_enabled = 0;
    } else if (total_current > 0) { // Positive current indicates discharging
        bms_mode = MODE_DISCHARGING;
        charge_enabled = 0;
        discharge_enabled = 1;
    } else { // No current flow
        if (soc < battery_config.soc_low_threshold) { // Charge if SOC is low
            bms_mode = MODE_CHARGING;
            charge_enabled = 1;
            discharge_enabled = 0;
        } else { // Enter sleep mode if SOC is sufficient
            bms_mode = MODE_SLEEP;
            charge_enabled = 0;
            discharge_enabled = 0;
        }
    }

    // Apply charge/discharge settings to both BQ76920 ICs
    BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled);
    BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled);
}

/**
  * @brief  Reads the internal temperature sensor of the STM32 microcontroller.
  * @retval int16_t: Temperature in degrees Celsius (°C).
  * @note   - Uses ADC1 to measure the MCU’s internal temperature sensor.
  *         - Conversion formula is based on STM32L4 reference data: (V_SENSE * V_REF / ADC_MAX - V_25) * 100 / SLOPE + 25.
  *         - Assumes V_REF = 3.3V, ADC_MAX = 4096 (12-bit), V_25 = 760 mV, SLOPE = 2.5 mV/°C.
  * @context Called in the main loop (`main`) to monitor the PCB temperature, which is used for
  *          overtemperature protection (`ERROR_OVERTEMP`) alongside battery temperatures.
  *          PCB temperature monitoring protects the electronics in the CubeSat’s harsh environment.
  * @integration The STM32’s ADC1 (configured in `MX_ADC1_Init`) is used to read the internal
  *              temperature sensor, providing a third temperature measurement point alongside
  *              NTC-1 and NTC-2 (via `Temperature_Read`).
  * @debug  - If temperature readings are inaccurate, verify ADC calibration and the conversion formula.
  *         - Ensure ADC1 is properly initialized and not conflicting with other ADC channels.
  *         - Use a reference thermometer to calibrate the internal sensor if needed.
  */
int16_t Read_Internal_Temperature(void)
{
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);
    // Wait for conversion to complete
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    // Read raw ADC value (12-bit, 0-4095)
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    // Convert ADC reading to temperature (°C)
    // Formula: (V_SENSE * 3300 / 4096 - 760) * 100 / 250 + 25
    // Where V_SENSE is the ADC reading scaled to voltage, 760 mV is V_25, 2.5 mV/°C is the slope
    int32_t temp = ((raw * 3300 / 4096) - 760) * 100 / 250 + 25;
    return (int16_t)temp;
}

/**
  * @brief  Sends the current BMS status to the OBC using the SSP protocol.
  * @retval None (transmits telemetry frame via RS485).
  * @note   - Constructs a telemetry frame with battery status (voltages, currents, SOC, SOH, etc.).
  *         - Sends the frame every 5 seconds or on request (e.g., SSP CMD_GOSTM).
  *         - Controls the RS485 DE pin to switch between transmit and receive modes.
  * @context Called in the main loop (`main`) to periodically send telemetry to the OBC, and also
  *          in response to SSP commands (e.g., GOSTM, GSTLM, GOTLM). Telemetry is critical for
  *          ground station monitoring of the CubeSat’s power system.
  * @integration The STM32 uses USART2 (configured in `MX_USART2_Init`) for RS485 communication.
  *              The SSP protocol (`ssp.c`) ensures reliable data exchange with the OBC. The DE pin
  *              is controlled via GPIO to manage half-duplex communication.
  * @debug  - If the OBC does not receive telemetry, check RS485 bus connectivity, DE pin toggling,
  *           and USART2 configuration (115200 baud, 8N1).
  *         - Verify that telemetry data (e.g., voltages, SOC) is correctly packed (`SSP_PackTelemetry`).
  */
void SSP_SendStatus(void)
{
    SSP_TelemetryTypeDef telemetry = {0}; // Structure to hold telemetry data
    SSP_FrameTypeDef frame = {0};         // SSP frame to transmit
    uint16_t frame_len;                   // Length of the constructed frame

    // Calculate total pack voltage by summing cell voltages from IC1
    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2] + group_voltages_1[3];
    // Populate telemetry structure with current BMS status
    telemetry.charge_immediately = charge_immediately;
    telemetry.bms_online = bms_online;
    telemetry.error_flags = error_flags;
    telemetry.pack_voltage_1 = (uint16_t)pack_voltage; // Total pack voltage from IC1
    telemetry.pack_voltage_2 = (uint16_t)pack_voltage; // Same for IC2 (redundancy check done elsewhere)
    telemetry.pack_current_1 = pack_current_1;
    telemetry.pack_current_2 = pack_current_2;
    telemetry.soc = (uint8_t)soc;
    telemetry.soh = (uint8_t)soh;
    telemetry.temp_1 = temperature_1;
    telemetry.temp_2 = temperature_2;
    telemetry.pcb_temp = pcb_temperature;
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        telemetry.group_voltages[i] = group_voltages_1[i]; // Cell voltages from IC1
    }
    telemetry.balancing_active = balancing_active;
    telemetry.balancing_mask_1 = balancing_mask_1;
    telemetry.balancing_mask_2 = balancing_mask_2;
    telemetry.charge_cycle_count = charge_cycle_count;
    telemetry.total_charge_time = total_charge_time;
    telemetry.total_discharge_time = total_discharge_time;
    telemetry.total_operating_time = total_operating_time;

    // Construct and send the SSP frame
    SSP_PackTelemetry(&telemetry, &frame); // Pack telemetry data into frame
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len); // Serialize frame
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET); // Enable RS485 transmit
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len); // Transmit frame
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); // Disable RS485 transmit
}

/**
  * @brief  Processes a received SSP frame from the OBC and responds accordingly.
  * @param  frame: Pointer to the received SSP frame structure.
  * @retval None (processes frame and sends response via RS485).
  * @note   - Handles commands like PING, SON/SOF (power line control), KEN (mission termination),
  *           SM (set mode), GM (get mode), GOSTM/GSTLM/GOTLM (get status), and SFP (set parameters).
  *         - Responds with ACK/NACK frames, logging actions for diagnostics.
  *         - Ignores frames not addressed to EPS or marked as replies/time-tagged.
  * @context Called in the main loop (`main`) when an SSP frame is received (`SSP_ReceiveFrame`).
  *          Enables the OBC to control the BMS (e.g., turn power lines on/off, request status) and
  *          supports firmware updates, critical for CubeSat mission operations.
  * @integration The STM32 uses USART2 (RS485) to receive commands and send responses via SSP (`ssp.c`).
  *              GPIO controls the RS485 DE pin for half-duplex communication. Commands like SON/SOF
  *              interact with the `power_lines` array, which may control TPS22810 load switches.
  * @debug  - If commands are not processed, verify the frame’s destination (`dest`) and command ID (`cmd_id`).
  *         - Check RS485 bus connectivity and DE pin toggling for response transmission.
  *         - Log received frames (`log_msg`) to trace command execution and errors.
  */
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame)
{
    // Filter frames: Ignore those not addressed to EPS, broadcast, or multicast
    if (frame->dest != SSP_ADDR_EPS && frame->dest != SSP_ADDR_BROADCAST && frame->dest != SSP_ADDR_MULTICAST) {
        return;
    }

    // Ignore reply frames (responses to our requests)
    if (frame->cmd_id & SSP_FRAME_TYPE_REPLY) {
        return;
    }

    // Ignore time-tagged commands (handled by the OBC)
    if (frame->cmd_id & SSP_CMD_TYPE_TIMETAG) {
        return;
    }

    // Log the received command for diagnostics
    char log_msg[MESSAGE_SIZE];
    snprintf(log_msg, sizeof(log_msg), "Received CMD: ID=0x%02X, SRC=0x%02X, LEN=%d", frame->cmd_id, frame->src, frame->data_len);
    Log_Error(log_msg);

    // Prepare response frame: Default to NACK with command ID echoed back
    SSP_FrameTypeDef response = {0};
    response.dest = frame->src;      // Respond to sender
    response.src = SSP_ADDR_EPS;     // Source is EPS
    response.data_len = 1;           // Default response length
    response.data[0] = frame->cmd_id; // Echo command ID

    // Process the command based on its ID (mask out flags to get base command)
    switch (frame->cmd_id & 0x3F) {
        case SSP_CMD_PING: // 0x00: Ping request
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; // Respond with ACK
            break;

        case SSP_CMD_SON: // 0x0B: Switch ON a power line
            if (frame->data_len == 1) { // Expect exactly 1 byte (power line ID)
                uint8_t pwrl_id = frame->data[0];
                if (pwrl_id <= 15) { // Valid power line ID (0-15)
                    power_lines[pwrl_id] = 1; // Turn on power line
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    snprintf(log_msg, sizeof(log_msg), "SON command: PWRL%d ON", pwrl_id);
                    Log_Error(log_msg);
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid ID
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_SOF: // 0x0C: Switch OFF a power line
            if (frame->data_len == 1) { // Expect exactly 1 byte (power line ID)
                uint8_t pwrl_id = frame->data[0];
                if (pwrl_id <= 15) { // Valid power line ID (0-15)
                    power_lines[pwrl_id] = 0; // Turn off power line
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    snprintf(log_msg, sizeof(log_msg), "SOF command: PWRL%d OFF", pwrl_id);
                    Log_Error(log_msg);
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid ID
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_KEN: // 0x31: Mission termination request
            if (frame->data_len == 0) { // Expect no data
                if (mission_termination_enabled) { // Check if termination is enabled (via SFP)
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("KEN command executed: Mission termination enabled");
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("KEN command rejected: Mission termination not enabled");
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_KDIS: // 0x32: Reconnect batteries (undo KEN)
            if (frame->data_len == 0) { // Expect no data
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                Log_Error("KDIS command executed: Batteries reconnected");
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_SM: // 0x15: Set BMS mode
            if (frame->data_len >= 1) { // Expect at least 1 byte (mode)
                bms_mode = frame->data[0]; // Update BMS mode
                Update_BMS_Mode();        // Apply mode change
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                Log_Error("Mode changed by OBC");
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_GM: // 0x16: Get BMS mode
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
            response.data_len = 1;
            response.data[0] = bms_mode; // Respond with current mode
            break;

        case SSP_CMD_GOSTM: // 0x25: Get operational status (telemetry)
            SSP_SendStatus(); // Send telemetry, no additional response needed
            return;

        case SSP_CMD_SFP: // 0x1B: Set flight parameter
            if (frame->data_len >= 2) { // Expect param_id and param_value
                uint8_t param_id = frame->data[0];
                uint8_t param_value = frame->data[1];
                if (param_id == 0x01) { // Set mission termination enable
                    mission_termination_enabled = param_value;
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("Mission termination enable set");
                }
                else if (param_id == 0x02) { // Request firmware update
                    if (param_value == 1) {
                        // Set firmware update flag in flash and reboot
                        HAL_FLASH_Unlock();
                        Flash_Erase(FLASH_LOG_PAGE);
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xDEADBEEF);
                        HAL_FLASH_Lock();
                        Log_Error("Firmware update requested, rebooting...");
                        HAL_Delay(100);
                        HAL_NVIC_SystemReset();
                    }
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                }
                else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Unknown parameter
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; // Invalid data length
            }
            break;

        case SSP_CMD_GSTLM: // 0x22: Get short telemetry
        case SSP_CMD_GOTLM: // 0x21: Get operational telemetry
            SSP_SendStatus(); // Send telemetry, no additional response needed
            return;

        default: // Unknown command
            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            break;
    }

    // Send the response frame
    uint16_t frame_len;
    SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len);
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET); // Enable RS485 transmit
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); // Disable RS485 transmit
}

/**
  * @brief  Implements the Constant Current - Constant Voltage (CC-CV) charging algorithm.
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR if temperature exceeds limits or I2C fails.
  * @note   - Switches between CC and CV modes based on cell voltages.
  *         - Includes safety checks for overtemperature conditions.
  *         - Controls a charging enable pin (GPIOE Pin 11) to manage the charging circuit.
  *         - Uses `max_charge_time` to prevent overcharging.
  * @context Called in the main loop (`main`) when the BMS is in charging mode (`MODE_CHARGING`).
  *          Ensures safe and efficient charging of the lithium-ion battery pack in the CubeSat,
  *          balancing charge speed and battery longevity.
  * @integration The STM32 uses I2C to read temperatures (`Temperature_Read`) and GPIO to control
  *              the charging circuit. The CC-CV algorithm leverages voltage readings from the BQ76920
  *              ICs to manage charging phases, integrating with the overall BMS state machine.
  * @debug  - If charging fails (HAL_ERROR), check I2C communication with temperature sensors and
  *           ensure GPIOE Pin 11 is correctly configured.
  *         - If the BMS does not transition to CV mode, verify cell voltage readings (`group_voltages_1/2`)
  *           and the `cv_threshold` value in `battery_config`.
  *         - Monitor `charge_duration` to ensure the timeout mechanism works as expected.
  */
HAL_StatusTypeDef ChargeBattery(void)
{
    int16_t temperature_1, temperature_2; // Variables to store temperature readings
    HAL_StatusTypeDef status;

    // Read temperatures from both NTC sensors
    status = Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2);
    if (status != HAL_OK || temperature_1 == INT16_MIN || temperature_2 == INT16_MIN)
    {
        // Disable charging if temperature reading fails
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        in_cv_mode = 0;         // Reset CV mode
        charge_start_time = 0;  // Reset charging timer
        return HAL_ERROR;
    }

    // Use the highest temperature for safety checks
    int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;

    // Check for overtemperature condition (battery or PCB)
    if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold)
    {
        // Disable charging if temperature exceeds limits
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        in_cv_mode = 0;         // Reset CV mode
        charge_start_time = 0;  // Reset charging timer
        return HAL_ERROR;
    }

    // CC-CV charging logic
    if (!in_cv_mode)
    {
        // Constant Current (CC) mode
        if (charge_start_time == 0)
        {
            charge_start_time = HAL_GetTick(); // Start charging timer
        }

        // Check for transition to Constant Voltage (CV) mode
        int16_t max_voltage = 0;
        // Find the maximum cell voltage across both ICs
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
        {
            if (group_voltages_1[i] > max_voltage) max_voltage = group_voltages_1[i];
            if (group_voltages_2[i] > max_voltage) max_voltage = group_voltages_2[i];
        }
        if (max_voltage > battery_config.cv_threshold)
        {
            in_cv_mode = 1; // Transition to CV mode
        }
    }
    else
    {
        // Constant Voltage (CV) mode: Check for timeout
        uint32_t charge_duration = (HAL_GetTick() - charge_start_time) / 1000; // Duration in seconds
        if (charge_duration > battery_config.max_charge_time)
        {
            // Disable charging if maximum charge time is exceeded
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
            in_cv_mode = 0;         // Reset CV mode
            charge_start_time = 0;  // Reset charging timer
        }
    }

    return HAL_OK; // Charging successful
}

/**
  * @brief  Checks if the system should enter bootloader mode for a firmware update.
  * @retval None (sets global `firmware_update_mode` and logs action).
  * @note   - Reads a firmware update flag (0xDEADBEEF) from flash (`FIRMWARE_UPDATE_FLAG_ADDR`).
  *         - If set, enters update mode and clears the flag to prevent re-entry.
  *         - Logs the decision for diagnostics.
  * @context Called during system startup in `main` to determine if a firmware update is requested
  *          (e.g., via SSP CMD_SFP). Firmware updates are critical for CubeSat maintenance and upgrades
  *          in orbit, allowing bug fixes or new features to be deployed.
  * @integration The STM32 uses flash memory to store the update flag persistently, ensuring the update
  *              request survives reboots. The bootloader mode interacts with `Bootloader_FirmwareUpdate`
  *              to handle the update process.
  * @debug  - If the system does not enter update mode when expected, check the flash address
  *           (`FIRMWARE_UPDATE_FLAG_ADDR`) and ensure the SFP command sets the correct value (0xDEADBEEF).
  *         - Verify that flash write operations succeed (`Flash_Erase`, `HAL_FLASH_Program`).
  */
void Bootloader_Check(void)
{
    // Read the firmware update flag from flash
    uint32_t firmware_update_flag = *(volatile uint32_t *)FIRMWARE_UPDATE_FLAG_ADDR;

    if (firmware_update_flag == 0xDEADBEEF)
    {
        firmware_update_mode = 1; // Set update mode
        Log_Error("Entering firmware update mode");

        // Clear the flag to prevent re-entry after update
        HAL_FLASH_Unlock();
        Flash_Erase(FLASH_LOG_PAGE);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xFFFFFFFF);
        HAL_FLASH_Lock();
    }
    else
    {
        firmware_update_mode = 0; // Normal operation mode
        Log_Error("Booting to application");
    }
}

/**
  * @brief  Handles the firmware update process over RS485.
  * @retval None (receives firmware packets, writes to flash, and reboots).
  * @note   - Receives firmware packets via SSP (CMD_FIRMWARE_UPDATE), each up to 128 bytes.
  *         - Erases the application flash region (`APP_START_ADDR` to `APP_END_ADDR`).
  *         - Writes packets to flash, verifies CRC, and sets a validity flag on completion.
  *         - Times out after 10 seconds (`FIRMWARE_UPDATE_TIMEOUT`) if no packets are received.
  * @context Called in `main` if `firmware_update_mode` is set, allowing the OBC to update the BMS
  *          firmware in orbit. Firmware updates are essential for fixing bugs or adding features
  *          in a CubeSat without physical access.
  * @integration The STM32 uses USART2 (RS485) to receive packets (`SSP_ReceiveFrame`) and flash
  *              memory to store the new firmware. CRC validation (`CalculateCRC16`) ensures data
  *              integrity, and the validity flag (`APP_VALIDITY_FLAG_ADDR`) ensures safe booting.
  * @debug  - If the update fails, check RS485 communication (DE pin, bus termination) and ensure
  *           the OBC sends packets in the correct format (size, CRC).
  *         - Verify flash write operations (`HAL_FLASH_Program`) and CRC calculation.
  *         - Monitor logs (`Log_Error`) to trace progress and errors during the update.
  */
void Bootloader_FirmwareUpdate(void)
{
    SSP_FrameTypeDef received_frame = {0}; // Structure for received SSP frames
    uint32_t last_packet_time = HAL_GetTick(); // Timestamp for timeout
    uint32_t current_address = APP_START_ADDR; // Current flash address for writing
    uint32_t total_bytes_received = 0;         // Total bytes received
    uint32_t expected_firmware_size = 0;       // Expected firmware size (from first packet)
    uint8_t firmware_buffer[FIRMWARE_UPDATE_PACKET_SIZE]; // Buffer for firmware packet data
    uint16_t calculated_crc = 0xFFFF;         // CRC accumulator for received data

    Log_Error("Waiting for firmware update packets...");

    // Erase the application flash region to prepare for new firmware
    HAL_FLASH_Unlock();
    for (uint32_t addr = APP_START_ADDR; addr < APP_END_ADDR; addr += FLASH_PAGE_SIZE) {
        Flash_Erase((addr - FLASH_BASE) / FLASH_PAGE_SIZE);
    }
    HAL_FLASH_Lock();

    // Loop to receive and process firmware packets
    while (1) {
        // Check for timeout if no packets are received
        if (HAL_GetTick() - last_packet_time > FIRMWARE_UPDATE_TIMEOUT) {
            Log_Error("Firmware update timeout, rebooting...");
            HAL_NVIC_SystemReset();
        }

        // Receive an SSP frame
        if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK) {
            last_packet_time = HAL_GetTick(); // Update timestamp

            // Filter frames: Ignore those not addressed to EPS or marked as replies
            if (received_frame.dest != SSP_ADDR_EPS && received_frame.dest != SSP_ADDR_BROADCAST) {
                continue;
            }
            if (received_frame.cmd_id & SSP_FRAME_TYPE_REPLY) {
                continue;
            }

            // Prepare response frame
            SSP_FrameTypeDef response = {0};
            response.dest = received_frame.src;
            response.src = SSP_ADDR_EPS;
            response.data_len = 1;
            response.data[0] = received_frame.cmd_id;

            // Process firmware update command
            switch (received_frame.cmd_id & 0x3F) {
                case SSP_CMD_FIRMWARE_UPDATE:
                    // First packet: Expect firmware size (4 bytes)
                    if (received_frame.data_len < 4 && total_bytes_received == 0) {
                        response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                        break;
                    }

                    if (total_bytes_received == 0) {
                        // Extract expected firmware size from the first packet
                        expected_firmware_size = (received_frame.data[0] << 24) |
                                                 (received_frame.data[1] << 16) |
                                                 (received_frame.data[2] << 8) |
                                                 received_frame.data[3];
                        Log_Error("Firmware update started, expected size: %lu bytes", expected_firmware_size);
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    } else {
                        // Subsequent packets: Validate size and process data
                        if (received_frame.data_len > FIRMWARE_UPDATE_PACKET_SIZE) {
                            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                            break;
                        }

                        // Copy packet data to firmware buffer
                        memcpy(firmware_buffer, received_frame.data, received_frame.data_len);

                        // Write packet to flash in 8-byte chunks
                        HAL_FLASH_Unlock();
                        for (uint32_t i = 0; i < received_frame.data_len; i += 8) {
                            uint64_t data = *(uint64_t *)(firmware_buffer + i);
                            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_address + i, data);
                        }
                        HAL_FLASH_Lock();

                        // Update CRC with the current packet
                        calculated_crc = CalculateCRC16(firmware_buffer, received_frame.data_len);

                        // Update tracking variables
                        total_bytes_received += received_frame.data_len;
                        current_address += received_frame.data_len;

                        Log_Error("Received %lu/%lu bytes", total_bytes_received, expected_firmware_size);
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;

                        // Complete update if all bytes received
                        if (total_bytes_received >= expected_firmware_size) {
                            // Verify CRC
                            uint16_t received_crc = (firmware_buffer[received_frame.data_len - 2] << 8) |
                                                    firmware_buffer[received_frame.data_len - 1];
                            if (calculated_crc == received_crc) {
                                // Set validity flag and store CRC
                                HAL_FLASH_Unlock();
                                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_VALIDITY_FLAG_ADDR, 0xA5A5A5A5);
                                uint64_t crc_data = calculated_crc;
                                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_END_ADDR - 8, crc_data);
                                HAL_FLASH_Lock();
                                Log_Error("Firmware update completed successfully, rebooting...");
                                HAL_Delay(100);
                                HAL_NVIC_SystemReset();
                            } else {
                                Log_Error("Firmware CRC16 mismatch, rebooting without setting validity flag...");
                                HAL_Delay(100);
                                HAL_NVIC_SystemReset();
                            }
                        }
                    }
                    break;

                default: // Unknown command
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                    break;
            }

            // Send response frame
            uint16_t frame_len;
            SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len);
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);
            SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
        }
    }
}

/**
  * @brief  Validates an application in flash and jumps to its reset handler.
  * @param  start_addr: Starting address of the application in flash (e.g., `APP_START_ADDR` or `BACKUP_START_ADDR`).
  * @retval uint8_t: 1 if the application is valid, 0 if invalid.
  * @note   - Validates the application by checking its CRC and a validity flag (0xA5A5A5A5).
  *         - CRC is stored at the end of the application region (`APP_END_ADDR - 8`).
  * @context Called by `JumpToApplication` to verify the main or backup application before jumping.
  *          Ensures the BMS boots into a valid application, critical for CubeSat reliability after
  *          firmware updates or resets.
  * @integration The STM32 uses flash memory to store the application and its metadata (CRC, validity flag).
  *              The CRC check (`CalculateCRC16`) ensures the application’s integrity before execution.
  * @debug  - If validation fails, check the CRC calculation (`CalculateCRC16`) and ensure the validity
  *           flag and CRC are correctly written during firmware updates (`Bootloader_FirmwareUpdate`).
  *         - Verify that `APP_START_ADDR` and `APP_END_ADDR` are correctly defined.
  */
static uint8_t IsApplicationValid(uint32_t start_addr)
{
    // Calculate CRC of the application (excluding the last 8 bytes where CRC is stored)
    uint16_t crc = CalculateCRC16((uint8_t *)start_addr, APP_END_ADDR - start_addr - 8);
    // Read the stored CRC from flash
    uint16_t stored_crc = *(uint16_t *)(APP_END_ADDR - 8);
    // Check CRC and validity flag
    return (crc == stored_crc) && (*(uint32_t *)APP_VALIDITY_FLAG_ADDR == 0xA5A5A5A5);
}




/**
  * @brief  Jumps to the application code after validation, with fallback to backup.
  * @retval None (jumps to application or halts with LED blinking).
  * @note   - Validates the main application (`APP_START_ADDR`) and jumps if valid.
  *         - Falls back to the backup application (`BACKUP_START_ADDR`) if the main application is invalid.
  *         - Halts with a blinking LED if both applications are invalid, indicating a critical failure.
  *         - Sets the main stack pointer (MSP) and calls the application’s reset handler.
  * @context Called during system startup in `main` to boot into the application code after bootloader
  *          checks. Ensures the BMS runs a valid firmware, critical for CubeSat reliability after updates.
  * @integration The STM32 uses this function to transition from bootloader to application mode,
  *              ensuring safe booting by validating firmware integrity (`IsApplicationValid`).
  *              The LED blinking in failure cases provides visual feedback for debugging.
  * @debug  - If the system halts with LED blinking, check the main and backup application regions
  *           (`APP_START_ADDR`, `BACKUP_START_ADDR`) for valid firmware.
  *         - Verify the validity flag and CRC values stored in flash (`APP_VALIDITY_FLAG_ADDR`, `APP_END_ADDR - 8`)
  *           to ensure they match the expected values.
  *         - Ensure the reset handler addresses (`APP_START_ADDR + 4`, `BACKUP_START_ADDR + 4`) point to valid
  *           functions in the application firmware.
  *         - Use a debugger to inspect the stack pointer (`MSP`) and program counter after the jump to confirm
  *           successful transition to the application.
  */
void JumpToApplication(void)
{
    // Validate the main application at APP_START_ADDR
    if (IsApplicationValid(APP_START_ADDR)) {
        // Read the reset handler address from the application vector table (offset 4)
        uint32_t app_jump_address = *(volatile uint32_t *)(APP_START_ADDR + 4);
        // Cast the address to a function pointer for the reset handler
        void (*app_reset_handler)(void) = (void (*)(void))app_jump_address;
        // Set the main stack pointer (MSP) to the value stored at APP_START_ADDR
        __set_MSP(*(volatile uint32_t *)APP_START_ADDR);
        // Jump to the application’s reset handler to start execution
        app_reset_handler();
    } else {
        // Main application is invalid, attempt to fall back to the backup application
        Log_Error("Main application invalid, falling back to backup...");
        if (IsApplicationValid(BACKUP_START_ADDR)) {
            // Read the reset handler address from the backup vector table
            uint32_t backup_jump_address = *(volatile uint32_t *)(BACKUP_START_ADDR + 4);
            // Cast the address to a function pointer for the reset handler
            void (*backup_reset_handler)(void) = (void (*)(void))backup_jump_address;
            // Set the main stack pointer (MSP) to the value stored at BACKUP_START_ADDR
            __set_MSP(*(volatile uint32_t *)BACKUP_START_ADDR);
            // Jump to the backup application’s reset handler
            backup_reset_handler();
        } else {
            // Both main and backup applications are invalid, enter failure mode
            Log_Error("Backup application also invalid, halting...");
            while (1) {
                // Blink the LED indefinitely to indicate a critical failure
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_Delay(500); // 500 ms blink interval for visibility
            }
        }
    }
}

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
  MX_USART1_UART_Init();
  MX_USART2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
S

    // Set an initial RTC time and date for logging purposes
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours = 12;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sDate.Year = 25; // 2025
    sDate.Month = 3; // March
    sDate.Date = 28; // 28th
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Initialize the logging system to prepare flash memory for error logging
    Log_Init();

    // Check if a firmware update is requested and enter bootloader mode if needed
    Bootloader_Check();

    if (firmware_update_mode)
    {
        // Enter firmware update mode and handle the update process
        Bootloader_FirmwareUpdate();
        HAL_NVIC_SystemReset(); // Reboot after update (should not reach here)
    }

    // Attempt to jump to the application code (main or backup)
    JumpToApplication();

    // Fallback if the jump fails: Log the failure and proceed with normal operation
    Log_Error("Failed to jump to application, entering normal operation");

    // Initialize the BQ76920 ICs for battery monitoring
    if (BQ76920_Init(&hi2c1) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C1) initialization failed");
        Error_Handler();
    }
    if (BQ76920_ConfigureProtection(&hi2c1) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C1) protection configuration failed");
        Error_Handler();
    }

    if (BQ76920_Init(&hi2c2) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C2) initialization failed");
        Error_Handler();
    }
    if (BQ76920_ConfigureProtection(&hi2c2) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C2) protection configuration failed");
        Error_Handler();
    }

    // Initialize Kalman filters for SOC and SOH estimation
    KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0);
    KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0);
    // Initialize PID controller for heater regulation
    PID_Init();

    // Mark the BMS as online and operational
    bms_online = 1;

    // Start LED blinking for 5 seconds to indicate successful startup
    startup_blink_start = HAL_GetTick();
    last_blink_toggle = HAL_GetTick();

    // Initialize battery parameters for coulomb counting
    initial_capacity = battery_config.nominal_capacity;
    actual_capacity = battery_config.nominal_capacity;
    coulomb_count = (INITIAL_SOC / 100.0) * battery_config.nominal_capacity;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    // Timestamps for periodic tasks
    uint32_t last_log_read = 0;    // Last time logs were sent to OBC
    uint32_t last_status_send = 0; // Last time telemetry was sent to OBC
    uint32_t last_time_sync = 0;   // Last time RTC was synchronized with OBC

    // Main loop: Runs every LOOP_TIME for real-time monitoring and control
    while (1)
    {
        // Blink LED for 5 seconds post-startup to indicate system activity
        if (HAL_GetTick() - startup_blink_start < STARTUP_BLINK_DURATION)
        {
            if (HAL_GetTick() - last_blink_toggle >= BLINK_INTERVAL)
            {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                last_blink_toggle = HAL_GetTick();
            }
        }
        else
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // Turn off LED after startup
        }

        // Read cell voltages from the first BQ76920 IC
        if (BQ76920_ReadVoltages(&hi2c1, group_voltages_1, 0) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C1) group voltages");
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                group_voltages_1[i] = 0; // Reset voltages on error
            }
        }
        // Read pack current from the first BQ76920 IC
        if (BQ76920_ReadCurrent(&hi2c1, &pack_current_1) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C1) current");
        }

        // Read cell voltages from the second BQ76920 IC
        if (BQ76920_ReadVoltages(&hi2c2, group_voltages_2, 0) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C2) group voltages");
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                group_voltages_2[i] = 0; // Reset voltages on error
            }
        }
        // Read pack current from the second BQ76920 IC
        if (BQ76920_ReadCurrent(&hi2c2, &pack_current_2) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C2) current");
        }

        // Read battery temperatures from NTC sensors
        if (Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2) != HAL_OK)
        {
            Log_Error("Error reading temperatures (I2C1/I2C2)");
            temperature_1 = INT16_MIN; // Indicate error with sentinel value
            temperature_2 = INT16_MIN;
        }

        // Check for redundancy discrepancies between the two BQ76920 ICs
        uint8_t discrepancy_flag = 0;
        BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
        if (discrepancy_flag)
        {
            error_flags |= ERROR_DISCREPANCY;
            Log_Error("Redundancy discrepancy detected");
        }

        // Read PCB temperature using the STM32’s internal sensor
        pcb_temperature = Read_Internal_Temperature();

        // Check BQ76920 status flags for faults (e.g., overvoltage, overcurrent)
        BQ76920_CheckStatus(&hi2c1, &hi2c2, &error_flags);

        // Check temperature limits for battery and PCB
        int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;
        int16_t lowest_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2;
        if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold)
        {
            error_flags |= ERROR_OVERTEMP;
            Log_Error("Overtemperature detected");
        }
        if (lowest_temp < battery_config.undertemp_threshold)
        {
            error_flags |= ERROR_UNDERTEMP;
            Log_Error("Undertemperature detected");
        }

        // Update operational timers based on current flow
        total_operating_time = HAL_GetTick() / 1000; // Total runtime in seconds
        int16_t total_current = (pack_current_1 + pack_current_2) / 2;
        if (total_current < 0) { // Charging (negative current)
            total_charge_time += (uint32_t)LOOP_TIME; // Increment charge time
            if (soc < 20.0 && !charging_started) {
                charging_started = 1; // Mark start of a charging cycle
            }
            if (soc >= 100.0 && charging_started) {
                charge_cycle_count++; // Increment cycle count on full charge
                charging_started = 0; // Reset charging cycle flag
            }
        } else if (total_current > 0) { // Discharging (positive current)
            total_discharge_time += (uint32_t)LOOP_TIME; // Increment discharge time
        }

        // Perform cell balancing on both BQ76920 ICs
        if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C1)");
        }
        if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C2)");
        }
        // Update balancing active flag based on masks
        balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0;

        // Control heaters using PID based on the lowest battery temperature
        PID_Control(lowest_temp);

        // Update SOC and SOH estimates
        Update_SOC_SOH();

        // Update BMS operating mode and charge/discharge status
        Update_BMS_Mode();

        // Apply CC-CV charging algorithm if in charging mode
        ChargeBattery();

        // Log the current system state for diagnostics
        char message[MESSAGE_SIZE];
        snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick());
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
        {
            char group_data[20];
            snprintf(group_data, sizeof(group_data), "Cell%d: %dmV ", i + 1, group_voltages_1[i]);
            strncat(message, group_data, MESSAGE_SIZE - strlen(message) - 1);
        }
        char temp_data[88];
        snprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC PCB: %dC SOC: %.1f%% SOH: %.1f%%",
                 pack_current_1, pack_current_2, temperature_1, temperature_2, pcb_temperature, soc, soh);
        strncat(message, temp_data, MESSAGE_SIZE - strlen(message) - 1);
        Log_Error(message);

        // Send logs to the OBC every 10 seconds
        if (HAL_GetTick() - last_log_read >= 10000)
        {
            Log_Read_All();
            last_log_read = HAL_GetTick();
        }

        // Send telemetry to the OBC every 5 seconds
        if (HAL_GetTick() - last_status_send >= 5000)
        {
            SSP_SendStatus();
            last_status_send = HAL_GetTick();
        }

        // Synchronize RTC with the OBC every 60 seconds
        if (HAL_GetTick() - last_time_sync >= 60000)
        {
            SSP_TimeTypeDef time = {0};
            if (SSP_RequestTime(&husart2, &time) == HAL_OK)
            {
                // Update RTC with received time
                RTC_TimeTypeDef sTime = {0};
                RTC_DateTypeDef sDate = {0};
                sTime.Hours = time.hour;
                sTime.Minutes = time.minute;
                sTime.Seconds = time.second;
                sDate.Year = (uint8_t)(time.year - 2000);
                sDate.Month = time.month;
                sDate.Date = time.day;
                HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
                Log_Error("Time synchronized with OBC");
            }
            else
            {
                Log_Error("Failed to synchronize time with OBC");
            }
            last_time_sync = HAL_GetTick();
        }

        // Process any received SSP frames from the OBC
        SSP_FrameTypeDef received_frame = {0};
        if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK)
        {
            SSP_ProcessReceivedFrame(&received_frame);
        }

        // Delay to maintain loop timing (LOOP_TIME in seconds)
        HAL_Delay((uint32_t)(LOOP_TIME * 1000));
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HEATER2_Pin|HEATER1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : HEATER2_Pin HEATER1_Pin */
  GPIO_InitStruct.Pin = HEATER2_Pin|HEATER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
