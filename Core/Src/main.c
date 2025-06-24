/* Role of main.c in the BMS Project:
 * This file is the main control program for the BMS in a CubeSat, initializing hardware
 * peripherals (ADC, I2C, UART, RTC) and managing the 4S2P lithium-ion battery pack using
 * two BQ76920 ICs. It controls heaters via PID, detects faults, and communicates with the
 * OBC via RS485 using SSP, while supporting firmware updates and logging.
 */

/* Importance of main.c in the BMS Project:
 * - Orchestrates System: Coordinates battery monitoring, temperature control, fault
 *   handling, and communication, ensuring cohesive BMS operation.
 * - Ensures Safety: Detects faults like overvoltage or overtemperature, taking protective
 *   actions to prevent battery damage.
 * - Enhances Reliability: Manages redundancy between BQ76920 ICs and fault recovery,
 *   critical for space applications.
 * - Enables Diagnostics: Logs errors and telemetry, sent to the OBC via RS485, supporting
 *   remote monitoring.
 * - Optimizes Power: Controls charging, discharging, and heaters, improving battery
 *   performance and longevity in the EPS.
 */

/* Objective of main.c in the BMS Project:
 * The objective is to initialize and manage the BMS’s hardware and software, ensuring safe
 * battery operation, fault handling, and communication with the OBC. It supports firmware
 * updates and logging, contributing to the CubeSat’s mission reliability and EPS efficiency.
 */

/* Include STM32 HAL library for hardware control (UART, I2C, ADC, etc.) */
#include "stm32l4xx_hal.h"
/* Include standard I/O library for string formatting with sprintf */
#include <stdio.h>
/* Include main header file for BMS-specific definitions and structures */
#include "main.h"

/* Define a placeholder for user-defined data types (not used here) */
typedef void *placeholder_typedef; /* Placeholder for custom data types, unused */
/* Define a placeholder for user-defined constants (not used here) */
#define PLACEHOLDER_DEFINE 0 /* Placeholder for custom constants, unused */
/* Define a placeholder for user-defined macros (not used here) */
#define PLACEHOLDER_MACRO(x) (x) /* Placeholder for custom macros, unused */

/* Declare handle for ADC1 to read internal temperature sensor */
ADC_HandleTypeDef hadc1;
/* Declare handle for I2C1 to communicate with first BQ76920 IC */
I2C_HandleTypeDef hi2c1;
/* Declare handle for I2C2 to communicate with second BQ76920 IC */
I2C_HandleTypeDef hi2c2;
/* Declare handle for I2C3 (unused in this code) */
I2C_HandleTypeDef hi2c3;
/* Declare handle for Real-Time Clock to track time */
RTC_HandleTypeDef hrtc;
/* Declare handle for USART1 for logging over RS485 */
UART_HandleTypeDef huart1;
/* Declare handle for USART2 for SSP communication with OBC */
UART_HandleTypeDef huart2;

/* Declare array to store voltages of 4 cells from first BQ76920 (in mV) */
uint16_t group_voltages_1[NUM_GROUPS_PER_IC];
/* Declare array to store voltages of 4 cells from second BQ76920 (in mV) */
uint16_t group_voltages_2[NUM_GROUPS_PER_IC];
/* Declare variable for pack current from first BQ76920 (in mA) */
int16_t pack_current_1;
/* Declare variable for pack current from second BQ76920 (in mA) */
int16_t pack_current_2;
/* Declare variable for temperature from first NTC sensor (°C) */
int16_t temperature_1;
/* Declare variable for temperature from second NTC sensor (°C) */
int16_t temperature_2;
/* Declare variable for PCB temperature from STM32 sensor (°C) */
int16_t pcb_temperature;
/* Declare variable for battery State of Charge (in percent) */
float soc = INITIAL_SOC;
/* Declare variable for battery State of Health (in percent) */
float soh = INITIAL_SOH;
/* Declare structure for Kalman filter to smooth SOC estimates */
KalmanFilter soc_kf;
/* Declare structure for Kalman filter to smooth SOH estimates */
KalmanFilter soh_kf;

/* Declare static variable for next log slot in flash memory */
static uint32_t next_slot = 0;
/* Declare static buffer for a single log entry */
static uint8_t log_buffer[LOG_ENTRY_SIZE];
/* Declare static variable for cumulative charge (mAh) */
static float coulomb_count;
/* Declare static variable for initial battery capacity (mAh) */
static float initial_capacity;
/* Declare static variable for current battery capacity (mAh) */
static float actual_capacity;
/* Declare static buffer for sending SSP frames */
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
/* Declare static buffer for receiving SSP frames */
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];
/* Declare static variable for current BMS mode */
static BMS_ModeTypeDef bms_mode = MODE_DISCHARGING;
/* Declare static flag to enable/disable charging */
static uint8_t charge_enabled __attribute__((used)) = 0; /* Flag to enable/disable charging */
/* Declare static flag to enable/disable discharging */
static uint8_t discharge_enabled __attribute__((used)) = 1; /* Flag to enable/disable discharging */
/* Declare static flag for immediate charging need */
static uint8_t charge_immediately = 0;
/* Declare static flag for BMS operational status */
static uint8_t bms_online = 0;
/* Declare static variable for error flags */
static uint32_t error_flags = 0;
/* Declare static variable for charge cycle count */
static uint32_t charge_cycle_count = 0;
/* Declare static variable for total charging time (seconds) */
static uint32_t total_charge_time = 0;
/* Declare static variable for total discharging time (seconds) */
static uint32_t total_discharge_time = 0;
/* Declare static variable for total operating time (seconds) */
static uint32_t total_operating_time = 0;
/* Declare static flag for charging cycle start */
static uint8_t charging_started = 0;
/* Declare static variable for charge start time (ms) */
static uint32_t charge_start_time = 0;
/* Declare static flag for Constant Voltage mode */
static uint8_t in_cv_mode = 0;
/* Declare static flag for mission termination enable */
static uint8_t mission_termination_enabled = 0;
/* Declare static variable for balancing mask of first BQ76920 */
static uint8_t balancing_mask_1 = 0;
/* Declare static variable for balancing mask of second BQ76920 */
static uint8_t balancing_mask_2 = 0;
/* Declare static flag for cell balancing activity */
static uint8_t balancing_active = 0;
/* Declare static array for 16 power line states */
static uint8_t power_lines[16] = {0};
/* Declare static variable for LED blink start time (ms) */
static uint32_t startup_blink_start = 0;
/* Declare static variable for last LED toggle time (ms) */
static uint32_t last_blink_toggle = 0;
/* Define constant for LED blinking duration (5 seconds) */
#define STARTUP_BLINK_DURATION 5000
/* Define constant for LED toggle interval (0.2 seconds) */
#define BLINK_INTERVAL 200

/* Declare structure for battery configuration */
BatteryConfig battery_config = {
    .nominal_capacity = 4000.0f, /* Set nominal capacity to 4000 mAh */
    .ov_threshold = 4200, /* Set overvoltage threshold to 4.2V */
    .uv_threshold = 2800, /* Set undervoltage threshold to 2.8V */
    .occ_threshold = 5000, /* Set overcurrent charge threshold to 5000 mA */
    .ocd_threshold = 5000, /* Set overcurrent discharge threshold to 5000 mA */
    .overtemp_threshold = 60, /* Set max temperature to 60°C */
    .undertemp_threshold = -20, /* Set min temperature to -20°C */
    .soc_low_threshold = 20.0f, /* Set low SOC threshold to 20% */
    .max_charge_time = 3600, /* Set max charge time to 3600 seconds */
    .cv_threshold = 4200 /* Set Constant Voltage threshold to 4.2V */
};

/* Declare static flag for firmware update mode */
static uint8_t firmware_update_mode = 0;
/* Define constant for firmware update packet size (128 bytes) */
#define FIRMWARE_UPDATE_PACKET_SIZE 128
/* Define constant for firmware update timeout (10 seconds) */
#define FIRMWARE_UPDATE_TIMEOUT 10000

/* Declare function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
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

/* Define function to erase a flash memory page
 * Inputs:
 * - page: The flash page number (uint32_t) to erase, specifying which memory section to clear.
 * Returns: void, meaning it returns nothing; it performs the erase operation directly.
 * What it does: Erases a specified page in flash memory to prepare it for writing new data.
 */
void Flash_Erase(uint32_t page)
{
    FLASH_EraseInitTypeDef erase_init; /* Create structure for erase settings */
    uint32_t page_error; /* Create variable to store erase errors */
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES; /* Set erase type to pages */
    erase_init.Page = page; /* Set page number to erase */
    erase_init.NbPages = 1; /* Set number of pages to 1 */
    HAL_FLASH_Unlock(); /* Unlock flash memory for erasing */
    HAL_FLASHEx_Erase(&erase_init, &page_error); /* Perform page erase */
    HAL_FLASH_Lock(); /* Lock flash memory for safety */
}

/* Define function to log errors to flash with a timestamp
 * Inputs:
 * - format: A format string (const char *) for the error message, like printf.
 * - ...: Variable arguments for formatting, allowing flexible message creation.
 * Returns: void, meaning it returns nothing; it writes the log to flash.
 * What it does: Formats an error message with a timestamp and writes it to flash memory.
 */
void Log_Error(const char *format, ...)
{
    RTC_TimeTypeDef sTime = {0}; /* Create structure for RTC time */
    RTC_DateTypeDef sDate = {0}; /* Create structure for RTC date */
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); /* Read current time from RTC */
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Read current date from RTC */
    uint64_t timestamp = HAL_GetTick(); /* Get system tick count (ms) */
    char message_buffer[MESSAGE_SIZE]; /* Create buffer for formatted message */
    va_list args; /* Create variable for variable arguments */
    va_start(args, format); /* Start processing variable arguments */
    vsnprintf(message_buffer, MESSAGE_SIZE, format, args); /* Format message into buffer */
    va_end(args); /* Clean up variable arguments */
    memset(log_buffer, 0, LOG_ENTRY_SIZE); /* Clear log buffer */
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE); /* Copy timestamp to buffer */
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message_buffer, MESSAGE_SIZE - 1); /* Copy message to buffer */
    uint32_t address = LOG_START_ADDR + (next_slot * LOG_ENTRY_SIZE); /* Calculate flash address */
    HAL_FLASH_Unlock(); /* Unlock flash memory */
    for (uint8_t i = 0; i < LOG_ENTRY_SIZE; i += 8) { /* Loop to write 8-byte chunks */
        uint64_t data = *(uint64_t *)(log_buffer + i); /* Read 8 bytes from buffer */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data); /* Write to flash */
    }
    HAL_FLASH_Lock(); /* Lock flash memory */
    next_slot = (next_slot + 1) % NUM_LOG_ENTRIES; /* Update next slot (circular) */
    HAL_FLASH_Unlock(); /* Unlock flash again */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot); /* Save next slot */
    HAL_FLASH_Lock(); /* Lock flash memory */
}

/* Define function to read and send all logs over RS485
 * Inputs: None
 * Returns: void, meaning it returns nothing; it sends logs via UART.
 * What it does: Reads all log entries from flash and transmits them over USART1.
 */
void Log_Read_All(void)
{
    char buffer[128]; /* Create buffer for log message */
    for (uint32_t i = 0; i < NUM_LOG_ENTRIES; i++) { /* Loop through log entries */
        uint32_t address = LOG_START_ADDR + (i * LOG_ENTRY_SIZE); /* Calculate log address */
        uint64_t timestamp = *(uint64_t *)address; /* Read timestamp from flash */
        char *message = (char *)(address + TIMESTAMP_SIZE); /* Get message pointer */
        snprintf(buffer, sizeof(buffer), "Log %lu: Time=%llu, Msg=%s\r\n", i, timestamp, message); /* Format log message */
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY); /* Send via USART1 */
    }
}

/* Define function to initialize the logging system
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes logging variables.
 * What it does: Loads the next log slot from flash and resets it if invalid.
 */
void Log_Init(void)
{
    next_slot = *(uint32_t *)NEXT_SLOT_ADDR; /* Load next slot index */
    if (next_slot >= NUM_LOG_ENTRIES) { /* Check if index is invalid */
        Flash_Erase(FLASH_LOG_PAGE); /* Erase log page */
        next_slot = 0; /* Reset slot index */
        HAL_FLASH_Unlock(); /* Unlock flash memory */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot); /* Save slot index */
        HAL_FLASH_Lock(); /* Lock flash memory */
    }
}

/* Define function to update battery SOC and SOH
 * Inputs: None
 * Returns: void, meaning it returns nothing; it updates global SOC and SOH variables.
 * What it does: Calculates SOC using coulomb counting and smooths it with a Kalman filter;
 *               updates SOH when fully charged.
 */
void Update_SOC_SOH(void)
{
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0; /* Integrate current for charge */
    float soc_measured = (coulomb_count / battery_config.nominal_capacity) * 100.0; /* Calculate raw SOC (%) */
    soc = KalmanFilter_Update(&soc_kf, soc_measured); /* Smooth SOC with Kalman filter */
    if (soc > 100.0) soc = 100.0; /* Cap SOC at 100% */
    if (soc < 0.0) soc = 0.0; /* Cap SOC at 0% */
    if (soc >= 100.0) { /* Check if fully charged */
        actual_capacity = coulomb_count; /* Update actual capacity */
        soh = (actual_capacity / initial_capacity) * 100.0; /* Calculate SOH (%) */
        soh = KalmanFilter_Update(&soh_kf, soh); /* Smooth SOH with Kalman filter */
        if (soh > 100.0) soh = 100.0; /* Cap SOH at 100% */
        if (soh < 0.0) soh = 0.0; /* Cap SOH at 0% */
    }
}

/* Define function to send BMS status to OBC
 * Inputs: None
 * Returns: void, meaning it returns nothing; it sends telemetry via RS485.
 * What it does: Packs BMS telemetry into an SSP frame and sends it over USART2.
 */
void SSP_SendStatus(void)
{
    SSP_TelemetryTypeDef telemetry = {0}; /* Create telemetry structure */
    SSP_FrameTypeDef frame = {0}; /* Create SSP frame structure */
    uint16_t frame_len; /* Create variable for frame length */
    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2] + group_voltages_1[3]; /* Sum cell voltages */
    telemetry.charge_immediately = charge_immediately; /* Set charge flag */
    telemetry.bms_online = bms_online; /* Set online status */
    telemetry.error_flags = error_flags; /* Set error flags */
    telemetry.pack_voltage_1 = (uint16_t)pack_voltage; /* Set pack voltage (IC1) */
    telemetry.pack_voltage_2 = (uint16_t)pack_voltage; /* Set pack voltage (IC2) */
    telemetry.pack_current_1 = pack_current_1; /* Set current (IC1) */
    telemetry.pack_current_2 = pack_current_2; /* Set current (IC2) */
    telemetry.soc = (uint8_t)soc; /* Set SOC as 8-bit */
    telemetry.soh = (uint8_t)soh; /* Set SOH as 8-bit */
    telemetry.temp_1 = temperature_1; /* Set NTC-1 temperature */
    telemetry.temp_2 = temperature_2; /* Set NTC-2 temperature */
    telemetry.pcb_temp = pcb_temperature; /* Set PCB temperature */
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells */
        telemetry.group_voltages[i] = group_voltages_1[i]; /* Copy cell voltages */
    }
    telemetry.balancing_active = balancing_active; /* Set balancing flag */
    telemetry.balancing_mask_1 = balancing_mask_1; /* Set balancing mask (IC1) */
    telemetry.balancing_mask_2 = balancing_mask_2; /* Set balancing mask (IC2) */
    telemetry.charge_cycle_count = charge_cycle_count; /* Set cycle count */
    telemetry.total_charge_time = total_charge_time; /* Set charge time */
    telemetry.total_discharge_time = total_discharge_time; /* Set discharge time */
    telemetry.total_operating_time = total_operating_time; /* Set operating time */
    SSP_PackTelemetry(&telemetry, &frame); /* Pack telemetry into frame */
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len); /* Build frame */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET); /* Set DE pin to transmit */
    SSP_TransmitFrame(&huart2, ssp_tx_buffer, frame_len); /* Send frame */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); /* Set DE pin to receive */
}

/* Define function to process received SSP frames
 * Inputs:
 * - frame: Pointer to an SSP frame structure (SSP_FrameTypeDef *), containing received
 *          command and data from the OBC.
 * Returns: void, meaning it returns nothing; it processes commands and sends responses.
 * What it does: Processes OBC commands (e.g., ping, mode change) and sends ACK/NACK responses.
 */
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame)
{
    if (frame->dest != SSP_ADDR_EPS && frame->dest != SSP_ADDR_BROADCAST && frame->dest != SSP_ADDR_MULTICAST) { /* Check if frame is for EPS */
        return; /* Exit if not addressed to EPS */
    }
    if (frame->cmd_id & SSP_FRAME_TYPE_REPLY) { /* Check if frame is a reply */
        return; /* Exit if reply frame */
    }
    if (frame->cmd_id & SSP_CMD_TYPE_TIMETAG) { /* Check if frame is time-tagged */
        return; /* Exit if time-tagged */
    }
    char log_msg[MESSAGE_SIZE]; /* Create buffer for logging */
    snprintf(log_msg, sizeof(log_msg), "Received CMD: ID=0x%02X, SRC=0x%02X, LEN=%d", frame->cmd_id, frame->src, frame->data_len); /* Format log message */
    Log_Error(log_msg); /* Log received command */
    SSP_FrameTypeDef response = {0}; /* Create response frame structure */
    response.dest = frame->src; /* Set response destination to sender */
    response.src = SSP_ADDR_EPS; /* Set response source to EPS */
    response.data_len = 1; /* Set response data length to 1 */
    response.data[0] = frame->cmd_id; /* Echo command ID */
    switch (frame->cmd_id & 0x3F) { /* Process command ID (mask flags) */
        case SSP_CMD_PING: /* Handle ping command */
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
            break;
        case SSP_CMD_SON: /* Handle switch-on command */
            if (frame->data_len == 1) { /* Check data length */
                uint8_t pwrl_id = frame->data[0]; /* Get power line ID */
                if (pwrl_id <= 15) { /* Validate ID */
                    power_lines[pwrl_id] = 1; /* Turn on power line */
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                    snprintf(log_msg, sizeof(log_msg), "SON command: PWRL%d ON", pwrl_id); /* Format log */
                    Log_Error(log_msg); /* Log action */
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_SOF: /* Handle switch-off command */
            if (frame->data_len == 1) { /* Check data length */
                uint8_t pwrl_id = frame->data[0]; /* Get power line ID */
                if (pwrl_id <= 15) { /* Validate ID */
                    power_lines[pwrl_id] = 0; /* Turn off power line */
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                    snprintf(log_msg, sizeof(log_msg), "SOF command: PWRL%d OFF", pwrl_id); /* Format log */
                    Log_Error(log_msg); /* Log action */
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_KEN: /* Handle mission termination */
            if (frame->data_len == 0) { /* Check no data */
                if (mission_termination_enabled) { /* Check termination enabled */
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                    Log_Error("KEN command executed: Mission termination enabled"); /* Log action */
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
                    Log_Error("KEN command rejected: Mission termination not enabled"); /* Log rejection */
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_KDIS: /* Handle battery reconnect */
            if (frame->data_len == 0) { /* Check no data */
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                Log_Error("KDIS command executed: Batteries reconnected"); /* Log action */
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_SM: /* Handle set mode command */
            if (frame->data_len >= 1) { /* Check data length */
                bms_mode = frame->data[0]; /* Set new mode */
                Update_BMS_Mode(); /* Apply mode change */
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                Log_Error("Mode changed by OBC"); /* Log action */
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_GM: /* Handle get mode command */
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
            response.data_len = 1; /* Set data length */
            response.data[0] = bms_mode; /* Set current mode */
            break;
        case SSP_CMD_GOSTM: /* Handle get status command */
            SSP_SendStatus(); /* Send telemetry */
            return; /* Exit without response */
        case SSP_CMD_SFP: /* Handle set flight parameter */
            if (frame->data_len >= 2) { /* Check data length */
                uint8_t param_id = frame->data[0]; /* Get parameter ID */
                uint8_t param_value = frame->data[1]; /* Get parameter value */
                if (param_id == 0x01) { /* Check mission termination parameter */
                    mission_termination_enabled = param_value; /* Set termination flag */
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                    Log_Error("Mission termination enable set"); /* Log action */
                }
                else if (param_id == 0x02) { /* Check firmware update request */
                    if (param_value == 1) { /* Check update requested */
                        HAL_FLASH_Unlock(); /* Unlock flash */
                        Flash_Erase(FLASH_LOG_PAGE); /* Erase log page */
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xDEADBEEF); /* Set update flag */
                        HAL_FLASH_Lock(); /* Lock flash */
                        Log_Error("Firmware update requested, rebooting..."); /* Log action */
                        HAL_Delay(100); /* Wait 100 ms */
                        HAL_NVIC_SystemReset(); /* Reset microcontroller */
                    }
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK response */
                }
                else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            }
            break;
        case SSP_CMD_GSTLM: /* Handle get short telemetry */
        case SSP_CMD_GOTLM: /* Handle get operational telemetry */
            SSP_SendStatus(); /* Send telemetry */
            return; /* Exit without response */
        default: /* Handle unknown command */
            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK response */
            break;
    }
    uint16_t frame_len; /* Create variable for response frame length */
    SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len); /* Build response frame */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET); /* Set DE pin to transmit */
    SSP_TransmitFrame(&huart2, ssp_tx_buffer, frame_len); /* Send response */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); /* Set DE pin to receive */
}


/* Define function to read the STM32’s internal temperature sensor
 * Inputs: None
 * Returns: int16_t, the temperature in degrees Celsius, calculated from the ADC reading
 *          of the STM32’s internal temperature sensor. The value is typically between
 *          -40°C and 125°C, representing the PCB temperature.
 * What it does: Uses ADC1 to read the STM32’s internal temperature sensor, converts the
 *               raw ADC value to degrees Celsius, and returns it for use in fault detection
 *               (e.g., overtemperature checks in main.c).
 */
int16_t Read_Internal_Temperature(void) {
    HAL_ADC_Start(&hadc1); /* Start ADC1 to begin temperature measurement */
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); /* Wait until ADC conversion is complete */
    uint32_t raw = HAL_ADC_GetValue(&hadc1); /* Get the raw ADC value (0 to 4095) */
    HAL_ADC_Stop(&hadc1); /* Stop ADC1 to save power */
    int32_t temp = ((raw * 3300 / 4096) - 760) * 100 / 250 + 25; /* Convert raw value to °C using STM32 formula */
    return (int16_t)temp; /* Cast result to 16-bit integer and return */
}

/* Define function to update the BMS operating mode
 * Inputs: None
 * Returns: void, meaning it returns nothing; it modifies global variables (bms_mode,
 *          charge_enabled, discharge_enabled, error_flags) and applies settings to
 *          BQ76920 ICs.
 * What it does: Checks for faults (e.g., overvoltage, overtemperature) and sets the BMS
 *               mode (fault, charging, discharging, sleep) based on error flags, SOC,
 *               and current. It adjusts charging/discharging states and logs actions for
 *               diagnostics, sent to the OBC via RS485.
 */
void Update_BMS_Mode(void) {
    static uint32_t fault_start_time = 0; /* Create static variable to track when a fault starts (in ms) */
    static uint8_t in_fault_mode = 0; /* Create static flag to indicate if BMS is in fault mode (1 = yes, 0 = no) */
    static uint8_t recovery_attempts = 0; /* Create static counter for recovery attempts */
    static const uint8_t MAX_RECOVERY_ATTEMPTS = 3; /* Define constant for max recovery attempts (3) */
    static const uint32_t FAULT_TIMEOUT = 30000; /* Define constant for fault timeout (30 seconds) */
    static const uint32_t TEMP_FAULT_TIMEOUT = 60000; /* Define constant for temperature fault timeout (60 seconds) */
    static const uint32_t COOLDOWN_PERIOD = 10000; /* Define constant for cooldown period (10 seconds) */
    static const uint32_t RECOVERY_DELAY = 5000; /* Define constant for recovery delay (5 seconds) */
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OCC | ERROR_OCD | ERROR_SCD | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY | ERROR_DEVICE_XREADY | ERROR_OVRD_ALERT)) { /* Check if any fault flags are set */
        if (!in_fault_mode) { /* Check if not already in fault mode */
            fault_start_time = HAL_GetTick(); /* Record current system time (ms) */
            in_fault_mode = 1; /* Set flag to indicate fault mode */
        }
        bms_mode = MODE_FAULT; /* Set BMS mode to fault */
        if (error_flags & ERROR_OVERVOLTAGE) { /* Check for overvoltage fault */
            charge_enabled = 0; /* Disable charging to protect battery */
            discharge_enabled = 1; /* Enable discharging to reduce voltage */
            Log_Error("Protective action: Disabled charging due to overvoltage"); /* Log action for diagnostics */
            uint8_t all_below_threshold = 1; /* Create flag to check if all cells are safe */
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through all 4 cells */
                if (group_voltages_1[i] > battery_config.ov_threshold || group_voltages_2[i] > battery_config.ov_threshold) { /* Check if any cell exceeds threshold */
                    all_below_threshold = 0; /* Clear flag if any cell is over threshold */
                    break; /* Exit loop early */
                }
            }
            if (all_below_threshold) { /* Check if all cells are below threshold */
                Log_Error("Overvoltage fault cleared"); /* Log fault clearance */
                error_flags &= ~ERROR_OVERVOLTAGE; /* Clear overvoltage flag */
                in_fault_mode = 0; /* Exit fault mode */
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                Log_Error("Overvoltage fault persists, triggering system reset"); /* Log persistent fault */
                HAL_NVIC_SystemReset(); /* Reset microcontroller to recover */
            }
        }
        else if (error_flags & ERROR_UNDERVOLTAGE) { /* Check for undervoltage fault */
            charge_enabled = 1; /* Enable charging to increase voltage */
            discharge_enabled = 0; /* Disable discharging to prevent damage */
            Log_Error("Protective action: Disabled discharging due to undervoltage"); /* Log action */
            uint8_t all_above_threshold = 1; /* Create flag to check if all cells are safe */
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through all cells */
                if (group_voltages_1[i] < battery_config.uv_threshold || group_voltages_2[i] < battery_config.uv_threshold) { /* Check if any cell is below threshold */
                    all_above_threshold = 0; /* Clear flag if any cell is below threshold */
                    break; /* Exit loop early */
                }
            }
            if (all_above_threshold) { /* Check if all cells are above threshold */
                Log_Error("Undervoltage fault cleared"); /* Log fault clearance */
                error_flags &= ~ERROR_UNDERVOLTAGE; /* Clear undervoltage flag */
                in_fault_mode = 0; /* Exit fault mode */
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                Log_Error("Undervoltage fault persists, triggering system reset"); /* Log persistent fault */
                HAL_NVIC_SystemReset(); /* Reset microcontroller */
            }
        }
        else if (error_flags & ERROR_OCC) { /* Check for overcurrent charging fault */
            charge_enabled = 0; /* Disable charging to protect battery */
            discharge_enabled = 1; /* Enable discharging */
            Log_Error("Protective action: Disabled charging due to overcurrent charge"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD) { /* Wait for 10-second cooldown */
                int16_t total_current = (pack_current_1 + pack_current_2) / 2; /* Calculate average current */
                if (total_current >= 0) { /* Check if current is safe (non-negative) */
                    Log_Error("Overcurrent charge fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_OCC; /* Clear overcurrent charge flag */
                    in_fault_mode = 0; /* Exit fault mode */
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                    Log_Error("Overcurrent charge fault persists, triggering system reset"); /* Log persistent fault */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        else if (error_flags & ERROR_OCD) { /* Check for overcurrent discharging fault */
            charge_enabled = 1; /* Enable charging */
            discharge_enabled = 0; /* Disable discharging to protect battery */
            Log_Error("Protective action: Disabled discharging due to overcurrent discharge"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD) { /* Wait for 10-second cooldown */
                int16_t total_current = (pack_current_1 + pack_current_2) / 2; /* Calculate average current */
                if (total_current <= 0) { /* Check if current is safe (non-positive) */
                    Log_Error("Overcurrent discharge fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_OCD; /* Clear overcurrent discharge flag */
                    in_fault_mode = 0; /* Exit fault mode */
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                    Log_Error("Overcurrent discharge fault persists, triggering system reset"); /* Log persistent fault */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        else if (error_flags & ERROR_SCD) { /* Check for short-circuit discharge fault */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging to protect system */
            Log_Error("Protective action: Disabled charging and discharging due to short-circuit discharge"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Wait for 30-second timeout */
                uint8_t status1, status2; /* Create variables to store BQ76920 status */
                uint8_t scd_cleared = 1; /* Create flag to track if short-circuit is cleared */
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 3))) { /* Check short-circuit bit on first IC */
                    scd_cleared = 0; /* Clear flag if fault is active */
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 3))) { /* Check short-circuit bit on second IC */
                    scd_cleared = 0; /* Clear flag if fault is active */
                }
                if (scd_cleared) { /* Check if fault is cleared */
                    Log_Error("Short-circuit discharge fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_SCD; /* Clear short-circuit flag */
                    in_fault_mode = 0; /* Exit fault mode */
                }
                else { /* If fault persists */
                    Log_Error("Short-circuit discharge fault persists, triggering system reset"); /* Log persistent fault */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        else if (error_flags & ERROR_OVERTEMP) { /* Check for overtemperature fault */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging to protect battery */
            Log_Error("Protective action: Disabled charging and discharging due to overtemperature"); /* Log action */
            if (temperature_1 < (battery_config.overtemp_threshold - 10) && temperature_2 < (battery_config.overtemp_threshold - 10) && pcb_temperature < (battery_config.overtemp_threshold - 10)) { /* Check temperatures with 10°C hysteresis */
                Log_Error("Overtemperature fault cleared"); /* Log fault clearance */
                error_flags &= ~ERROR_OVERTEMP; /* Clear overtemperature flag */
                in_fault_mode = 0; /* Exit fault mode */
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT) { /* Check if fault persists for 60 seconds */
                Log_Error("Overtemperature fault persists, triggering system reset"); /* Log persistent fault */
                HAL_NVIC_SystemReset(); /* Reset microcontroller */
            }
        }
        else if (error_flags & ERROR_UNDERTEMP) { /* Check for undertemperature fault */
            charge_enabled = 0; /* Disable charging to prevent damage */
            discharge_enabled = 1; /* Enable discharging */
            Log_Error("Protective action: Disabled charging due to undertemperature"); /* Log action */
            if (temperature_1 > (battery_config.undertemp_threshold + 10) && temperature_2 > (battery_config.undertemp_threshold + 10)) { /* Check temperatures with 10°C hysteresis */
                Log_Error("Undertemperature fault cleared"); /* Log fault clearance */
                error_flags &= ~ERROR_UNDERTEMP; /* Clear undertemperature flag */
                in_fault_mode = 0; /* Exit fault mode */
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT) { /* Check if fault persists for 60 seconds */
                Log_Error("Undertemperature fault persists, triggering system reset"); /* Log persistent fault */
                HAL_NVIC_SystemReset(); /* Reset microcontroller */
            }
        }
        else if (error_flags & ERROR_DISCREPANCY) { /* Check for redundancy discrepancy between ICs */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging to protect system */
            Log_Error("Protective action: Disabled charging and discharging due Wise to redundancy discrepancy"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY) { /* Wait for 5-second recovery delay */
                Log_Error("Attempting to reinitialize BQ76920 ICs to resolve discrepancy"); /* Log recovery attempt */
                if (BQ76920_Init(&hi2c1) != HAL_OK) { /* Try reinitializing first IC */
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)"); /* Log failure */
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK) { /* Try reinitializing second IC */
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)"); /* Log failure */
                }
                uint8_t discrepancy_flag = 0; /* Create flag for discrepancy check */
                BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag); /* Check redundancy */
                if (!discrepancy_flag) { /* Check if discrepancy is resolved */
                    Log_Error("Redundancy discrepancy fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_DISCREPANCY; /* Clear discrepancy flag */
                    in_fault_mode = 0; /* Exit fault mode */
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                    Log_Error("Redundancy discrepancy fault persists, triggering system reset"); /* Log persistent fault */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        else if (error_flags & ERROR_DEVICE_XREADY) { /* Check for BQ76920 initialization fault */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging */
            Log_Error("Protective action: Disabled charging and discharging due to DEVICE_XREADY"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY) { /* Wait for 5-second recovery delay */
                recovery_attempts++; /* Increment recovery attempt counter */
                Log_Error("Attempting to reinitialize BQ76920 ICs (attempt %d)", recovery_attempts); /* Log attempt */
                uint8_t init_success = 1; /* Create flag for initialization success */
                if (BQ76920_Init(&hi2c1) != HAL_OK) { /* Reinitialize first IC */
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)"); /* Log failure */
                    init_success = 0; /* Clear success flag */
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK) { /* Reinitialize second IC */
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)"); /* Log failure */
                    init_success = 0; /* Clear success flag */
                }
                if (init_success) { /* Check if initialization succeeded */
                    Log_Error("DEVICE_XREADY fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_DEVICE_XREADY; /* Clear fault flag */
                    in_fault_mode = 0; /* Exit fault mode */
                    recovery_attempts = 0; /* Reset attempt counter */
                }
                else if (recovery_attempts >= MAX_RECOVERY_ATTEMPTS) { /* Check if max attempts reached */
                    Log_Error("Failed to recover from DEVICE_XREADY after %d attempts, triggering system reset", MAX_RECOVERY_ATTEMPTS); /* Log failure */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        else if (error_flags & ERROR_OVRD_ALERT) { /* Check for general BQ76920 alert */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging */
            Log_Error("Protective action: Disabled charging and discharging due to OVRD_ALERT"); /* Log action */
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD) { /* Wait for 10-second cooldown */
                uint8_t status1, status2; /* Create variables for BQ76920 status */
                uint8_t alert_cleared = 1; /* Create flag for alert clearance */
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 6))) { /* Check alert bit on first IC */
                    alert_cleared = 0; /* Clear flag if alert is active */
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 6))) { /* Check alert bit on second IC */
                    alert_cleared = 0; /* Clear flag if alert is active */
                }
                if (alert_cleared) { /* Check if alert is cleared */
                    Log_Error("OVRD_ALERT fault cleared"); /* Log fault clearance */
                    error_flags &= ~ERROR_OVRD_ALERT; /* Clear alert flag */
                    in_fault_mode = 0; /* Exit fault mode */
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT) { /* Check if fault persists for 30 seconds */
                    Log_Error("OVRD_ALERT fault persists, triggering system reset"); /* Log persistent fault */
                    HAL_NVIC_SystemReset(); /* Reset microcontroller */
                }
            }
        }
        BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled); /* Apply charge/discharge settings to first IC */
        BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled); /* Apply charge/discharge settings to second IC */
        return; /* Exit function if in fault mode */
    }
    in_fault_mode = 0; /* Reset fault mode flag */
    recovery_attempts = 0; /* Reset recovery attempt counter */
    charge_immediately = (soc < battery_config.soc_low_threshold) ? 1 : 0; /* Set charge flag if SOC is below 20% */
    int16_t total_current = (pack_current_1 + pack_current_2) / 2; /* Calculate average current from both ICs */
    if (total_current < 0) { /* Check if current is negative (charging) */
        bms_mode = MODE_CHARGING; /* Set BMS to charging mode */
        charge_enabled = 1; /* Enable charging */
        discharge_enabled = 0; /* Disable discharging */
    } else if (total_current > 0) { /* Check if current is positive (discharging) */
        bms_mode = MODE_DISCHARGING; /* Set BMS to discharging mode */
        charge_enabled = 0; /* Disable charging */
        discharge_enabled = 1; /* Enable discharging */
    } else { /* If no current */
        if (soc < battery_config.soc_low_threshold) { /* Check if SOC is below 20% */
            bms_mode = MODE_CHARGING; /* Set BMS to charging mode */
            charge_enabled = 1; /* Enable charging */
            discharge_enabled = 0; /* Disable discharging */
        } else { /* If SOC is sufficient */
            bms_mode = MODE_SLEEP; /* Set BMS to sleep mode */
            charge_enabled = 0; /* Disable charging */
            discharge_enabled = 0; /* Disable discharging */
        }
    }
    BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled); /* Apply charge/discharge settings to first IC */
    BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled); /* Apply charge/discharge settings to second IC */
}
/* Define function to control battery charging
 * Inputs: None
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if charging is managed
 *          successfully, or failure (HAL_ERROR) if temperature readings fail or
 *          overtemperature is detected.
 * What it does: Manages charging based on temperature and voltage, switching between
 *               Constant Current and Constant Voltage modes.
 */
HAL_StatusTypeDef ChargeBattery(void)
{
    int16_t temperature_1, temperature_2; /* Create variables for temperatures */
    HAL_StatusTypeDef status; /* Create variable for status */
    status = Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2); /* Read NTC temperatures */
    if (status != HAL_OK || temperature_1 == INT16_MIN || temperature_2 == INT16_MIN) { /* Check for errors */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); /* Disable charging */
        in_cv_mode = 0; /* Reset CV mode */
        charge_start_time = 0; /* Reset charge timer */
        return HAL_ERROR; /* Return error */
    }
    int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2; /* Select higher temperature */
    if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold) { /* Check overtemperature */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); /* Disable charging */
        in_cv_mode = 0; /* Reset CV mode */
        charge_start_time = 0; /* Reset charge timer */
        return HAL_ERROR; /* Return error */
    }
    if (!in_cv_mode) { /* Check if in CC mode */
        if (charge_start_time == 0) { /* Check if charging started */
            charge_start_time = HAL_GetTick(); /* Record start time */
        }
        int16_t max_voltage = 0; /* Create variable for max cell voltage */
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells */
            if (group_voltages_1[i] > max_voltage) max_voltage = group_voltages_1[i]; /* Update max */
            if (group_voltages_2[i] > max_voltage) max_voltage = group_voltages_2[i]; /* Update max */
        }
        if (max_voltage > battery_config.cv_threshold) { /* Check CV threshold */
            in_cv_mode = 1; /* Switch to CV mode */
        }
    }
    else { /* In CV mode */
        uint32_t charge_duration = (HAL_GetTick() - charge_start_time) / 1000; /* Calculate charge duration */
        if (charge_duration > battery_config.max_charge_time) { /* Check max charge time */
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); /* Disable charging */
            in_cv_mode = 0; /* Reset CV mode */
            charge_start_time = 0; /* Reset charge timer */
        }
    }
    return HAL_OK; /* Return success */
}

/* Define function to check for firmware update mode
 * Inputs: None
 * Returns: void, meaning it returns nothing; it sets firmware_update_mode and performs actions.
 * What it does: Checks a flash flag to enter firmware update mode or normal operation.
 */
void Bootloader_Check(void)
{
    uint32_t firmware_update_flag = *(volatile uint32_t *)FIRMWARE_UPDATE_FLAG_ADDR; /* Read update flag */
    if (firmware_update_flag == 0xDEADBEEF) { /* Check for update mode */
        firmware_update_mode = 1; /* Set update mode */
        Log_Error("Entering firmware update mode"); /* Log action */
        HAL_FLASH_Unlock(); /* Unlock flash */
        Flash_Erase(FLASH_LOG_PAGE); /* Erase log page */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xFFFFFFFF); /* Clear flag */
        HAL_FLASH_Lock(); /* Lock flash */
    }
    else { /* Normal mode */
        firmware_update_mode = 0; /* Clear update mode */
        Log_Error("Booting to application"); /* Log action */
    }
}

/* Define function to handle firmware updates
 * Inputs: None
 * Returns: void, meaning it returns nothing; it receives and writes firmware to flash.
 * What it does: Receives firmware packets over RS485, writes them to flash, and verifies CRC.
 */
void Bootloader_FirmwareUpdate(void)
{
    SSP_FrameTypeDef received_frame = {0}; /* Create structure for received frame */
    uint32_t last_packet_time = HAL_GetTick(); /* Record current time */
    uint32_t current_address = APP_START_ADDR; /* Set start address for firmware */
    uint32_t total_bytes_received = 0; /* Initialize received bytes counter */
    uint32_t expected_firmware_size = 0; /* Initialize expected size */
    uint8_t firmware_buffer[FIRMWARE_UPDATE_PACKET_SIZE]; /* Create buffer for firmware */
    uint16_t calculated_crc = 0xFFFF; /* Initialize CRC */
    Log_Error("Waiting for firmware update packets..."); /* Log start */
    HAL_FLASH_Unlock(); /* Unlock flash */
    for (uint32_t addr = APP_START_ADDR; addr < APP_END_ADDR; addr += FLASH_PAGE_SIZE) { /* Loop through flash */
        Flash_Erase((addr - FLASH_BASE) / FLASH_PAGE_SIZE); /* Erase page */
    }
    HAL_FLASH_Lock(); /* Lock flash */
    while (1) { /* Infinite loop */
        if (HAL_GetTick() - last_packet_time > FIRMWARE_UPDATE_TIMEOUT) { /* Check timeout */
            Log_Error("Firmware update timeout, rebooting..."); /* Log timeout */
            HAL_NVIC_SystemReset(); /* Reset microcontroller */
        }
        if (SSP_ReceiveFrame(&huart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK) { /* Check for frame */
            last_packet_time = HAL_GetTick(); /* Update packet time */
            if (received_frame.dest != SSP_ADDR_EPS && received_frame.dest != SSP_ADDR_BROADCAST) { /* Check destination */
                continue; /* Skip if not for EPS */
            }
            if (received_frame.cmd_id & SSP_FRAME_TYPE_REPLY) { /* Check if reply */
                continue; /* Skip reply */
            }
            SSP_FrameTypeDef response = {0}; /* Create response frame */
            response.dest = received_frame.src; /* Set response destination */
            response.src = SSP_ADDR_EPS; /* Set response source */
            response.data_len = 1; /* Set response data length */
            response.data[0] = received_frame.cmd_id; /* Echo command ID */
            switch (received_frame.cmd_id & 0x3F) { /* Process command */
                case SSP_CMD_FIRMWARE_UPDATE: /* Handle firmware update */
                    if (received_frame.data_len < 4 && total_bytes_received == 0) { /* Check first packet */
                        response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK */
                        break;
                    }
                    if (total_bytes_received == 0) { /* Check first packet */
                        expected_firmware_size = (received_frame.data[0] << 24) | (received_frame.data[1] << 16) | (received_frame.data[2] << 8) | received_frame.data[3]; /* Get size */
                        Log_Error("Firmware update started, expected size: %lu bytes", expected_firmware_size); /* Log start */
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK */
                    } else {
                        if (received_frame.data_len > FIRMWARE_UPDATE_PACKET_SIZE) { /* Check packet size */
                            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK */
                            break;
                        }
                        memcpy(firmware_buffer, received_frame.data, received_frame.data_len); /* Copy packet data */
                        HAL_FLASH_Unlock(); /* Unlock flash */
                        for (uint32_t i = 0; i < received_frame.data_len; i += 8) { /* Loop through data */
                            uint64_t data = *(uint64_t *)(firmware_buffer + i); /* Get 8 bytes */
                            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_address + i, data); /* Write to flash */
                        }
                        HAL_FLASH_Lock(); /* Lock flash */
                        calculated_crc = CalculateCRC16(firmware_buffer, received_frame.data_len); /* Update CRC */
                        total_bytes_received += received_frame.data_len; /* Update byte count */
                        current_address += received_frame.data_len; /* Update address */
                        Log_Error("Received %lu/%lu bytes", total_bytes_received, expected_firmware_size); /* Log progress */
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY; /* Set ACK */
                        if (total_bytes_received >= expected_firmware_size) { /* Check if complete */
                            uint16_t received_crc = (firmware_buffer[received_frame.data_len - 2] << 8) | firmware_buffer[received_frame.data_len - 1]; /* Get CRC */
                            if (calculated_crc == received_crc) { /* Verify CRC */
                                HAL_FLASH_Unlock(); /* Unlock flash */
                                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_VALIDITY_FLAG_ADDR, 0xA5A5A5A5); /* Set validity flag */
                                uint64_t crc_data = calculated_crc; /* Store CRC */
                                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_END_ADDR - 8, crc_data); /* Write CRC */
                                HAL_FLASH_Lock(); /* Lock flash */
                                Log_Error("Firmware update completed successfully, rebooting..."); /* Log success */
                                HAL_Delay(100); /* Wait 100 ms */
                                HAL_NVIC_SystemReset(); /* Reset microcontroller */
                            } else {
                                Log_Error("Firmware CRC16 mismatch, rebooting without setting validity flag..."); /* Log failure */
                                HAL_Delay(100); /* Wait 100 ms */
                                HAL_NVIC_SystemReset(); /* Reset microcontroller */
                            }
                        }
                    }
                    break;
                default: /* Handle unknown command */
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY; /* Set NACK */
                    break;
            }
            uint16_t frame_len; /* Create variable for response frame length */
            SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len); /* Build response */
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET); /* Set DE pin to transmit */
            SSP_TransmitFrame(&huart2, ssp_tx_buffer, frame_len); /* Send response */
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); /* Set DE pin to receive */
        }
    }
}

/* Define function to validate application code
 * Inputs:
 * - start_addr: Starting address (uint32_t) of the application in flash to validate.
 * Returns: uint8_t, returning 1 if the application is valid (CRC and flag match), 0 otherwise.
 * What it does: Checks the application’s CRC and validity flag to ensure integrity.
 */
static uint8_t IsApplicationValid(uint32_t start_addr)
{
    uint16_t crc = CalculateCRC16((uint8_t *)start_addr, APP_END_ADDR - start_addr - 8); /* Calculate CRC */
    uint16_t stored_crc = *(uint16_t *)(APP_END_ADDR - 8); /* Read stored CRC */
    return (crc == stored_crc) && (*(uint32_t *)APP_VALIDITY_FLAG_ADDR == 0xA5A5A5A5); /* Check CRC and flag */
}

/* Define function to jump to application code
 * Inputs: None
 * Returns: void, meaning it returns nothing; it jumps to application code or halts.
 * What it does: Attempts to jump to the main or backup application, halting if both invalid.
 */
void JumpToApplication(void)
{
    if (IsApplicationValid(APP_START_ADDR)) { /* Check main application validity */
        uint32_t app_jump_address = *(volatile uint32_t *)(APP_START_ADDR + 4); /* Read reset handler */
        void (*app_reset_handler)(void) = (void (*)(void))app_jump_address; /* Cast to function pointer */
        __set_MSP(*(volatile uint32_t *)APP_START_ADDR); /* Set stack pointer */
        app_reset_handler(); /* Jump to main application */
    } else {
        Log_Error("Main application invalid, falling back to backup..."); /* Log failure */
        if (IsApplicationValid(BACKUP_START_ADDR)) { /* Check backup validity */
            uint32_t backup_jump_address = *(volatile uint32_t *)(BACKUP_START_ADDR + 4); /* Read reset handler */
            void (*backup_reset_handler)(void) = (void (*)(void))backup_jump_address; /* Cast to function pointer */
            __set_MSP(*(volatile uint32_t *)BACKUP_START_ADDR); /* Set stack pointer */
            backup_reset_handler(); /* Jump to backup application */
        } else {
            Log_Error("Backup application also invalid, halting..."); /* Log failure */
            while (1) { /* Infinite loop */
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); /* Toggle LED */
                HAL_Delay(500); /* Wait 500 ms */
            }
        }
    }
}

/* Define main function, program entry point
 * Inputs: None
 * Returns: int, typically 0 for success, but unused here as it runs indefinitely.
 * What it does: Initializes hardware, checks firmware mode, and runs the main BMS loop.
 */
int main(void)
{
    HAL_Init(); /* Initialize STM32 HAL library */
    SystemClock_Config(); /* Configure system clock */
    MX_GPIO_Init(); /* Initialize GPIO pins */
    MX_I2C1_Init(); /* Initialize I2C1 */
    MX_I2C2_Init(); /* Initialize I2C2 */
    MX_I2C3_Init(); /* Initialize I2C3 */
    MX_RTC_Init(); /* Initialize RTC */
    MX_USART1_UART_Init(); /* Initialize USART1 */
    MX_USART2_UART_Init(); /* Initialize USART2 */
    MX_ADC1_Init(); /* Initialize ADC1 */
    RTC_TimeTypeDef sTime = {0}; /* Create structure for RTC time */
    RTC_DateTypeDef sDate = {0}; /* Create structure for RTC date */
    sTime.Hours = 12; /* Set initial hour to 12 */
    sTime.Minutes = 0; /* Set initial minutes to 0 */
    sTime.Seconds = 0; /* Set initial seconds to 0 */
    sDate.Year = 25; /* Set year to 2025 */
    sDate.Month = 3; /* Set month to March */
    sDate.Date = 28; /* Set date to 28 */
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY; /* Set weekday to Friday */
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); /* Set RTC time */
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Set RTC date */
    Log_Init(); /* Initialize logging system */
    Bootloader_Check(); /* Check for firmware update mode */
    if (firmware_update_mode) { /* Check update mode */
        Bootloader_FirmwareUpdate(); /* Handle firmware update */
        HAL_NVIC_SystemReset(); /* Reset microcontroller */
    }
    JumpToApplication(); /* Attempt to jump to application */
    Log_Error("Failed to jump to application, entering normal operation"); /* Log failure */
    if (BQ76920_Init(&hi2c1) != HAL_OK) { /* Initialize first BQ76920 */
        Log_Error("BQ76920 (I2C1) initialization failed"); /* Log error */
        Error_Handler(); /* Call error handler */
    }
    if (BQ76920_ConfigureProtection(&hi2c1) != HAL_OK) { /* Configure protection for first IC */
        Log_Error("BQ76920 (I2C1) protection configuration failed"); /* Log error */
        Error_Handler(); /* Call error handler */
    }
    if (BQ76920_Init(&hi2c2) != HAL_OK) { /* Initialize second BQ76920 */
        Log_Error("BQ76920 (I2C2) initialization failed"); /* Log error */
        Error_Handler(); /* Call error handler */
    }
    if (BQ76920_ConfigureProtection(&hi2c2) != HAL_OK) { /* Configure protection for second IC */
        Log_Error("BQ76920 (I2C2) protection configuration failed"); /* Log error */
        Error_Handler(); /* Call error handler */
    }
    KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0); /* Initialize SOC Kalman filter */
    KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0); /* Initialize SOH Kalman filter */
    PID_Init(); /* Initialize PID controller */
    bms_online = 1; /* Set BMS online */
    startup_blink_start = HAL_GetTick(); /* Record blink start time */
    last_blink_toggle = HAL_GetTick(); /* Record last toggle time */
    initial_capacity = battery_config.nominal_capacity; /* Set initial capacity */
    actual_capacity = battery_config.nominal_capacity; /* Set actual capacity */
    coulomb_count = (INITIAL_SOC / 100.0) * battery_config.nominal_capacity; /* Set initial charge */
    uint32_t last_log_read = 0; /* Initialize last log read time */
    uint32_t last_status_send = 0; /* Initialize last status send time */
    uint32_t last_time_sync = 0; /* Initialize last time sync time */
    while (1) { /* Start infinite loop */
        if (HAL_GetTick() - startup_blink_start < STARTUP_BLINK_DURATION) { /* Check if within 5 seconds */
            if (HAL_GetTick() - last_blink_toggle >= BLINK_INTERVAL) { /* Check toggle interval */
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); /* Toggle LED */
                last_blink_toggle = HAL_GetTick(); /* Update toggle time */
            }
        }
        else { /* After 5 seconds */
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* Turn off LED */
        }
        if (BQ76920_ReadVoltages(&hi2c1, group_voltages_1, 0) != HAL_OK) { /* Read voltages (IC1) */
            Log_Error("Error reading BQ76920 (I2C1) group voltages"); /* Log error */
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells */
                group_voltages_1[i] = 0; /* Reset voltages */
            }
        }
        if (BQ76920_ReadCurrent(&hi2c1, &pack_current_1) != HAL_OK) { /* Read current (IC1) */
            Log_Error("Error reading BQ76920 (I2C1) current"); /* Log error */
        }
        if (BQ76920_ReadVoltages(&hi2c2, group_voltages_2, 0) != HAL_OK) { /* Read voltages (IC2) */
            Log_Error("Error reading BQ76920 (I2C2) group voltages"); /* Log error */
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells */
                group_voltages_2[i] = 0; /* Reset voltages */
            }
        }
        if (BQ76920_ReadCurrent(&hi2c2, &pack_current_2) != HAL_OK) { /* Read current (IC2) */
            Log_Error("Error reading BQ76920 (I2C2) current"); /* Log error */
        }
        if (Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2) != HAL_OK) { /* Read temperatures */
            Log_Error("Error reading temperatures (I2C1/I2C2)"); /* Log error */
            temperature_1 = INT16_MIN; /* Set error value for temperature 1 */
            temperature_2 = INT16_MIN; /* Set error value for temperature 2 */
        }
        uint8_t discrepancy_flag = 0; /* Create flag for redundancy check */
        BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag); /* Check redundancy */
        if (discrepancy_flag) { /* Check discrepancy */
            error_flags |= ERROR_DISCREPANCY; /* Set discrepancy flag */
            Log_Error("Redundancy discrepancy detected"); /* Log error */
        }
        pcb_temperature = Read_Internal_Temperature(); /* Read PCB temperature */
        BQ76920_CheckStatus(&hi2c1, &hi2c2, &error_flags); /* Check BQ76920 status */
        int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2; /* Select higher temperature */
        int16_t lowest_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2; /* Select lower temperature */
        if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold) { /* Check overtemperature */
            error_flags |= ERROR_OVERTEMP; /* Set overtemperature flag */
            Log_Error("Overtemperature detected"); /* Log error */
        }
        if (lowest_temp < battery_config.undertemp_threshold) { /* Check undertemperature */
            error_flags |= ERROR_UNDERTEMP; /* Set undertemperature flag */
            Log_Error("Undertemperature detected"); /* Log error */
        }
        total_operating_time = HAL_GetTick() / 1000; /* Update operating time */
        int16_t total_current = (pack_current_1 + pack_current_2) / 2; /* Calculate average current */
        if (total_current < 0) { /* Check if charging */
            total_charge_time += (uint32_t)LOOP_TIME; /* Increment charge time */
            if (soc < 20.0 && !charging_started) { /* Check charge cycle start */
                charging_started = 1; /* Set charging started */
            }
            if (soc >= 100.0 && charging_started) { /* Check charge cycle complete */
                charge_cycle_count++; /* Increment cycle count */
                charging_started = 0; /* Reset charging started */
            }
        } else if (total_current > 0) { /* Check if discharging */
            total_discharge_time += (uint32_t)LOOP_TIME; /* Increment discharge time */
        }
        if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK) { /* Balance cells (IC1) */
            Log_Error("Error balancing groups (I2C1)"); /* Log error */
        }
        if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK) { /* Balance cells (IC2) */
            Log_Error("Error balancing groups (I2C2)"); /* Log error */
        }
        balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0; /* Update balancing flag */
        PID_Control(lowest_temp); /* Control heaters */
        Update_SOC_SOH(); /* Update SOC and SOH */
        Update_BMS_Mode(); /* Update BMS mode */
        ChargeBattery(); /* Manage charging */
        char message[MESSAGE_SIZE]; /* Create buffer for status message */
        snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick()); /* Start message with timestamp */
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells */
            char group_data[20]; /* Create buffer for cell data */
            snprintf(group_data, sizeof(group_data), "Cell%d: %dmV ", i + 1, group_voltages_1[i]); /* Format cell voltage */
            strncat(message, group_data, MESSAGE_SIZE - strlen(message) - 1); /* Append to message */
        }
        char temp_data[88]; /* Create buffer for additional data */
        snprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC PCB: %dC SOC: %.1f%% SOH: %.1f%%", pack_current_1, pack_current_2, temperature_1, temperature_2, pcb_temperature, soc, soh); /* Format data */
        strncat(message, temp_data, MESSAGE_SIZE - strlen(message) - 1); /* Append to message */
        Log_Error(message); /* Log status */
        if (HAL_GetTick() - last_log_read >= 10000) { /* Check if 10 seconds elapsed */
            Log_Read_All(); /* Send logs */
            last_log_read = HAL_GetTick(); /* Update log read time */
        }
        if (HAL_GetTick() - last_status_send >= 5000) { /* Check if 5 seconds elapsed */
            SSP_SendStatus(); /* Send telemetry */
            last_status_send = HAL_GetTick(); /* Update status send time */
        }
        if (HAL_GetTick() - last_time_sync >= 60000) { /* Check if 60 seconds elapsed */
            SSP_TimeTypeDef time = {0}; /* Create structure for time */
            if (SSP_RequestTime(&huart2, &time) == HAL_OK) { /* Request time from OBC */
                RTC_TimeTypeDef sTime = {0}; /* Create structure for RTC time */
                RTC_DateTypeDef sDate = {0}; /* Create structure for RTC date */
                sTime.Hours = time.hour; /* Set hour */
                sTime.Minutes = time.minute; /* Set minutes */
                sTime.Seconds = time.second; /* Set seconds */
                sDate.Year = (uint8_t)(time.year - 2000); /* Set year */
                sDate.Month = time.month; /* Set month */
                sDate.Date = time.day; /* Set day */
                HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); /* Update RTC time */
                HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Update RTC date */
                Log_Error("Time synchronized with OBC"); /* Log success */
            }
            else { /* Time sync failed */
                Log_Error("Failed to synchronize time with OBC"); /* Log failure */
            }
            last_time_sync = HAL_GetTick(); /* Update sync time */
        }
        SSP_FrameTypeDef received_frame = {0}; /* Create structure for received frame */
        if (SSP_ReceiveFrame(&huart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK) { /* Check for frame */
            SSP_ProcessReceivedFrame(&received_frame); /* Process frame */
        }
        HAL_Delay((uint32_t)(LOOP_TIME * 1000)); /* Delay for loop timing */
    }
}

/* Define function to configure system clock
 * Inputs: None
 * Returns: void, meaning it returns nothing; it configures the clock and calls Error_Handler on failure.
 * What it does: Sets up the STM32’s clock using HSE and LSE oscillators for system and RTC timing.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /* Create structure for oscillator settings */
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /* Create structure for clock settings */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) /* Set voltage scaling */
        Error_Handler(); /* Call error handler if scaling fails */
    HAL_PWR_EnableBkUpAccess(); /* Enable backup domain access */
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW); /* Set low drive for LSE oscillator */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE; /* Select HSE and LSE oscillators */
    RCC_OscInitStruct.HSEState = RCC_HSE_ON; /* Enable HSE oscillator */
    RCC_OscInitStruct.LSEState = RCC_LSE_ON; /* Enable LSE oscillator */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; /* Disable PLL */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) /* Apply oscillator settings */
        Error_Handler(); /* Call error handler if configuration fails */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; /* Select clock types */
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE; /* Set HSE as system clock */
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* Set AHB divider to 1 */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; /* Set APB1 divider to 1 */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; /* Set APB2 divider to 1 */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) /* Apply clock settings */
        Error_Handler(); /* Call error handler if configuration fails */
    HAL_RCC_EnableCSS(); /* Enable Clock Security System */
    HAL_RCCEx_EnableLSECSS(); /* Enable LSE Clock Security System */
}

/* Define function to initialize I2C1 for first BQ76920
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes I2C1 and calls Error_Handler on failure.
 * What it does: Configures I2C1 for communication with the first BQ76920 IC.
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1; /* Select I2C1 peripheral */
    hi2c1.Init.Timing = 0x00201D2B; /* Set timing for ~100 kHz */
    hi2c1.Init.OwnAddress1 = 0; /* Set no own address (master mode) */
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; /* Use 7-bit addressing */
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; /* Disable dual addressing */
    hi2c1.Init.OwnAddress2 = 0; /* Set no second address */
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK; /* Disable address masking */
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; /* Disable general call */
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; /* Enable clock stretching */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) /* Initialize I2C1 */
        Error_Handler(); /* Call error handler if initialization fails */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) /* Enable analog filter */
        Error_Handler(); /* Call error handler if filter fails */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) /* Disable digital filter */
        Error_Handler(); /* Call error handler if filter fails */
}

/* Define function to initialize I2C2 for second BQ76920
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes I2C2 and calls Error_Handler on failure.
 * What it does: Configures I2C2 for communication with the second BQ76920 IC.
 */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2; /* Select I2C2 peripheral */
    hi2c2.Init.Timing = 0x00201D2B; /* Set timing for ~100 kHz */
    hi2c2.Init.OwnAddress1 = 0; /* Set no own address (master mode) */
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; /* Use 7-bit addressing */
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; /* Disable dual addressing */
    hi2c2.Init.OwnAddress2 = 0; /* Set no second address */
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK; /* Disable address masking */
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; /* Disable general call */
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; /* Enable clock stretching */
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) /* Initialize I2C2 */
        Error_Handler(); /* Call error handler if initialization fails */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) /* Enable analog filter */
        Error_Handler(); /* Call error handler if filter fails */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) /* Disable digital filter */
        Error_Handler(); /* Call error handler if filter fails */
}

/* Define function to initialize I2C3 (unused)
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes I2C3 and calls Error_Handler on failure.
 * What it does: Configures I2C3, though it is not used in this BMS implementation.
 */
static void MX_I2C3_Init(void)
{
    hi2c3.Instance = I2C3; /* Select I2C3 peripheral */
    hi2c3.Init.Timing = 0x00201D2B; /* Set timing for ~100 kHz */
    hi2c3.Init.OwnAddress1 = 0; /* Set no own address (master mode) */
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; /* Use 7-bit addressing */
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; /* Disable dual addressing */
    hi2c3.Init.OwnAddress2 = 0; /* Set no second address */
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK; /* Disable address masking */
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; /* Disable general call */
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; /* Enable clock stretching */
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) /* Initialize I2C3 */
        Error_Handler(); /* Call error handler if initialization fails */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) /* Enable analog filter */
        Error_Handler(); /* Call error handler if filter fails */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) /* Disable digital filter */
        Error_Handler(); /* Call error handler if filter fails */
}

/* Define function to initialize the Real-Time Clock
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes RTC and calls Error_Handler on failure.
 * What it does: Configures the RTC for timekeeping and timestamping logs.
 */
static void MX_RTC_Init(void)
{
    RTC_TimeTypeDef sTime = {0}; /* Create structure for time settings */
    RTC_DateTypeDef sDate = {0}; /* Create structure for date settings */
    hrtc.Instance = RTC; /* Select RTC peripheral */
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24; /* Set 24-hour format */
    hrtc.Init.AsynchPrediv = 127; /* Set asynchronous prescaler */
    hrtc.Init.SynchPrediv = 255; /* Set synchronous prescaler */
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE; /* Disable RTC output pin */
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE; /* No output remapping */
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH; /* Set output polarity */
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN; /* Set output type */
    if (HAL_RTC_Init(&hrtc) != HAL_OK) /* Initialize RTC */
        Error_Handler(); /* Call error handler if initialization fails */
    sTime.Hours = 0x0; /* Set initial hours to 0 */
    sTime.Minutes = 0x0; /* Set initial minutes to 0 */
    sTime.Seconds = 0x0; /* Set initial seconds to 0 */
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; /* Disable daylight saving */
    sTime.StoreOperation = RTC_STOREOPERATION_RESET; /* Reset stored operation */
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) /* Set RTC time */
        Error_Handler(); /* Call error handler if setting fails */
    sDate.WeekDay = RTC_WEEKDAY_MONDAY; /* Set weekday to Monday */
    sDate.Month = RTC_MONTH_JANUARY; /* Set month to January */
    sDate.Date = 0x1; /* Set date to 1 */
    sDate.Year = 0x0; /* Set year to 2000 */
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) /* Set RTC date */
        Error_Handler(); /* Call error handler if setting fails */
    if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK) /* Enable timestamp */
        Error_Handler(); /* Call error handler if timestamp fails */
}

/* Define function to initialize USART1 for logging
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes USART1 and calls Error_Handler on failure.
 * What it does: Configures USART1 for sending logs over RS485.
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1; /* Select USART1 peripheral */
    huart1.Init.BaudRate = 115200; /* Set baud rate to 115200 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B; /* Set 8-bit word length */
    huart1.Init.StopBits = UART_STOPBITS_1; /* Set 1 stop bit */
    huart1.Init.Parity = UART_PARITY_NONE; /* Disable parity */
    huart1.Init.Mode = UART_MODE_TX_RX; /* Enable TX and RX */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE; /* Disable hardware flow control */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; /* Set 16x oversampling */
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; /* Disable one-bit sampling */
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; /* Disable advanced features */
    if (HAL_UART_Init(&huart1) != HAL_OK) /* Initialize USART1 */
        Error_Handler(); /* Call error handler if initialization fails */
}

/* Define function to initialize USART2 for SSP communication
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes USART2 and calls Error_Handler on failure.
 * What it does: Configures USART2 for SSP communication over RS485, though incorrectly in half-duplex.
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2; /* Select USART2 peripheral */
    huart2.Init.BaudRate = 115200; /* Set baud rate to 115200 */
    huart2.Init.WordLength = UART_WORDLENGTH_8B; /* Set 8-bit word length */
    huart2.Init.StopBits = UART_STOPBITS_1; /* Set 1 stop bit */
    huart2.Init.Parity = UART_PARITY_NONE; /* Disable parity */
    huart2.Init.Mode = UART_MODE_TX_RX; /* Enable TX and RX */
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; /* Disable hardware flow control */
    huart2.Init.OverSampling = UART_OVERSAMPLING_16; /* Set 16x oversampling */
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; /* Disable one-bit sampling */
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; /* Disable advanced features */
    if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) /* Initialize USART2 in half-duplex */
        Error_Handler(); /* Call error handler if initialization fails */
}

/* Define function to initialize GPIO pins
 * Inputs: None
 * Returns: void, meaning it returns nothing; it configures GPIO pins.
 * What it does: Sets up GPIO pins for LED, RS485 DE, heaters, and inputs.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0}; /* Create structure for GPIO settings */
    __HAL_RCC_GPIOC_CLK_ENABLE(); /* Enable clock for GPIOC (LED) */
    __HAL_RCC_GPIOH_CLK_ENABLE(); /* Enable clock for GPIOH (unused) */
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* Enable clock for GPIOA (RS485 DE) */
    __HAL_RCC_GPIOB_CLK_ENABLE(); /* Enable clock for GPIOB (heaters, inputs) */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* Set LED pin low */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); /* Set RS485 DE pin low */
    HAL_GPIO_WritePin(GPIOB, HEATER2_Pin|HEATER1_Pin, GPIO_PIN_RESET); /* Set heater pins low */
    GPIO_InitStruct.Pin = LED_Pin; /* Select LED pin */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* Set as push-pull output */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Set low speed */
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct); /* Initialize LED pin */
    GPIO_InitStruct.Pin = RS4852_DE_Pin; /* Select RS485 DE pin */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* Set as push-pull output */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Set low speed */
    HAL_GPIO_Init(RS4852_DE_GPIO_Port, &GPIO_InitStruct); /* Initialize DE pin */
    GPIO_InitStruct.Pin = BOOT2_Pin; /* Select BOOT2 input pin */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* Set as input */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    HAL_GPIO_Init(BOOT2_GPIO_Port, &GPIO_InitStruct); /* Initialize BOOT2 pin */
    GPIO_InitStruct.Pin = ALERT2_Pin; /* Select ALERT2 input pin */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* Set as input */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    HAL_GPIO_Init(ALERT2_GPIO_Port, &GPIO_InitStruct); /* Initialize ALERT2 pin */
    GPIO_InitStruct.Pin = BOOT_Pin|ALERT_Pin; /* Select BOOT and ALERT pins */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* Set as inputs */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); /* Initialize BOOT and ALERT pins */
    GPIO_InitStruct.Pin = HEATER2_Pin|HEATER1_Pin; /* Select heater pins */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* Set as push-pull outputs */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* Disable pull resistors */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Set low speed */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); /* Initialize heater pins */
}

/* Define function to handle errors
 * Inputs: None
 * Returns: void, meaning it returns nothing; it halts the program.
 * What it does: Disables interrupts and enters an infinite loop to stop execution.
 */
void Error_Handler(void)
{
    __disable_irq(); /* Disable all interrupts */
    while (1) /* Infinite loop to halt */
    {
    }
}

#ifdef USE_FULL_ASSERT
/* Define function to handle assertion failures
 * Inputs:
 * - file: Pointer to a string (uint8_t *) with the source file name.
 * - line: Line number (uint32_t) where the assertion failed.
 * Returns: void, meaning it returns nothing; it is used for debugging.
 * What it does: Handles assertion failures in debug mode (empty implementation).
 */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
