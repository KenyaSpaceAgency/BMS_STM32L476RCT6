/* Role of adc.c in the BMS Project:
 * This file configures the Analog-to-Digital Converter (ADC1) on the STM32 microcontroller
 * to read the internal temperature sensor’s voltage, enabling the BMS to monitor the Printed
 * Circuit Board (PCB) temperature in a CubeSat. It sets up ADC1 to measure channel 16,
 * providing digital temperature data for safety checks.
 */

/* Importance of adc.c in the BMS Project:
 * - Ensures Safety: PCB temperature readings help detect overtemperature conditions,
 *   preventing damage to electronics in the CubeSat’s harsh space environment.
 * - Enhances Reliability: Accurate ADC configuration provides precise temperature data,
 *   used to trigger protective actions like disabling charging if temperatures are too high.
 * - Supports Diagnostics: Temperature data is logged and sent to the On-Board Computer (OBC)
 *   via RS485 using the Simple Serial Protocol (SSP), allowing remote monitoring of BMS health.
 * - Integrates with BMS: Works with main.c’s Read_Internal_Temperature to provide data for
 *   fault detection, complementing battery temperature readings from BQ76920 ICs.
 */

/* Objective of adc.c in the BMS Project:
 * The primary objective is to initialize ADC1 to convert the analog voltage from the STM32’s
 * internal temperature sensor (channel 16) into a digital value (0-4095 for 12-bit resolution).
 * This data is used to monitor PCB temperature, ensuring safe operation of the BMS and reliable
 * communication of telemetry to the OBC over RS485, supporting the CubeSat’s power system.
 */

/* Include the ADC header file to use ADC-related functions and definitions */
#include "adc.h"

/* Define a placeholder for user-defined data types (none used in this file) */
typedef void *placeholder_typedef; /* Placeholder for custom data types, not used here */

/* Define a placeholder for user-defined constants (none used in this file) */
#define PLACEHOLDER_DEFINE 0 /* Placeholder for custom constants, not used here */

/* Define a placeholder for user-defined macros (none used in this file) */
#define PLACEHOLDER_MACRO(x) (x) /* Placeholder for custom macros, not used here */

/* Declare the ADC1 handle as external, defined elsewhere (e.g., in main.c) */
extern ADC_HandleTypeDef hadc1; /* Reference ADC1 handle for temperature sensor */

/* Declare a function to configure the system clock (not used in this file) */
void SystemClock_Config(void);

/* Define function to set up ADC1 for reading the internal temperature sensor */
void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0}; /* Create a structure to hold ADC multi-mode settings */
    ADC_ChannelConfTypeDef sConfig = {0}; /* Create a structure to hold ADC channel settings */
    hadc1.Instance = ADC1; /* Select ADC1 peripheral on the STM32 microcontroller */
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; /* Divide ADC clock by 4 to set sampling speed */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; /* Set ADC to 12-bit resolution (0-4095 range) */
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; /* Align ADC output data to the right in registers */
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; /* Disable scan mode to read one channel at a time */
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; /* Set End of Conversion flag after each conversion */
    hadc1.Init.LowPowerAutoWait = DISABLE; /* Disable low-power mode to avoid delays */
    hadc1.Init.ContinuousConvMode = DISABLE; /* Disable continuous conversions; trigger manually */
    hadc1.Init.NbrOfConversion = 1; /* Set to perform one conversion per trigger */
    hadc1.Init.DiscontinuousConvMode = DISABLE; /* Disable discontinuous mode for single conversion */
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; /* Use software to start conversions */
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Disable external trigger edge */
    hadc1.Init.DMAContinuousRequests = DISABLE; /* Disable DMA for direct data reading */
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED; /* Keep old data if new conversion overwrites it */
    hadc1.Init.OversamplingMode = DISABLE; /* Disable oversampling to simplify operation */
    if (HAL_ADC_Init(&hadc1) != HAL_OK) /* Initialize ADC1 with the specified settings */
    {
        Error_Handler(); /* Call error handler function if initialization fails */
    }
    multimode.Mode = ADC_MODE_INDEPENDENT; /* Set ADC1 to operate independently, not with other ADCs */
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) /* Apply multi-mode configuration */
    {
        Error_Handler(); /* Call error handler if multi-mode setup fails */
    }
    sConfig.Channel = ADC_CHANNEL_16; /* Select channel 16, connected to the internal temperature sensor */
    sConfig.Rank = ADC_REGULAR_RANK_1; /* Set channel 16 as the first in the conversion sequence */
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5; /* Set sampling time to 2.5 ADC clock cycles */
    sConfig.SingleDiff = ADC_SINGLE_ENDED; /* Use single-ended mode to measure voltage relative to ground */
    sConfig.OffsetNumber = ADC_OFFSET_NONE; /* Disable offset correction for raw data */
    sConfig.Offset = 0; /* Set offset value to 0 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) /* Configure the ADC channel settings */
    {
        Error_Handler(); /* Call error handler if channel configuration fails */
    }
}
