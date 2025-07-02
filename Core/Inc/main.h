//main.h

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "BQ76920.h"
#include "kalman_filter.h"
#include "Temperature.h"

#define FLASH_BASE_ADDRESS 0x0803F800 // Last 2KB page for STM32L476RCT6 (256KB Flash)
#define FLASH_TELEMETRY_ADDRESS (FLASH_BASE_ADDRESS + 0x100)
#define R_NOMINAL 10000.0f
#define T_NOMINAL 25.0f
#define BETA_VALUE 3950.0f

void Error_Handler(void);

#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOC
#define BOOT2_Pin GPIO_PIN_7
#define BOOT2_GPIO_Port GPIOC
#define ALERT2_Pin GPIO_PIN_12
#define ALERT2_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_4
#define BOOT1_GPIO_Port GPIOB
#define ALERT1_Pin GPIO_PIN_5
#define ALERT1_GPIO_Port GPIOB
#define HEATER2_Pin GPIO_PIN_8
#define HEATER2_GPIO_Port GPIOB
#define HEATER1_Pin GPIO_PIN_9
#define HEATER1_GPIO_Port GPIOB
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA


typedef struct {
    uint16_t vcell_ic1[NUMBER_OF_CELLS];
    uint16_t vcell_ic2[NUMBER_OF_CELLS];
    uint16_t vpack_ic1;
    uint16_t vpack_ic2;
    int16_t current_ic1;
    int16_t current_ic2;
    float soc;
    float soh;
    float pcb_temperature;
    float pack_temperature_ic1;
    float pack_temperature_ic2;
    float die_temperature_ic1;
    float die_temperature_ic2;
    float thermistor_temperature_ic1;
    float thermistor_temperature_ic2;
    uint8_t heater1_state;  // 1 = ON, 0 = OFF
    uint8_t heater2_state;  // 1 = ON, 0 = OFF
    uint8_t balancing_active;
    uint8_t balancing_mask_ic1;
    uint8_t balancing_mask_ic2;
    uint8_t charge_immediately;
    uint8_t bms_online;
    uint8_t error_flags[8];
    uint8_t ovrd_alert_ic1;
    uint8_t ovrd_alert_ic2;
    uint8_t device_xready_ic1;
    uint8_t device_xready_ic2;
    uint8_t load_present_ic1;
    uint8_t load_present_ic2;
    uint16_t charge_cycle_count;
    uint32_t total_charge_time;
    uint32_t total_discharge_time;
    uint32_t total_operating_time;
    uint16_t raw_adc_gain_ic1;
    uint16_t raw_adc_offset_ic1;
    uint16_t raw_adc_gain_ic2;
    uint16_t raw_adc_offset_ic2;
    uint8_t i2c_comm_error_ic1;
    uint8_t i2c_comm_error_ic2;
    uint64_t sync_counter;
    uint8_t sync_valid;
} TelemetryData;

#ifdef __cplusplus
}
#endif
//Do not respond

#endif /* __MAIN_H */
