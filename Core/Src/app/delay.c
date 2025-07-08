/*
 * delay.c
 *
 *  Created on: Jul 7, 2025
 *      Author: yomue
 */

/*
 * delay.c
 *
 *  Created on: Jul 3, 2025
 *      Author: yomue
 */
#include "main.h" // Include main header file for hardware definitions
#include "delay.h" // Include the header file for software delay functions
#include "stm32l4xx_hal.h" // Include HAL library for STM32


extern TIM_HandleTypeDef htim2;

static inline void software_delay_start(SoftwareDelay_t* timer, uint32_t delay_ms) {
    timer->start_time = __HAL_TIM_GET_COUNTER(&htim2);
    timer->delay_ms = delay_ms;
}

static inline uint8_t software_delay_elapsed(SoftwareDelay_t* timer) {
    uint32_t elapsed_us = (__HAL_TIM_GET_COUNTER(&htim2) - timer->start_time);
    return (elapsed_us >= (timer->delay_ms * 1000));
}

void SoftwareDelay(uint32_t delay_ms) {
    SoftwareDelay_t timer;
    software_delay_start(&timer, delay_ms);
    while (!software_delay_elapsed(&timer));
}

