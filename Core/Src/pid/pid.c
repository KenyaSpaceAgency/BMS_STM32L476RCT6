/*
 * pid.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "pid.h"
#include <stdio.h>   // For snprintf
#include <stdlib.h>  // For abs

/* Private variables ---------------------------------------------------------*/
static float integral = 0.0;
static float previous_error = 0.0;
static uint32_t last_duty_cycle = 0;

/* Extern variables ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;

/**
  * @brief  Initializes the PID controller
  * @retval None
  */
void PID_Init(void)
{
    integral = 0.0;
    previous_error = 0.0;
    last_duty_cycle = 0;
}

/**
  * @brief  Controls the heaters using PID and PWM
  * @param  temp: Current temperature in degrees Celsius
  * @retval None
  */
void PID_Control(int16_t temp)
{
    if (temp >= TEMP_UPPER_LIMIT) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // HEATER2
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // HEATER1
        integral = 0.0;
        return;
    }

    float error = TARGET_TEMP - temp;
    integral += error * DT;
    float derivative = (error - previous_error) / DT;
    float output = KP * error + KI * integral + KD * derivative;

    if (output < 0) output = 0;
    if (output > 100) output = 100;

    uint32_t duty_cycle = (uint32_t)(output * 10); // 0-1000 range for PWM
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle); // HEATER2
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle); // HEATER1

    if (labs((int32_t)duty_cycle - (int32_t)last_duty_cycle) > 50) {
        char message[56];
        snprintf(message, sizeof(message), "Heater duty cycle: %lu%%, Temp: %dC", duty_cycle / 10, temp);
        Log_Error(message);
        last_duty_cycle = duty_cycle;
    }

    previous_error = error;
}
