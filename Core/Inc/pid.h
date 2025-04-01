/*
 * pid.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define TARGET_TEMP         20  // Target temperature in °C
#define TEMP_UPPER_LIMIT    30  // Upper limit to turn off heaters
#define KP                  10.0 // Proportional gain
#define KI                  0.1  // Integral gain
#define KD                  1.0  // Derivative gain
#define DT                  1.0  // Time step (1 second)

/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(void);
void PID_Control(int16_t temp);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
