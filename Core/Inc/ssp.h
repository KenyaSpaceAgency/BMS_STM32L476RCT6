/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ssp.h
  * @brief          : Header for ssp.c file.
  *                   This file contains the common defines of the SSP protocol.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSP_H
#define __SSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SSP_ADDR_EPS 0x01
#define SSP_ADDR_OBC 0x02
#define SSP_ADDR_BROADCAST 0xFF
#define SSP_ADDR_MULTICAST 0xFE

#define NUM_GROUPS_PER_IC 4 // Updated for 4S configuration (4 series groups)
#define SSP_MAX_DATA_LEN 64 // Maximum data length for SSP frames
#define SSP_MAX_FRAME_LEN (SSP_MAX_DATA_LEN + 5) // Including header and CRC

#define SSP_FLAG 0x7E
#define SSP_HEADER_SIZE 5

#define SSP_FRAME_TYPE_REPLY 0x40
#define SSP_CMD_TYPE_TIMETAG 0x80

#define SSP_CMD_PING 0x00
#define SSP_CMD_SON 0x0B
#define SSP_CMD_SOF 0x0C
#define SSP_CMD_KEN 0x31
#define SSP_CMD_KDIS 0x32
#define SSP_CMD_SM 0x15
#define SSP_CMD_GM 0x16
#define SSP_CMD_GOSTM 0x25
#define SSP_CMD_SFP 0x1B
#define SSP_CMD_GSTLM 0x22
#define SSP_CMD_GOTLM 0x21
#define SSP_CMD_GTIME 0x23
#define SSP_CMD_FIRMWARE_UPDATE 0x40

#define SSP_CMD_ACK 0x00
#define SSP_CMD_NACK 0x01

// Note: Telemetry includes 4 group voltages for 4S2P configuration.
// Pin configuration: VCO=ground, VC1=Cell 2 positive, VC2=Cell 3 positive,
// VC3=VC4=Cell 3 positive, VC5=Cell 4 positive. Cell 1 voltage is set to 0.
/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint8_t mode;
    uint8_t charge_enabled;
    uint8_t discharge_enabled;
    uint8_t charge_immediately;
    uint8_t bms_online;
    uint32_t error_flags;
    uint16_t pack_voltage_1;
    uint16_t pack_voltage_2;
    int16_t pack_current_1;
    int16_t pack_current_2;
    uint8_t soc;
    uint8_t soh;
    int16_t temp_1;
    int16_t temp_2;
    int16_t pcb_temp;
    uint16_t group_voltages[NUM_GROUPS_PER_IC]; // Updated to 4 groups for 4S2P
    uint8_t balancing_active;
    uint8_t balancing_mask_1;
    uint8_t balancing_mask_2;
    uint32_t charge_cycle_count;
    uint32_t total_charge_time;
    uint32_t total_discharge_time;
    uint32_t total_operating_time;
} SSP_TelemetryTypeDef;

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} SSP_TimeTypeDef;

typedef struct {
    uint8_t dest;
    uint8_t src;
    uint8_t cmd_id;
    uint8_t data_len;
    uint8_t data[SSP_MAX_DATA_LEN];
    uint16_t crc;
} SSP_FrameTypeDef;
/* USER CODE END ET */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef SSP_ReceiveFrame(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame);
HAL_StatusTypeDef SSP_TransmitFrame(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t buffer_len);
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame);
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *buffer_len);
HAL_StatusTypeDef SSP_RequestTime(UART_HandleTypeDef *huart, SSP_TimeTypeDef *time);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __SSP_H */
