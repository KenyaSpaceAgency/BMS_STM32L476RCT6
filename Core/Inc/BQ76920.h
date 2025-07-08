/*
 * BQ76920.h
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */

//BQ76920.h

#ifndef INC_BQ76920_H_
#define INC_BQ76920_H_

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "kalman_filter.h"


#define BQ76920_ADDRESS (0x18 << 1)
#define NUMBER_OF_CELLS 4
#define RSENSE 1
#define balanceThreshold 0.05f
#define OV_THRESHOLD 4.3f
#define UV_THRESHOLD 2.5f
#define VOLTAGE_READ_INTERVAL 250
#define LOOP_TIME 0.1f
#define CRC8_POLYNOMIAL 0x07


#define SYS_STAT 0x00
#define CELLBAL1 0x01
#define CELLBAL2 0x02
#define CELLBAL3 0x03
#define SYS_CTRL1 0x04
#define SYS_CTRL2 0x05
#define PROTECT1 0x06
#define PROTECT2 0x07
#define PROTECT3 0x08
#define OV_TRIP 0x09
#define UV_TRIP 0x0A
#define CC_CFG 0x0B
#define VC1_HI 0x0C
#define VC1_LO 0x0D
#define VC2_HI 0x0E
#define VC2_LO 0x0F
#define VC3_HI 0x10
#define VC3_LO 0x11
#define VC4_HI 0x12
#define VC4_LO 0x13
#define VC5_HI 0x14
#define VC5_LO 0x15
#define BAT_HI 0x2A
#define BAT_LO 0x2B
#define TS1_HI 0x2C
#define TS1_LO 0x2D
#define CC_HI 0x32
#define CC_LO 0x33
#define ADCGAIN1 0x50
#define ADCOFFSET 0x51
#define ADCGAIN2 0x59

#define grossOV 4.18f
#define netOV 4.02f
#define netUV 3.08f
#define grossUV 2.7f
#define grossCapacity 3200
#define netCapacity 2240
#define nominalV 3.67f
#define nominalPackV 14.68f
#define thresholdRange 0.2f
#define MaxChargeCurrent 3100
#define MaxDischargeCurrent 10000
#define ROUND_TRIP_EFFICIENCY 0.9f

// BQ76920 gain/offset calibration registers (per datasheet Table 11)
#define CC_GAIN1   0x50  // Current gain MSB
#define CC_GAIN2   0x51  // Current gain LSB
#define CC_OFFSET  0x52  // Current offset (signed)



// Define simple macros to validate measurements. These help catch bad sensor reads early.
#define IS_VALID_VOLTAGE(v)    ((v) >= 2.5f && (v) <= 4.5f)       // Acceptable voltage range per cell (V)
#define IS_VALID_CURRENT(c)    (fabsf(c) < 200.0f)                // Max ±200A range
#define IS_VALID_TEMPERATURE(t) ((t) >= -40.0f && (t) <= 85.0f)   // Temperature sensor sanity: -40°C to +85°C


enum SCD_D {
    SDC_70us_delay = 0x00,
    SDC_100us_delay = 0x01,
    SDC_200us_delay = 0x02,
    SDC_400us_delay = 0x03
};

enum SCD_T {
    SCD_Threshold_44mV = 0x00,
    SCD_Threshold_67mV = 0x01,
    SCD_Threshold_89mV = 0x02,
    SCD_Threshold_111mV = 0x03,
    SCD_Threshold_133mV = 0x04,
    SCD_Threshold_155mV = 0x05,
    SCD_Threshold_178mV = 0x06,
    SCD_Threshold_200mV = 0x07
};

enum OCD_D {
    ODC_8ms_delay = 0x00,
    ODC_20ms_delay = 0x01,
    ODC_40ms_delay = 0x02,
    ODC_80ms_delay = 0x03,
    ODC_160ms_delay = 0x04,
    ODC_320ms_delay = 0x05,
    ODC_640ms_delay = 0x06,
    ODC_1280ms_delay = 0x07
};

enum OCD_T {
    OCD_Threshold_8mV = 0x00,
    OCD_Threshold_11mV = 0x01,
    OCD_Threshold_14mV = 0x02,
    OCD_Threshold_17mV = 0x03,
    OCD_Threshold_19mV = 0x04,
    OCD_Threshold_22mV = 0x05,
    OCD_Threshold_25mV = 0x06,
    OCD_Threshold_28mV = 0x07,
    OCD_Threshold_31mV = 0x08,
    OCD_Threshold_33mV = 0x09,
    OCD_Threshold_36mV = 0x0A,
    OCD_Threshold_39mV = 0x0B,
    OCD_Threshold_42mV = 0x0C,
    OCD_Threshold_44mV = 0x0D,
    OCD_Threshold_47mV = 0x0E,
    OCD_Threshold_50mV = 0x0F
};

enum UV_D {
    UV_Delay_1s = 0x00,
    UV_Delay_4s = 0x01,
    UV_Delay_8s = 0x02,
    UV_Delay_16s = 0x03
};

enum OV_D {
    OV_Delay_1s = 0x00,
    OV_Delay_2s = 0x01,
    OV_Delay_4s = 0x02,
    OV_Delay_8s = 0x03
};

enum cell {
    VC1 = VC1_HI,
    VC2 = VC2_HI,
    VC3 = VC3_HI,
	VC4 = VC3_HI, // VC4 is tied to VC3
    VC5 = VC5_HI
};

typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    GPIO_TypeDef *bootPort;
    uint16_t bootPin;
    int8_t OFFSET;
    uint16_t GAIN;
    uint8_t Alert[8];
    int32_t currentUsage;
    int32_t wattUsage;
    float Vcell[4]; // VC1, VC2, VC3, VC5 (VC4 tied to VC3)
    float Vpack;
    float SOC;
    float SOCEnergy;
    float SOCCapacity;
    float SOH;
    float smallestV;
    float SOHEnergy;
    float SOHCapacity;
    float SOHOCV;
} BQ76920_t;

typedef struct {
    float soc_energy;
    float soc_capacity;
    float soh;
    float soh_energy;
    float soh_capacity;
    float soh_ocv;
    int32_t watt_usage;
    int32_t current_usage;
    uint8_t balancing_mask;
    uint8_t sys_ctrl2_flags;
} BMSMetrics_t;

void BQ76920_Initialise(BQ76920_t *BMS, I2C_HandleTypeDef *i2cHandle);
float getCellVoltage(BQ76920_t *BMS, int cell);
float getPackVoltage(BQ76920_t *BMS);
float getCurrent(BQ76920_t *BMS);
float SOCPack(BQ76920_t *BMS, float PackCurrent, float Vpack);
float SOHPack(BQ76920_t *BMS);
void readAlert(BQ76920_t *BMS);
bool EnableBalanceCell(BQ76920_t *BMS, float PackCurrent);
void turnCHGOn(BQ76920_t *BMS);
void turnDSGOn(BQ76920_t *BMS);
void turnCHGOff(BQ76920_t *BMS);
void turnDSGOff(BQ76920_t *BMS);
void CLEAR_SYS_STAT(BQ76920_t *BMS);
uint8_t getAlert(BQ76920_t *BMS, uint8_t k);
uint8_t checkUV(float Vcell[4]);
uint8_t checkNotUV(float Vcell[4], uint8_t UV);
uint8_t checkOV(float Vcell[4]);
uint8_t checkNotOV(float Vcell[4], uint8_t OV);
uint8_t checkOC(float PackCurrent);
uint8_t checkNotOC(float PackCurrent, uint8_t OC);
uint8_t checkSC(float PackCurrent);
uint8_t checkNotSC(float PackCurrent, uint8_t SC);
void justWrite1(BQ76920_t *BMS);
uint8_t justRead1(BQ76920_t *BMS);
uint8_t justRead2(BQ76920_t *BMS);
float justGetter1(BQ76920_t *BMS);
float justGetter2(BQ76920_t *BMS);
float justGetter3(BQ76920_t *BMS);
float justGetter4(BQ76920_t *BMS);
float justGetter5(BQ76920_t *BMS);
float justGetter6(BQ76920_t *BMS);
int32_t justGetter7(BQ76920_t *BMS);
int32_t justGetter8(BQ76920_t *BMS);
void BQ76920_ReadGainAndOffset(BQ76920_t *BMS);
HAL_StatusTypeDef BQ76920_ReadRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc);
HAL_StatusTypeDef BQ76920_WriteRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc);
float BQ76920_GetTemperature(BQ76920_t *BMS, uint8_t temp_sel);
float BQ76920_GetExternalThermistorTemperature(BQ76920_t *BMS, float beta, float R_nominal, float T_nominal);


#endif /* INC_BQ76920_H_ */
//Do not respond


