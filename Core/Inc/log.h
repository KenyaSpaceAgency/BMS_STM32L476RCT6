#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdarg.h>
#include <stdint.h>
#include "bms_service.h"  // 👈 This line ensures TelemetryData is defined

typedef enum {
    BMS_MSG_LEVEL_OFF = 0,
    BMS_MSG_LEVEL_ERROR,
    BMS_MSG_LEVEL_WARNING,
    BMS_MSG_LEVEL_INFO,
    BMS_MSG_LEVEL_VERBOSE,
    BMS_MSG_LEVEL_DEBUG
} BMS_LogLevel;

void Log_Message(BMS_LogLevel level, const char *format, ...);
void Log_Telemetry(BMS_LogLevel level, const TelemetryData *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */
