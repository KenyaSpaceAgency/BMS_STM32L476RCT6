################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/BMS_Service.c \
../Core/Src/app/BQ76920.c \
../Core/Src/app/Log.c \
../Core/Src/app/RS485slave.c \
../Core/Src/app/Temperature.c \
../Core/Src/app/delay.c \
../Core/Src/app/eeprom.c \
../Core/Src/app/flash.c \
../Core/Src/app/i2c_comm.c \
../Core/Src/app/i2c_command_handlers.c \
../Core/Src/app/kalman_filter.c 

OBJS += \
./Core/Src/app/BMS_Service.o \
./Core/Src/app/BQ76920.o \
./Core/Src/app/Log.o \
./Core/Src/app/RS485slave.o \
./Core/Src/app/Temperature.o \
./Core/Src/app/delay.o \
./Core/Src/app/eeprom.o \
./Core/Src/app/flash.o \
./Core/Src/app/i2c_comm.o \
./Core/Src/app/i2c_command_handlers.o \
./Core/Src/app/kalman_filter.o 

C_DEPS += \
./Core/Src/app/BMS_Service.d \
./Core/Src/app/BQ76920.d \
./Core/Src/app/Log.d \
./Core/Src/app/RS485slave.d \
./Core/Src/app/Temperature.d \
./Core/Src/app/delay.d \
./Core/Src/app/eeprom.d \
./Core/Src/app/flash.d \
./Core/Src/app/i2c_comm.d \
./Core/Src/app/i2c_command_handlers.d \
./Core/Src/app/kalman_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/%.o Core/Src/app/%.su Core/Src/app/%.cyclo: ../Core/Src/app/%.c Core/Src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Core/Src/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Core/Src/FreeRTOS/Source/portable/GCC/ARM_CM4F/ -I../Core/Src/FreeRTOS/Source/include -I../Core/Src/FreeRTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app

clean-Core-2f-Src-2f-app:
	-$(RM) ./Core/Src/app/BMS_Service.cyclo ./Core/Src/app/BMS_Service.d ./Core/Src/app/BMS_Service.o ./Core/Src/app/BMS_Service.su ./Core/Src/app/BQ76920.cyclo ./Core/Src/app/BQ76920.d ./Core/Src/app/BQ76920.o ./Core/Src/app/BQ76920.su ./Core/Src/app/Log.cyclo ./Core/Src/app/Log.d ./Core/Src/app/Log.o ./Core/Src/app/Log.su ./Core/Src/app/RS485slave.cyclo ./Core/Src/app/RS485slave.d ./Core/Src/app/RS485slave.o ./Core/Src/app/RS485slave.su ./Core/Src/app/Temperature.cyclo ./Core/Src/app/Temperature.d ./Core/Src/app/Temperature.o ./Core/Src/app/Temperature.su ./Core/Src/app/delay.cyclo ./Core/Src/app/delay.d ./Core/Src/app/delay.o ./Core/Src/app/delay.su ./Core/Src/app/eeprom.cyclo ./Core/Src/app/eeprom.d ./Core/Src/app/eeprom.o ./Core/Src/app/eeprom.su ./Core/Src/app/flash.cyclo ./Core/Src/app/flash.d ./Core/Src/app/flash.o ./Core/Src/app/flash.su ./Core/Src/app/i2c_comm.cyclo ./Core/Src/app/i2c_comm.d ./Core/Src/app/i2c_comm.o ./Core/Src/app/i2c_comm.su ./Core/Src/app/i2c_command_handlers.cyclo ./Core/Src/app/i2c_command_handlers.d ./Core/Src/app/i2c_command_handlers.o ./Core/Src/app/i2c_command_handlers.su ./Core/Src/app/kalman_filter.cyclo ./Core/Src/app/kalman_filter.d ./Core/Src/app/kalman_filter.o ./Core/Src/app/kalman_filter.su

.PHONY: clean-Core-2f-Src-2f-app

