################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/BMS_Service.c \
../Core/Src/app/BQ76920.c \
../Core/Src/app/Log.c \
../Core/Src/app/Temperature.c \
../Core/Src/app/flash.c \
../Core/Src/app/i2c_comm.c \
../Core/Src/app/kalman_filter.c 

OBJS += \
./Core/Src/app/BMS_Service.o \
./Core/Src/app/BQ76920.o \
./Core/Src/app/Log.o \
./Core/Src/app/Temperature.o \
./Core/Src/app/flash.o \
./Core/Src/app/i2c_comm.o \
./Core/Src/app/kalman_filter.o 

C_DEPS += \
./Core/Src/app/BMS_Service.d \
./Core/Src/app/BQ76920.d \
./Core/Src/app/Log.d \
./Core/Src/app/Temperature.d \
./Core/Src/app/flash.d \
./Core/Src/app/i2c_comm.d \
./Core/Src/app/kalman_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/%.o Core/Src/app/%.su Core/Src/app/%.cyclo: ../Core/Src/app/%.c Core/Src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app

clean-Core-2f-Src-2f-app:
	-$(RM) ./Core/Src/app/BMS_Service.cyclo ./Core/Src/app/BMS_Service.d ./Core/Src/app/BMS_Service.o ./Core/Src/app/BMS_Service.su ./Core/Src/app/BQ76920.cyclo ./Core/Src/app/BQ76920.d ./Core/Src/app/BQ76920.o ./Core/Src/app/BQ76920.su ./Core/Src/app/Log.cyclo ./Core/Src/app/Log.d ./Core/Src/app/Log.o ./Core/Src/app/Log.su ./Core/Src/app/Temperature.cyclo ./Core/Src/app/Temperature.d ./Core/Src/app/Temperature.o ./Core/Src/app/Temperature.su ./Core/Src/app/flash.cyclo ./Core/Src/app/flash.d ./Core/Src/app/flash.o ./Core/Src/app/flash.su ./Core/Src/app/i2c_comm.cyclo ./Core/Src/app/i2c_comm.d ./Core/Src/app/i2c_comm.o ./Core/Src/app/i2c_comm.su ./Core/Src/app/kalman_filter.cyclo ./Core/Src/app/kalman_filter.d ./Core/Src/app/kalman_filter.o ./Core/Src/app/kalman_filter.su

.PHONY: clean-Core-2f-Src-2f-app

