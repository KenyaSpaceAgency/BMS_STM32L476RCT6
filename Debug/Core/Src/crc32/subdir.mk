################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/crc32/crc16.c 

OBJS += \
./Core/Src/crc32/crc16.o 

C_DEPS += \
./Core/Src/crc32/crc16.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/crc32/%.o Core/Src/crc32/%.su Core/Src/crc32/%.cyclo: ../Core/Src/crc32/%.c Core/Src/crc32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/kalman filter" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/pid" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/Temperature" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-crc32

clean-Core-2f-Src-2f-crc32:
	-$(RM) ./Core/Src/crc32/crc16.cyclo ./Core/Src/crc32/crc16.d ./Core/Src/crc32/crc16.o ./Core/Src/crc32/crc16.su

.PHONY: clean-Core-2f-Src-2f-crc32

