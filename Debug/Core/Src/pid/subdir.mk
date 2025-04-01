################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/pid/pid.c 

OBJS += \
./Core/Src/pid/pid.o 

C_DEPS += \
./Core/Src/pid/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/pid/%.o Core/Src/pid/%.su Core/Src/pid/%.cyclo: ../Core/Src/pid/%.c Core/Src/pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/kalman filter" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/pid" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/Temperature" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-pid

clean-Core-2f-Src-2f-pid:
	-$(RM) ./Core/Src/pid/pid.cyclo ./Core/Src/pid/pid.d ./Core/Src/pid/pid.o ./Core/Src/pid/pid.su

.PHONY: clean-Core-2f-Src-2f-pid

