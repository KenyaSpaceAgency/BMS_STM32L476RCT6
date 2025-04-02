################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CC-CV/cccv.c 

OBJS += \
./Core/Src/CC-CV/cccv.o 

C_DEPS += \
./Core/Src/CC-CV/cccv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/CC-CV/%.o Core/Src/CC-CV/%.su Core/Src/CC-CV/%.cyclo: ../Core/Src/CC-CV/%.c Core/Src/CC-CV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/kalman filter" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/pid" -I"C:/Users/yomue/STM32CubeIDE/EPS_BMS/BMS_STM32L476RCT6/Core/Src/Temperature" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-CC-2d-CV

clean-Core-2f-Src-2f-CC-2d-CV:
	-$(RM) ./Core/Src/CC-CV/cccv.cyclo ./Core/Src/CC-CV/cccv.d ./Core/Src/CC-CV/cccv.o ./Core/Src/CC-CV/cccv.su

.PHONY: clean-Core-2f-Src-2f-CC-2d-CV

