################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Temperature/temperature.c 

OBJS += \
./Core/Src/Temperature/temperature.o 

C_DEPS += \
./Core/Src/Temperature/temperature.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Temperature/%.o Core/Src/Temperature/%.su Core/Src/Temperature/%.cyclo: ../Core/Src/Temperature/%.c Core/Src/Temperature/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Temperature

clean-Core-2f-Src-2f-Temperature:
	-$(RM) ./Core/Src/Temperature/temperature.cyclo ./Core/Src/Temperature/temperature.d ./Core/Src/Temperature/temperature.o ./Core/Src/Temperature/temperature.su

.PHONY: clean-Core-2f-Src-2f-Temperature

