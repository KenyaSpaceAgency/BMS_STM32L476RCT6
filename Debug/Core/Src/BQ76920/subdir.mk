################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BQ76920/BQ76920.c 

OBJS += \
./Core/Src/BQ76920/BQ76920.o 

C_DEPS += \
./Core/Src/BQ76920/BQ76920.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BQ76920/%.o Core/Src/BQ76920/%.su Core/Src/BQ76920/%.cyclo: ../Core/Src/BQ76920/%.c Core/Src/BQ76920/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BQ76920

clean-Core-2f-Src-2f-BQ76920:
	-$(RM) ./Core/Src/BQ76920/BQ76920.cyclo ./Core/Src/BQ76920/BQ76920.d ./Core/Src/BQ76920/BQ76920.o ./Core/Src/BQ76920/BQ76920.su

.PHONY: clean-Core-2f-Src-2f-BQ76920

