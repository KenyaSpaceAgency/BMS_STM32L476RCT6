################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/kalman\ filter/kalman_filter.c 

OBJS += \
./Core/Src/kalman\ filter/kalman_filter.o 

C_DEPS += \
./Core/Src/kalman\ filter/kalman_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/kalman\ filter/kalman_filter.o: ../Core/Src/kalman\ filter/kalman_filter.c Core/Src/kalman\ filter/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/kalman filter/kalman_filter.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-kalman-20-filter

clean-Core-2f-Src-2f-kalman-20-filter:
	-$(RM) ./Core/Src/kalman\ filter/kalman_filter.cyclo ./Core/Src/kalman\ filter/kalman_filter.d ./Core/Src/kalman\ filter/kalman_filter.o ./Core/Src/kalman\ filter/kalman_filter.su

.PHONY: clean-Core-2f-Src-2f-kalman-20-filter

