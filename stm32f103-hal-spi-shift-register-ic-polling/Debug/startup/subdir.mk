################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103-hal-spi-shift-register-ic-polling/inc" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/CMSIS/core" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/CMSIS/device" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/HAL_Driver/Inc/Legacy" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/HAL_Driver/Inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


