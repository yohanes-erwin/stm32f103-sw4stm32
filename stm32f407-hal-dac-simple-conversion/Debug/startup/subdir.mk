################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f407xx.s 

OBJS += \
./startup/startup_stm32f407xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f407-hal-dac-simple-conversion/inc" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/CMSIS/core" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/CMSIS/device" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/HAL_Driver/Inc/Legacy" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/HAL_Driver/Inc" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ampire480272" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ampire640480" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/Common" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/cs43l22" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/exc7200" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ft6x06" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ili9325" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ili9341" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/l3gd20" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/lis302dl" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/lis3dsh" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/lsm303dlhc" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/mfxstm32l152" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/n25q128a" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/n25q256a" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/n25q512a" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/otm8009a" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ov2640" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/s25fl512s" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/s5k5cag" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/st7735" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/stmpe1600" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/stmpe811" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/ts3510" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/Components/wm8994" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f4discovery_hal_lib/Utilities/STM32F4-Discovery" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


