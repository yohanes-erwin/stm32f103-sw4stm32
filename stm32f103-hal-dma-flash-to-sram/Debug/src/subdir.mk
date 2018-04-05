################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f1xx.c 

OBJS += \
./src/main.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f1xx.o 

C_DEPS += \
./src/main.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F1 -DSTM32F103C8Tx -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103-hal-dma-flash-to-sram/inc" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/CMSIS/core" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/CMSIS/device" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/HAL_Driver/Inc/Legacy" -I"D:/Online Teaching/Udemy/Rev. Code/stm32f103c8-custom_hal_lib/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


