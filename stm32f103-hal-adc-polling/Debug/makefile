################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: stm32f103-hal-adc-polling.elf

dependents:
	-cd D:/Online\ Teaching/Udemy/Rev.\ Code/stm32f103c8-custom_hal_lib/Debug && $(MAKE) clean all

# Tool invocations
stm32f103-hal-adc-polling.elf: $(OBJS) $(USER_OBJS) D:/Online\ Teaching/Udemy/Rev.\ Code/stm32f103c8-custom_hal_lib/Debug/libstm32f103c8-custom_hal_lib.a
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -L"D:\Online Teaching\Udemy\Rev. Code\stm32f103c8-custom_hal_lib\Debug" -T"D:\Online Teaching\Udemy\Rev. Code\stm32f103-hal-adc-polling\LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm -o "stm32f103-hal-adc-polling.elf" @"objects.list" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "stm32f103-hal-adc-polling.elf" "stm32f103-hal-adc-polling.bin"
	arm-none-eabi-size "stm32f103-hal-adc-polling.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
D:/Online\ Teaching/Udemy/Rev.\ Code/stm32f103c8-custom_hal_lib/Debug/libstm32f103c8-custom_hal_lib.a:

-include ../makefile.targets
