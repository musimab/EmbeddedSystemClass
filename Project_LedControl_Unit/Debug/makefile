################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Proje_LedControl_Unit.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Proje_LedControl_Unit.list \

OBJCOPY_BIN += \
Proje_LedControl_Unit.bin \


# All Target
all: Proje_LedControl_Unit.elf secondary-outputs

# Tool invocations
Proje_LedControl_Unit.elf: $(OBJS) $(USER_OBJS) P:\STM_Cube_IDE_Projects\023_HAL_CAN_HC-SR04_RECEIVE_DATA\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Proje_LedControl_Unit.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"P:\STM_Cube_IDE_Projects\023_HAL_CAN_HC-SR04_RECEIVE_DATA\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Proje_LedControl_Unit.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Proje_LedControl_Unit.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Proje_LedControl_Unit.list"
	@echo 'Finished building: $@'
	@echo ' '

Proje_LedControl_Unit.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Proje_LedControl_Unit.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
