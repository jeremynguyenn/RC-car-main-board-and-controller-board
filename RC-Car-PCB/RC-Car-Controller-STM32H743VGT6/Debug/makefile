################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/JPEG/subdir.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Middlewares/Custom/MenuOLED/Src/subdir.mk
-include Drivers/XBEE_Driver/Src/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Drivers/STC3100_Driver/Src/subdir.mk
-include Drivers/ST7789_Driver/Src/subdir.mk
-include Drivers/SSD1306_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := RC-Car-Controller-STM32H743VGT6
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RC-Car-Controller-STM32H743VGT6.elf \

MAP_FILES += \
RC-Car-Controller-STM32H743VGT6.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RC-Car-Controller-STM32H743VGT6.list \


# All Target
all: main-build

# Main-build Target
main-build: RC-Car-Controller-STM32H743VGT6.elf secondary-outputs

# Tool invocations
RC-Car-Controller-STM32H743VGT6.elf RC-Car-Controller-STM32H743VGT6.map: $(OBJS) $(USER_OBJS) C:\Users\garage-B\Documents\GitHub\RC-Car-PCB\RC-Car-Controller-STM32H743VGT6\STM32H743VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "RC-Car-Controller-STM32H743VGT6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\garage-B\Documents\GitHub\RC-Car-PCB\RC-Car-Controller-STM32H743VGT6\STM32H743VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RC-Car-Controller-STM32H743VGT6.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RC-Car-Controller-STM32H743VGT6.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RC-Car-Controller-STM32H743VGT6.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) RC-Car-Controller-STM32H743VGT6.elf RC-Car-Controller-STM32H743VGT6.list RC-Car-Controller-STM32H743VGT6.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
