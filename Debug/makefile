################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/planificador/subdir.mk
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
Inclinometro.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Inclinometro.list \

OBJCOPY_BIN += \
Inclinometro.bin \


# All Target
all: Inclinometro.elf secondary-outputs

# Tool invocations
Inclinometro.elf: $(OBJS) $(USER_OBJS) N:\Documentos\TD2\Proyecto\TD2_Grupo7_Inclinometro_99f29e5ae7d70251f41d07addf2b9cd188657a98\TD2-Grupo7-Inclinometro-99f29e5ae7d70251f41d07addf2b9cd188657a98\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Inclinometro.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"N:\Documentos\TD2\Proyecto\TD2_Grupo7_Inclinometro_99f29e5ae7d70251f41d07addf2b9cd188657a98\TD2-Grupo7-Inclinometro-99f29e5ae7d70251f41d07addf2b9cd188657a98\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Inclinometro.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Inclinometro.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Inclinometro.list"
	@echo 'Finished building: $@'
	@echo ' '

Inclinometro.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Inclinometro.bin"
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
