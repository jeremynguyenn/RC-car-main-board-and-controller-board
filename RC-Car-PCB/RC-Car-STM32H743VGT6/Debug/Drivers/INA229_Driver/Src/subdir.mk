################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/INA229_Driver/Src/INA229.c 

OBJS += \
./Drivers/INA229_Driver/Src/INA229.o 

C_DEPS += \
./Drivers/INA229_Driver/Src/INA229.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/INA229_Driver/Src/%.o Drivers/INA229_Driver/Src/%.su Drivers/INA229_Driver/Src/%.cyclo: ../Drivers/INA229_Driver/Src/%.c Drivers/INA229_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/XBEE_Driver/Inc -I../Drivers/INA229_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Utilities/JPEG -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-INA229_Driver-2f-Src

clean-Drivers-2f-INA229_Driver-2f-Src:
	-$(RM) ./Drivers/INA229_Driver/Src/INA229.cyclo ./Drivers/INA229_Driver/Src/INA229.d ./Drivers/INA229_Driver/Src/INA229.o ./Drivers/INA229_Driver/Src/INA229.su

.PHONY: clean-Drivers-2f-INA229_Driver-2f-Src

