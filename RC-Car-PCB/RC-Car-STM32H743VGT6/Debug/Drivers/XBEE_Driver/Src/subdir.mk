################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/XBEE_Driver/Src/XBEE.c 

OBJS += \
./Drivers/XBEE_Driver/Src/XBEE.o 

C_DEPS += \
./Drivers/XBEE_Driver/Src/XBEE.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/XBEE_Driver/Src/%.o Drivers/XBEE_Driver/Src/%.su Drivers/XBEE_Driver/Src/%.cyclo: ../Drivers/XBEE_Driver/Src/%.c Drivers/XBEE_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/XBEE_Driver/Inc -I../Drivers/INA229_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Utilities/JPEG -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-XBEE_Driver-2f-Src

clean-Drivers-2f-XBEE_Driver-2f-Src:
	-$(RM) ./Drivers/XBEE_Driver/Src/XBEE.cyclo ./Drivers/XBEE_Driver/Src/XBEE.d ./Drivers/XBEE_Driver/Src/XBEE.o ./Drivers/XBEE_Driver/Src/XBEE.su

.PHONY: clean-Drivers-2f-XBEE_Driver-2f-Src

