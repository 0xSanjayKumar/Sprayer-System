################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.c 

OBJS += \
./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.o 

C_DEPS += \
./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L4xx_HAL_Driver/Legacy/%.o Drivers/STM32L4xx_HAL_Driver/Legacy/%.su Drivers/STM32L4xx_HAL_Driver/Legacy/%.cyclo: ../Drivers/STM32L4xx_HAL_Driver/Legacy/%.c Drivers/STM32L4xx_HAL_Driver/Legacy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -IC:/Users/Yash/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/Yash/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/Yash/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/Yash/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Legacy

clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Legacy:
	-$(RM) ./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.cyclo ./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.d ./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.o ./Drivers/STM32L4xx_HAL_Driver/Legacy/stm32l4xx_hal_can.su

.PHONY: clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Legacy

