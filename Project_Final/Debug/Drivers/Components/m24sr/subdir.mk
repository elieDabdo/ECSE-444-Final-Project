################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/m24sr/m24sr.c 

OBJS += \
./Drivers/Components/m24sr/m24sr.o 

C_DEPS += \
./Drivers/Components/m24sr/m24sr.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/m24sr/%.o Drivers/Components/m24sr/%.su Drivers/Components/m24sr/%.cyclo: ../Drivers/Components/m24sr/%.c Drivers/Components/m24sr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components/lps22hb" -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components/lis3mdl" -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components/lsm6dsl" -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components" -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components/hts221" -I"C:/Users/eliea/STM32CubeIDE/workspace_1.16.0/Project_Final/Drivers/Components/Common" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-m24sr

clean-Drivers-2f-Components-2f-m24sr:
	-$(RM) ./Drivers/Components/m24sr/m24sr.cyclo ./Drivers/Components/m24sr/m24sr.d ./Drivers/Components/m24sr/m24sr.o ./Drivers/Components/m24sr/m24sr.su

.PHONY: clean-Drivers-2f-Components-2f-m24sr

