################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/lsm303c/lsm303c.c 

OBJS += \
./Drivers/Components/lsm303c/lsm303c.o 

C_DEPS += \
./Drivers/Components/lsm303c/lsm303c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/lsm303c/%.o Drivers/Components/lsm303c/%.su Drivers/Components/lsm303c/%.cyclo: ../Drivers/Components/lsm303c/%.c Drivers/Components/lsm303c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components/lps22hb" -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components/lis3mdl" -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components/lsm6dsl" -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components" -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components/hts221" -I"/Users/alexcattani/Desktop/ECSE-444-Final-Project-menu/Project_Final/Drivers/Components/Common" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-lsm303c

clean-Drivers-2f-Components-2f-lsm303c:
	-$(RM) ./Drivers/Components/lsm303c/lsm303c.cyclo ./Drivers/Components/lsm303c/lsm303c.d ./Drivers/Components/lsm303c/lsm303c.o ./Drivers/Components/lsm303c/lsm303c.su

.PHONY: clean-Drivers-2f-Components-2f-lsm303c
