################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/Agv/Core/src/core.c \
../Middlewares/Third_Party/Agv/Core/src/utils.c 

OBJS += \
./Middlewares/Third_Party/Agv/Core/src/core.o \
./Middlewares/Third_Party/Agv/Core/src/utils.o 

C_DEPS += \
./Middlewares/Third_Party/Agv/Core/src/core.d \
./Middlewares/Third_Party/Agv/Core/src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/Agv/Core/src/%.o Middlewares/Third_Party/Agv/Core/src/%.su Middlewares/Third_Party/Agv/Core/src/%.cyclo: ../Middlewares/Third_Party/Agv/Core/src/%.c Middlewares/Third_Party/Agv/Core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=5 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Core/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Factory/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Communication_pack/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Host_communication/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Motors/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Control/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Kinematics/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-Agv-2f-Core-2f-src

clean-Middlewares-2f-Third_Party-2f-Agv-2f-Core-2f-src:
	-$(RM) ./Middlewares/Third_Party/Agv/Core/src/core.cyclo ./Middlewares/Third_Party/Agv/Core/src/core.d ./Middlewares/Third_Party/Agv/Core/src/core.o ./Middlewares/Third_Party/Agv/Core/src/core.su ./Middlewares/Third_Party/Agv/Core/src/utils.cyclo ./Middlewares/Third_Party/Agv/Core/src/utils.d ./Middlewares/Third_Party/Agv/Core/src/utils.o ./Middlewares/Third_Party/Agv/Core/src/utils.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-Agv-2f-Core-2f-src

