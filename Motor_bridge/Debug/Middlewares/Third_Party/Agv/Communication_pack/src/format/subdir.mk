################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.c \
../Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.c 

OBJS += \
./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.o \
./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.o 

C_DEPS += \
./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.d \
./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/Agv/Communication_pack/src/format/%.o Middlewares/Third_Party/Agv/Communication_pack/src/format/%.su Middlewares/Third_Party/Agv/Communication_pack/src/format/%.cyclo: ../Middlewares/Third_Party/Agv/Communication_pack/src/format/%.c Middlewares/Third_Party/Agv/Communication_pack/src/format/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=5 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Core/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Factory/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Communication_pack/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Host_communication/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Motors/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Control/include" -I"C:/Users/User/Desktop/STM32F407/Motor_bridge/Middlewares/Third_Party/Agv/Kinematics/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-Agv-2f-Communication_pack-2f-src-2f-format

clean-Middlewares-2f-Third_Party-2f-Agv-2f-Communication_pack-2f-src-2f-format:
	-$(RM) ./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.cyclo ./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.d ./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.o ./Middlewares/Third_Party/Agv/Communication_pack/src/format/modbus_rtu_format.su ./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.cyclo ./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.d ./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.o ./Middlewares/Third_Party/Agv/Communication_pack/src/format/ros_format.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-Agv-2f-Communication_pack-2f-src-2f-format

