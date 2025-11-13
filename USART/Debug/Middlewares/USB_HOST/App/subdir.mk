################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/USB_HOST/App/usb_host.c 

OBJS += \
./Middlewares/USB_HOST/App/usb_host.o 

C_DEPS += \
./Middlewares/USB_HOST/App/usb_host.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_HOST/App/%.o Middlewares/USB_HOST/App/%.su Middlewares/USB_HOST/App/%.cyclo: ../Middlewares/USB_HOST/App/%.c Middlewares/USB_HOST/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=5 -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-USB_HOST-2f-App

clean-Middlewares-2f-USB_HOST-2f-App:
	-$(RM) ./Middlewares/USB_HOST/App/usb_host.cyclo ./Middlewares/USB_HOST/App/usb_host.d ./Middlewares/USB_HOST/App/usb_host.o ./Middlewares/USB_HOST/App/usb_host.su

.PHONY: clean-Middlewares-2f-USB_HOST-2f-App

