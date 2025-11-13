################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/USB_HOST/Target/usbh_conf.c \
../Middlewares/USB_HOST/Target/usbh_platform.c 

OBJS += \
./Middlewares/USB_HOST/Target/usbh_conf.o \
./Middlewares/USB_HOST/Target/usbh_platform.o 

C_DEPS += \
./Middlewares/USB_HOST/Target/usbh_conf.d \
./Middlewares/USB_HOST/Target/usbh_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_HOST/Target/%.o Middlewares/USB_HOST/Target/%.su Middlewares/USB_HOST/Target/%.cyclo: ../Middlewares/USB_HOST/Target/%.c Middlewares/USB_HOST/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=5 -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-USB_HOST-2f-Target

clean-Middlewares-2f-USB_HOST-2f-Target:
	-$(RM) ./Middlewares/USB_HOST/Target/usbh_conf.cyclo ./Middlewares/USB_HOST/Target/usbh_conf.d ./Middlewares/USB_HOST/Target/usbh_conf.o ./Middlewares/USB_HOST/Target/usbh_conf.su ./Middlewares/USB_HOST/Target/usbh_platform.cyclo ./Middlewares/USB_HOST/Target/usbh_platform.d ./Middlewares/USB_HOST/Target/usbh_platform.o ./Middlewares/USB_HOST/Target/usbh_platform.su

.PHONY: clean-Middlewares-2f-USB_HOST-2f-Target

