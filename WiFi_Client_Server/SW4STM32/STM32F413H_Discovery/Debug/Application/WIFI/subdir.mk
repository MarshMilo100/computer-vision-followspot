################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/es_wifi.c \
C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/es_wifi_io.c \
C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/wifi.c 

OBJS += \
./Application/WIFI/es_wifi.o \
./Application/WIFI/es_wifi_io.o \
./Application/WIFI/wifi.o 

C_DEPS += \
./Application/WIFI/es_wifi.d \
./Application/WIFI/es_wifi_io.d \
./Application/WIFI/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Application/WIFI/es_wifi.o: C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/es_wifi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_STM32F413H_DISCO -c -I../../../Inc -I../../../../Common/Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/STM32F413H-Discovery -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/WIFI/es_wifi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/WIFI/es_wifi_io.o: C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/es_wifi_io.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_STM32F413H_DISCO -c -I../../../Inc -I../../../../Common/Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/STM32F413H-Discovery -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/WIFI/es_wifi_io.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/WIFI/wifi.o: C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/Common/Src/wifi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_STM32F413H_DISCO -c -I../../../Inc -I../../../../Common/Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/STM32F413H-Discovery -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/WIFI/wifi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

