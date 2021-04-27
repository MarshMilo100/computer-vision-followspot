################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/WiFi_Client_Server/SW4STM32/startup_stm32f413xx.s 

OBJS += \
./SW4STM32/startup_stm32f413xx.o 

S_DEPS += \
./SW4STM32/startup_stm32f413xx.d 


# Each subdirectory must supply rules for building sources it contributes
SW4STM32/startup_stm32f413xx.o: C:/Users/Bobby/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Projects/STM32F413H-Discovery/Applications/WIFI/WiFi_Client_Server/SW4STM32/startup_stm32f413xx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"SW4STM32/startup_stm32f413xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

