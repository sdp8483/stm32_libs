################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f407xx_gpio.c 

OBJS += \
./Drivers/Src/stm32f407xx_gpio.o 

C_DEPS += \
./Drivers/Src/stm32f407xx_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f407xx_gpio.o: ../Drivers/Src/stm32f407xx_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DDEBUG -DSTM32F407VETx -c -I../Inc -I"/home/sam/STM32CubeIDE/MCU1/stm32f4xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xx_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

