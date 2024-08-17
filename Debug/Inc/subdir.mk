################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/misc.c \
../Inc/stm32f4xx_dcmi.c \
../Inc/stm32f4xx_dma.c \
../Inc/stm32f4xx_gpio.c \
../Inc/stm32f4xx_i2c.c \
../Inc/stm32f4xx_rcc.c \
../Inc/stm32f4xx_tim.c \
../Inc/stm32f4xx_usart.c 

OBJS += \
./Inc/misc.o \
./Inc/stm32f4xx_dcmi.o \
./Inc/stm32f4xx_dma.o \
./Inc/stm32f4xx_gpio.o \
./Inc/stm32f4xx_i2c.o \
./Inc/stm32f4xx_rcc.o \
./Inc/stm32f4xx_tim.o \
./Inc/stm32f4xx_usart.o 

C_DEPS += \
./Inc/misc.d \
./Inc/stm32f4xx_dcmi.d \
./Inc/stm32f4xx_dma.d \
./Inc/stm32f4xx_gpio.d \
./Inc/stm32f4xx_i2c.d \
./Inc/stm32f4xx_rcc.d \
./Inc/stm32f4xx_tim.d \
./Inc/stm32f4xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o Inc/%.su Inc/%.cyclo: ../Inc/%.c Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DSTM32F40_41xxx -c -I../Inc -I"C:/Users/Muhammad Asif/Desktop/Embedded Files/my_workspace/stm32_examples/Libraries/CMSIS/Include" -I"C:/Users/Muhammad Asif/Desktop/Embedded Files/my_workspace/stm32_examples/Libraries/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Muhammad Asif/Desktop/Embedded Files/my_workspace/stm32_examples/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/Muhammad Asif/Desktop/Embedded Files/my_workspace/stm32_examples/Libraries/STM32F4xx_StdPeriph_Driver/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Inc

clean-Inc:
	-$(RM) ./Inc/misc.cyclo ./Inc/misc.d ./Inc/misc.o ./Inc/misc.su ./Inc/stm32f4xx_dcmi.cyclo ./Inc/stm32f4xx_dcmi.d ./Inc/stm32f4xx_dcmi.o ./Inc/stm32f4xx_dcmi.su ./Inc/stm32f4xx_dma.cyclo ./Inc/stm32f4xx_dma.d ./Inc/stm32f4xx_dma.o ./Inc/stm32f4xx_dma.su ./Inc/stm32f4xx_gpio.cyclo ./Inc/stm32f4xx_gpio.d ./Inc/stm32f4xx_gpio.o ./Inc/stm32f4xx_gpio.su ./Inc/stm32f4xx_i2c.cyclo ./Inc/stm32f4xx_i2c.d ./Inc/stm32f4xx_i2c.o ./Inc/stm32f4xx_i2c.su ./Inc/stm32f4xx_rcc.cyclo ./Inc/stm32f4xx_rcc.d ./Inc/stm32f4xx_rcc.o ./Inc/stm32f4xx_rcc.su ./Inc/stm32f4xx_tim.cyclo ./Inc/stm32f4xx_tim.d ./Inc/stm32f4xx_tim.o ./Inc/stm32f4xx_tim.su ./Inc/stm32f4xx_usart.cyclo ./Inc/stm32f4xx_usart.d ./Inc/stm32f4xx_usart.o ./Inc/stm32f4xx_usart.su

.PHONY: clean-Inc

