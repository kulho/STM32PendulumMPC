################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MPC_setup.c \
../Src/func.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/tiny_ekf.c 

OBJS += \
./Src/MPC_setup.o \
./Src/func.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/tiny_ekf.o 

C_DEPS += \
./Src/MPC_setup.d \
./Src/func.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/tiny_ekf.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DSTM32F407xx -DDEBUG -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/qpOASES/include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/kulho/eclipse-workspace/STM32F407/Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


