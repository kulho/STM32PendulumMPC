################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_stm32f407xx.S 

OBJS += \
./startup/startup_stm32f407xx.o 

S_UPPER_DEPS += \
./startup/startup_stm32f407xx.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -x assembler-with-cpp -DSTM32F407xx -DDEBUG -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/qpOASES/include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/kulho/eclipse-workspace/STM32F407/Inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


