################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../qpOASES/src/BLASReplacement.cpp \
../qpOASES/src/Bounds.cpp \
../qpOASES/src/Constraints.cpp \
../qpOASES/src/Flipper.cpp \
../qpOASES/src/Indexlist.cpp \
../qpOASES/src/LAPACKReplacement.cpp \
../qpOASES/src/Matrices.cpp \
../qpOASES/src/MessageHandling.cpp \
../qpOASES/src/OQPinterface.cpp \
../qpOASES/src/Options.cpp \
../qpOASES/src/QProblem.cpp \
../qpOASES/src/QProblemB.cpp \
../qpOASES/src/SQProblem.cpp \
../qpOASES/src/SQProblemSchur.cpp \
../qpOASES/src/SolutionAnalysis.cpp \
../qpOASES/src/SparseSolver.cpp \
../qpOASES/src/SubjectTo.cpp \
../qpOASES/src/Utils.cpp \
../qpOASES/src/qpOASES_wrapper.cpp 

OBJS += \
./qpOASES/src/BLASReplacement.o \
./qpOASES/src/Bounds.o \
./qpOASES/src/Constraints.o \
./qpOASES/src/Flipper.o \
./qpOASES/src/Indexlist.o \
./qpOASES/src/LAPACKReplacement.o \
./qpOASES/src/Matrices.o \
./qpOASES/src/MessageHandling.o \
./qpOASES/src/OQPinterface.o \
./qpOASES/src/Options.o \
./qpOASES/src/QProblem.o \
./qpOASES/src/QProblemB.o \
./qpOASES/src/SQProblem.o \
./qpOASES/src/SQProblemSchur.o \
./qpOASES/src/SolutionAnalysis.o \
./qpOASES/src/SparseSolver.o \
./qpOASES/src/SubjectTo.o \
./qpOASES/src/Utils.o \
./qpOASES/src/qpOASES_wrapper.o 

CPP_DEPS += \
./qpOASES/src/BLASReplacement.d \
./qpOASES/src/Bounds.d \
./qpOASES/src/Constraints.d \
./qpOASES/src/Flipper.d \
./qpOASES/src/Indexlist.d \
./qpOASES/src/LAPACKReplacement.d \
./qpOASES/src/Matrices.d \
./qpOASES/src/MessageHandling.d \
./qpOASES/src/OQPinterface.d \
./qpOASES/src/Options.d \
./qpOASES/src/QProblem.d \
./qpOASES/src/QProblemB.d \
./qpOASES/src/SQProblem.d \
./qpOASES/src/SQProblemSchur.d \
./qpOASES/src/SolutionAnalysis.d \
./qpOASES/src/SparseSolver.d \
./qpOASES/src/SubjectTo.d \
./qpOASES/src/Utils.d \
./qpOASES/src/qpOASES_wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
qpOASES/src/%.o: ../qpOASES/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DSTM32F407xx -DDEBUG -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/qpOASES/include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/CMSIS/Include" -I"/Users/kulho/eclipse-workspace/STM32F407/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/kulho/eclipse-workspace/STM32F407/Inc" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


