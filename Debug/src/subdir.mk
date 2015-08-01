################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CRC.cpp \
../src/MotorController.cpp \
../src/MotorMessage.cpp \
../src/ubiquity_motor.cpp 

OBJS += \
./src/CRC.o \
./src/MotorController.o \
./src/MotorMessage.o \
./src/ubiquity_motor.o 

CPP_DEPS += \
./src/CRC.d \
./src/MotorController.d \
./src/MotorMessage.d \
./src/ubiquity_motor.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


