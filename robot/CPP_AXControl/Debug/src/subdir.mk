################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CPP_AXControl.cpp \
../src/Dynamixel.cpp \
../src/SerialPort.cpp \
../src/Utils.cpp 

OBJS += \
./src/CPP_AXControl.o \
./src/Dynamixel.o \
./src/SerialPort.o \
./src/Utils.o 

CPP_DEPS += \
./src/CPP_AXControl.d \
./src/Dynamixel.d \
./src/SerialPort.d \
./src/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


