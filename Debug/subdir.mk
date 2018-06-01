################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../dcdc.c \
../light.c \
../relays.c \
../slave.c \
../ssr.c 

CPP_SRCS += \
../MataHari.cpp 

OBJS += \
./MataHari.o \
./dcdc.o \
./light.o \
./relays.o \
./slave.o \
./ssr.o 

C_DEPS += \
./dcdc.d \
./light.d \
./relays.d \
./slave.d \
./ssr.d 

CPP_DEPS += \
./MataHari.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.18/libraries/SPI -I/Users/normanpellet/Library/Arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.18/cores/arduino -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.18/libraries/Wire -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.17/libraries/SPI -I/Users/normanpellet/Library/Arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.17/variants/arduino_zero -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.17/cores/arduino -I/Users/normanpellet/Library/Arduino15/packages/arduino/hardware/samd/1.6.17/libraries/Wire -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


