ifeq ($(OS),Windows_NT)
SHELL = $(ComSpec)
RMDIR = rmdir /s /q
RM = del /q
mymkdir = if not exist "$1" mkdir "$1"
else
RMDIR = rm -fr
RM = rm -f
mymkdir = mkdir -p $1
endif

PROJECT_OBJS = \
	project/MPPT.cpp.o \
	project/commands.cpp.o \
	project/dcdc.cpp.o \
	project/light.cpp.o \
	project/relays.cpp.o \
	project/../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp.o \
	project/slave.cpp.o \
	project/ssr.cpp.o \

PLATFORM_CORE_OBJS = \
	core/abi.cpp.o \
	core/avr/dtostrf.c.o \
	core/cortex_handlers.c.o \
	core/delay.c.o \
	core/hooks.c.o \
	core/IPAddress.cpp.o \
	core/itoa.c.o \
	core/main.cpp.o \
	core/new.cpp.o \
	core/Print.cpp.o \
	core/pulse.c.o \
	core/pulse_asm.S.o \
	core/Reset.cpp.o \
	core/SERCOM.cpp.o \
	core/startup.c.o \
	core/Stream.cpp.o \
	core/Tone.cpp.o \
	core/Uart.cpp.o \
	core/USB/CDC.cpp.o \
	core/USB/PluggableUSB.cpp.o \
	core/USB/samd21_host.c.o \
	core/USB/USBCore.cpp.o \
	core/WInterrupts.c.o \
	core/wiring.c.o \
	core/wiring_analog.c.o \
	core/wiring_digital.c.o \
	core/wiring_private.c.o \
	core/wiring_shift.c.o \
	core/WMath.cpp.o \
	core/WString.cpp.o \

PLATFORM_VARIANT_OBJS = \
	variant/variant.cpp.o \

LIBRARIES_OBJS = \
	libraries/SPI/SPI.cpp.o \
	libraries/Wire/Wire.cpp.o \

TARGETS = \
	MPPT.hex \
	MPPT.bin \

all: $(TARGETS)

MPPT.hex: MPPT.elf
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objcopy" -O ihex -R .eeprom  "./MPPT.elf" "./MPPT.hex"

MPPT.bin: MPPT.elf
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objcopy" -O binary  "./MPPT.elf" "./MPPT.bin"

MPPT.elf: $(PROJECT_OBJS) $(LIBRARIES_OBJS) core.a
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc"  "-L." -Os -Wl,--gc-sections -save-temps  "-T/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero/linker_scripts/gcc/flash_with_bootloader.ld" "-Wl,-Map,./MPPT.map" --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -o "./MPPT.elf" $(PROJECT_OBJS) $(LIBRARIES_OBJS) -Wl,--start-group "-L/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Lib/GCC/" -larm_cortexM0l_math -lm "./core.a" -Wl,--end-group

core.a:	$(PLATFORM_CORE_OBJS) $(PLATFORM_VARIANT_OBJS)

clean:
	-$(RMDIR) project core variant libraries
	-$(RM) *.hex *.eep *.bin *.elf *.a *.ar *.d

size:
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-size" -A "./MPPT.elf"

project/MPPT.cpp.o: ../../MPPT.cpp project/MPPT.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/MPPT.cpp.d: ;

-include project/MPPT.cpp.d 

project/commands.cpp.o: ../../commands.cpp project/commands.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/commands.cpp.d: ;

-include project/commands.cpp.d 

project/dcdc.cpp.o: ../../dcdc.cpp project/dcdc.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/dcdc.cpp.d: ;

-include project/dcdc.cpp.d 

project/light.cpp.o: ../../light.cpp project/light.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/light.cpp.d: ;

-include project/light.cpp.d 

project/relays.cpp.o: ../../relays.cpp project/relays.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/relays.cpp.d: ;

-include project/relays.cpp.d 

project/../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp.o: ../../../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp project/../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp.d: ;

-include project/../../../Documents/Arduino/libraries/scpiparser/scpiparser.cpp.d 

project/slave.cpp.o: ../../slave.cpp project/slave.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/slave.cpp.d: ;

-include project/slave.cpp.d 

project/ssr.cpp.o: ../../ssr.cpp project/ssr.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

project/ssr.cpp.d: ;

-include project/ssr.cpp.d 


core/abi.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/abi.cpp core/abi.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/abi.cpp.d: ;

-include core/abi.cpp.d

core/avr/dtostrf.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/avr/dtostrf.c core/avr/dtostrf.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/avr/dtostrf.c.d: ;

-include core/avr/dtostrf.c.d

core/cortex_handlers.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/cortex_handlers.c core/cortex_handlers.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/cortex_handlers.c.d: ;

-include core/cortex_handlers.c.d

core/delay.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/delay.c core/delay.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/delay.c.d: ;

-include core/delay.c.d

core/hooks.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/hooks.c core/hooks.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/hooks.c.d: ;

-include core/hooks.c.d

core/IPAddress.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/IPAddress.cpp core/IPAddress.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/IPAddress.cpp.d: ;

-include core/IPAddress.cpp.d

core/itoa.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/itoa.c core/itoa.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/itoa.c.d: ;

-include core/itoa.c.d

core/main.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/main.cpp core/main.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/main.cpp.d: ;

-include core/main.cpp.d

core/new.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/new.cpp core/new.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/new.cpp.d: ;

-include core/new.cpp.d

core/Print.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/Print.cpp core/Print.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Print.cpp.d: ;

-include core/Print.cpp.d

core/pulse.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/pulse.c core/pulse.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/pulse.c.d: ;

-include core/pulse.c.d

core/pulse_asm.S.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/pulse_asm.S
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -c -g -x assembler-with-cpp -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Reset.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/Reset.cpp core/Reset.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Reset.cpp.d: ;

-include core/Reset.cpp.d

core/SERCOM.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/SERCOM.cpp core/SERCOM.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/SERCOM.cpp.d: ;

-include core/SERCOM.cpp.d

core/startup.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/startup.c core/startup.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/startup.c.d: ;

-include core/startup.c.d

core/Stream.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/Stream.cpp core/Stream.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Stream.cpp.d: ;

-include core/Stream.cpp.d

core/Tone.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/Tone.cpp core/Tone.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Tone.cpp.d: ;

-include core/Tone.cpp.d

core/Uart.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/Uart.cpp core/Uart.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/Uart.cpp.d: ;

-include core/Uart.cpp.d

core/USB/CDC.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/USB/CDC.cpp core/USB/CDC.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/USB/CDC.cpp.d: ;

-include core/USB/CDC.cpp.d

core/USB/PluggableUSB.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/USB/PluggableUSB.cpp core/USB/PluggableUSB.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/USB/PluggableUSB.cpp.d: ;

-include core/USB/PluggableUSB.cpp.d

core/USB/samd21_host.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/USB/samd21_host.c core/USB/samd21_host.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/USB/samd21_host.c.d: ;

-include core/USB/samd21_host.c.d

core/USB/USBCore.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/USB/USBCore.cpp core/USB/USBCore.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/USB/USBCore.cpp.d: ;

-include core/USB/USBCore.cpp.d

core/WInterrupts.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/WInterrupts.c core/WInterrupts.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/WInterrupts.c.d: ;

-include core/WInterrupts.c.d

core/wiring.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/wiring.c core/wiring.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/wiring.c.d: ;

-include core/wiring.c.d

core/wiring_analog.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/wiring_analog.c core/wiring_analog.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/wiring_analog.c.d: ;

-include core/wiring_analog.c.d

core/wiring_digital.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/wiring_digital.c core/wiring_digital.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/wiring_digital.c.d: ;

-include core/wiring_digital.c.d

core/wiring_private.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/wiring_private.c core/wiring_private.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/wiring_private.c.d: ;

-include core/wiring_private.c.d

core/wiring_shift.c.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/wiring_shift.c core/wiring_shift.c.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"
	
core/wiring_shift.c.d: ;

-include core/wiring_shift.c.d

core/WMath.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/WMath.cpp core/WMath.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/WMath.cpp.d: ;

-include core/WMath.cpp.d

core/WString.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino/WString.cpp core/WString.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

core/WString.cpp.d: ;

-include core/WString.cpp.d


variant/variant.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero/variant.cpp variant/variant.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs  "./core.a" "$@"

variant/variant.cpp.d: ;

-include variant/variant.cpp.d


libraries/SPI/SPI.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI/SPI.cpp libraries/SPI/SPI.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

libraries/SPI/SPI.cpp.d: ;

-include libraries/SPI/SPI.cpp.d

libraries/Wire/Wire.cpp.o: /Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire/Wire.cpp libraries/Wire/Wire.cpp.d
	@$(call mymkdir,$(dir $@))
	"/Users/normanpellet/.arduinocdt/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -mcpu=cortex-m0plus -mthumb -c -g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -MMD -DF_CPU=48000000L -DARDUINO=10608 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD  -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804d -DUSBCON '-DUSB_MANUFACTURER="Arduino LLC"' '-DUSB_PRODUCT="Arduino Zero"' "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS/4.5.0/CMSIS/Include/" "-I/Users/normanpellet/.arduinocdt/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/cores/arduino" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/variants/arduino_zero" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/SPI" -I"/Users/normanpellet/.arduinocdt/packages/arduino/hardware/samd/1.6.18/libraries/Wire" "$<" -o "$@"

libraries/Wire/Wire.cpp.d: ;

-include libraries/Wire/Wire.cpp.d

