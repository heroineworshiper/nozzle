AVR_DIR := /root/arduino-1.8.15/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
# use the baud rate your arduino ISP was hacked to
AVR_DUDE := avrdude -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200
AVR_DUDE2 := avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b115200
AVR_CFLAGS := -O2 -mmcu=atmega328p
AVR_CFLAGS2 := -O2 -mmcu=atmega8
AVR_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x0000 -nostdlib
AVR_LFLAGS2 := -O2 -mmcu=atmega8 -Wl,--section-start=.text=0x0000 -nostdlib
BOOT_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x3800 -nostdlib

# inlet board
board0.hex: nozzle.c nozzle.h uart.c table.h
	$(AVR_GCC) $(AVR_CFLAGS) -o board0.o nozzle.c uart.c -DBOARD=0
	$(AVR_GCC) $(AVR_LFLAGS) -o board0.elf board0.o
	$(AVR_OBJCOPY) board0.elf board0.hex

# program atmega328 fuse.  page 283
nozzle_fuse:
	$(AVR_DUDE) -e -Ulock:w:0x3F:m -Uefuse:w:0x05:m -Uhfuse:w:0xD3:m -Ulfuse:w:0xE2:m 

# elbow 1 board
board1.hex: nozzle.c nozzle.h uart.c
	$(AVR_GCC) $(AVR_CFLAGS) -o board1.o nozzle.c uart.c -DBOARD=1
	$(AVR_GCC) $(AVR_LFLAGS) -o board1.elf board1.o
	$(AVR_OBJCOPY) board1.elf board1.hex

# elbow 2 board
board2.hex: nozzle.c nozzle.h uart.c
	$(AVR_GCC) $(AVR_CFLAGS) -o board2.o nozzle.c uart.c -DBOARD=2
	$(AVR_GCC) $(AVR_LFLAGS) -o board2.elf board2.o
	$(AVR_OBJCOPY) board2.elf board2.hex


# program atmega8 fuse.  page 216
#board1_fuse:
#	$(AVR_DUDE2) -e -Ulock:w:0x3F:m -Uhfuse:w:0xD9:m -Ulfuse:w:0xE4:m 
#	$(AVR_DUDE2) -e -Ulock:w:0x3F:m -Uhfuse:w:0xD9:m -Ulfuse:w:0xE4:m 


# program the controller with ISP
board0_isp: board0.hex
	$(AVR_DUDE) -Uflash:w:board0.hex:i -Ulock:w:0x0F:m

board1_isp: board1.hex
	$(AVR_DUDE) -Uflash:w:board1.hex:i -Ulock:w:0x0F:m

board2_isp: board2.hex
	$(AVR_DUDE) -Uflash:w:board2.hex:i -Ulock:w:0x0F:m

monitor: monitor.c
	gcc -O2 -o monitor monitor.c

tables2: tables2.c nozzle.h
	gcc -o tables2 tables2.c -lm

# program the failed bootloader
#bootloader_isp: bootloader.hex
#	$(AVR_DUDE) -Uflash:w:bootloader.hex:i -Ulock:w:0x0F:m

# failed bootloader
#bootloader.hex: bootloader.c avr_debug.c
#	$(AVR_GCC) $(AVR_CFLAGS) -o bootloader.o bootloader.c avr_debug.c
#	$(AVR_GCC) $(BOOT_LFLAGS) -o bootloader.elf bootloader.o
#	$(AVR_OBJCOPY) bootloader.elf bootloader.hex

# failed bootloader
#program: program.c
#	gcc -o program program.c

clean:
	rm -f *.hex *.elf *.o

