#
# This Makefile supports compiling an avr program and uploading it to
# an avr chip. To compile, do `make', and to upload the program to the
# avr, do `make upload'.
# You will need to change the settings below to set it up properly. These
# are explained here.
#
# SOURCES: the cpp source files (e.g. main.cpp util.cpp)
# PORT: usb/serial device name (e.g. /dev/ttyUSB*)
# BAUD_RATE: the baud-rate of the serial line (e.g. 57600)
# MCU: the avr microcontroller, (e.g. atmega328p)
# F_CPU: the frequency of the cpu in Hz (e.g. 16000000)


# ========== Edit below here ========= #
SOURCES = main.c AVR-lib/timers.c AVR-lib/usart.c command.c AVR-lib/i2c_safe.c AVR-lib/adc.c AVR-lib/lib/pid.c log.c hardware.c process-control.c esp120.c
PORT = /dev/ttyUSB*
BAUD_RATE = 115200
MCU = atmega328p
F_CPU = 16000000
# ========== Edit above here ========= #

CC = avr-gcc
CXX = avr-g++
AR = avr-ar
OBJ_COPY = avr-objcopy
AVR_SIZE = avr-size
AVRDUDE = avrdude
AVRDUDE_PROGRAMMER = arduino
WORKDIR = .build
TARGET = project

CFLAGS = -c -g -Os -Wall -w -ffunction-sections -fdata-sections -mmcu=$(MCU) \
         -DF_CPU=$(F_CPU)L
CPPFLAGS = $(CFLAGS) -fno-exceptions
LDFLAGS = -Os -Wl,--gc-sections -mmcu=$(MCU) -lm
HEXFLAGS = -O ihex -R .eeprom

PROG_OBJ = $(addprefix $(WORKDIR)/, $(SOURCES:.c=.o))

all: $(WORKDIR) $(WORKDIR)/$(TARGET).hex

subdirs: $(WORKDIR)
$(WORKDIR):
	mkdir -p $@/AVR-lib/lib

# uploading the hex file to the AVR
upload: all
	@$(AVR_SIZE) $(WORKDIR)/$(TARGET).elf
	$(AVRDUDE) -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -P $(PORT) \
	-b $(BAUD_RATE) \
	-D -Uflash:w:$(WORKDIR)/$(TARGET).hex:i

# cleaning up
clean:
	rm -rf $(WORKDIR)

# compiling the program sources
$(WORKDIR)/%.o: %.c
	$(CXX) $(CFLAGS) $< -o $@

# linking the program
$(WORKDIR)/$(TARGET).elf: $(PROG_OBJ)
	$(CC) $(PROG_OBJ) -o $@ $(LDFLAGS)

# creating the hex file from the elf file (to be stored in the AVR's flash)
%.hex: %.elf
	$(OBJ_COPY) $(HEXFLAGS) $< $@

.PHONY: all subdirs upload clean
