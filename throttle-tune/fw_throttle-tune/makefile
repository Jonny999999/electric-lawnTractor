# Project settings
MCU = atmega8
#MCU = atmega328p

# Clock frequency.
# Possible values are 8000000UL, 4000000UL, 2000000UL, 1000000UL
F_CPU = 8000000UL

BOARD_TYPE = ethernet

BUILD_DIR = build

TARGET = $(BUILD_DIR)/main

FORMAT = ihex

# Project files
PROJECT_PATH = .
PROJECT_OBJS = main.o
#PROJECT_OBJS = main.o display.o usart.o
#----------------------------------------------------------
# Programmer settings
AVRDUDE_PROGRAMMER = usbasp
AVRDUDE_PORT = /dev/ttyUSB0
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_FLAGS = -p $(MCU) -c $(AVRDUDE_PROGRAMMER)

# configure fuses for ATMEGA8 internal oscillator:
ifeq ($(F_CPU), 8000000UL)
FUSE_LOW = 0b11100100:m
endif
ifeq ($(F_CPU), 4000000UL)
FUSE_LOW = 0b11100011:m
endif
ifeq ($(F_CPU), 2000000UL)
FUSE_LOW = 0b11100010:m
endif
ifeq ($(F_CPU), 1000000UL)
FUSE_LOW = 0b11100001:m
endif

ifeq ($(FUSE_LOW),)
$(error invalid F_CPU settings)
endif

AVRDUDE_WRITE_FUSES = -U lfuse:w:$(FUSE_LOW)
# -F forces programming without checking device
#  identifier.
#  This is, because the arduino bootloader does
#  not conform to the stk500v1 standard when sending
#  the device identifier. Hence the device ID check
#  always will fail.
#----------------------------------------------------------
# compiler suite
CC=avr-gcc
CPP=avr-g++
# AR=avr-ar # we currently do not package the libraries in .a files, so this is not needed
OBJCOPY = avr-objcopy
SIZE = avr-size
AVRDUDE = avrdude
#----------------------------------------------------------
# Linker settings
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
# --gc-sections will allow the linker to remove unused sections
LDFLAGS += -Wl,--gc-sections
#----------------------------------------------------------
# Compiler settings
# -mmcu selects the AVR chip type -> specifies instructionset etc.
# -DF_CPU defines the F_CPU preprocessor makro
# -ffunction-sections -fdata-sections are needed to place
#  functions in separate sections, to be able to remove
#  unneeded functions during linking
C_FLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) $(INCLUDE_FLAGS) -Os
C_FLAGS += -ffunction-sections -fdata-sections



# Accumulate all paths to header files
INCLUDE_FLAGS = -I$(ARDUINO_CORE_PATH)
INCLUDE_FLAGS += -I$(ARDUINO_VARIANT_PATH)
INCLUDE_FLAGS += -I$(ETHERNET_LIBRARY_PATH)
INCLUDE_FLAGS += -I$(ETHERNET_UTIL_LIBRARY_PATH)
INCLUDE_FLAGS += -I$(SPI_LIBRARY_PATH)
INCLUDE_FLAGS += -I$(ARDUINO_CORE_LIBC_PATH)
#----------------------------------------------------------

# generate lists of output files (in build dir)
PROJECT_OBJS_FIN=$(addprefix $(BUILD_DIR)/,$(PROJECT_OBJS))

#--------------------------------------------------------------
# rules:
all: directory build size

directory:
	mkdir -p build

build: elf hex

# upload the program to the microcontroller flash:
upload:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
fuse:
	$(info lfuse: $(FUSE_LOW))
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES)

# analyze compiled program size:
size:
	$(SIZE) $(TARGET).elf

elf: $(TARGET).elf
hex: $(TARGET).hex

# link everything together:
$(TARGET).elf: project
	$(CC) $(C_FLAGS) $(PROJECT_OBJS_FIN) -o $(TARGET).elf $(LDFLAGS)

# convert elf to hex file for programmer:
# exclude eeprom section, in case it exists
# as we do not want to write this to flash
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

clean:
	rm -rv $(BUILD_DIR)

project: $(PROJECT_OBJS_FIN)

# build Project files
$(PROJECT_OBJS_FIN): %.o: $(patsubst %.o,%.c,$(PROJECT_OBJS))
	$(CC) -c $(C_FLAGS) $(PROJECT_PATH)/$(subst $(BUILD_DIR)/,,$*).c -o $*.o
