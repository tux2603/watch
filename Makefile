# CC=avr-gcc
# LD=avr-gcc
# AS=avr-as
# OBJCOPY=avr-objcopy

# MCU=attiny402
# MCU_ARCH=avrxmega3
# TARGET=main
# AVR_LIBC_DIR=/etc/avr-libc/avr/include
# CFLAGS=-g -Os -mmcu=attiny402 -I$(AVR_LIBC_DIR)
# ASFLAGS=-mmcu=avrxmega3 -mno-skip-bug

# all: bin/$(TARGET).hex

# bin/$(TARGET).hex: bin/main.elf
# 	$(OBJCOPY) -R eeprom -j .text -j .data -O ihex bin/main.elf bin/main.hex

## TAKE TWO

MCU=attiny402
MCU_ARCH=avrxmega3
TARGET=main

CFLAGS=-mmcu=$(MCU) -Os -g

CC=avr-gcc
OBJCOPY=avr-objcopy

all: bin/$(TARGET).hex src/$(TARGET).S

bin/$(TARGET).hex: bin/$(TARGET).elf
	$(OBJCOPY) -R .eeprom -O ihex bin/main.elf bin/main.hex

bin/$(TARGET).elf: src/$(TARGET).c
	$(CC) $(CFLAGS) -o bin/$(TARGET).elf src/$(TARGET).c

src/$(TARGET).S: src/$(TARGET).c
	$(CC) $(CFLAGS) -g -S -o src/$(TARGET).S src/$(TARGET).c

clean:
	rm -f bin/$(TARGET).elf bin/$(TARGET).hex src/$(TARGET).S