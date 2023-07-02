MCU=attiny402
MCU_ARCH=avrxmega3
CFLAGS=-mmcu=$(MCU) -Os -g -fshort-enums
PORT=/dev/ttyACM0

all: archive bin/main.hex src/main.S

archive: pcb/gerbers.zip

flash: bin/main.hex bin/eeprom.hex
	avrdude -v -c jtag2updi -P $(PORT) -p t402 -U flash:w:bin/main.hex -U eeprom:w:bin/eeprom.hex

bin/main.hex: bin/main.elf
	avr-objcopy -R .eeprom -O ihex bin/main.elf bin/main.hex

bin/main.elf: src/main.c
	avr-gcc $(CFLAGS) -o bin/main.elf src/main.c

src/main.S: src/main.c src/config.h
	avr-gcc $(CFLAGS) -g -S -o src/main.S src/main.c

clean:
	rm -f bin/main.elf bin/main.hex src/main.S

pcb/gerbers.zip: pcb/gerber/*
	cd pcb && zip gerbers.zip gerber/*