MCU=atmega328p
F_CPU=16000000UL
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
TARGET=roger_rf_node

all: $(TARGET).hex

$(TARGET).elf: main.c rf_node_common.c
	$(CC) $(CFLAGS) -o $@ main.c

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

clean:
	rm -f *.elf *.hex
