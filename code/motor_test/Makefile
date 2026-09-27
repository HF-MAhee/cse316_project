MCU     = atmega32
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy

# ---------------------------------------------------------------------------
#  Standalone motor test. Pins and baud rate come from ../main/config.h, so
#  it drives exactly the hardware the maze firmware does.
#
#      make          build it
#      make flash    build it and flash it (USBasp)
#
#  Re-flash the maze firmware afterwards:  cd ../main && make flash
# ---------------------------------------------------------------------------

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall -Wextra \
          -funsigned-char -I../main
TARGET  = motor_test

all: $(TARGET).hex
	@avr-size --format=avr --mcu=$(MCU) $(TARGET).elf

$(TARGET).elf: motor_test.c ../main/config.h
	$(CC) $(CFLAGS) motor_test.c -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	avrdude -c usbasp -p m32 -U flash:w:$(TARGET).hex

clean:
	rm -f *.elf *.hex

.PHONY: all flash clean
