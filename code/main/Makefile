MCU     = atmega32
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy
# -MMD -MP generate a .d file of header dependencies per object. Without them
# editing config.h rebuilt NOTHING -- every tunable in the project lives in that
# one header, so a "rebuilt" image could silently keep old constants. That is the
# stale-.o trap the project notes warn about; this closes it properly so
# `make clean` is no longer required after every header edit.
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall -Wextra \
          -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections \
          -MMD -MP
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

SRC = main.c timer.c i2c.c mpu6050.c heading.c motors.c sonar.c drive.c turn.c maze.c debug.c gyrodiag.c power.c
OBJ = $(SRC:.c=.o)
TARGET = agv

all: $(TARGET).hex size

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Read the Data line. On a 2 KB part it is the number that bites first, and a
# link that overflows it fails here rather than producing a broken image.
size: $(TARGET).elf
	avr-size --format=avr --mcu=$(MCU) $(TARGET).elf

flash: $(TARGET).hex
	avrdude -c usbasp -p m32 -U flash:w:$(TARGET).hex

clean:
	rm -f *.o *.d *.elf *.hex

-include $(OBJ:.o=.d)

.PHONY: all size flash clean
