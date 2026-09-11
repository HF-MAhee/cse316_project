MCU     = atmega32
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall -Wextra \
          -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

SRC = main.c timer.c i2c.c mpu6050.c heading.c motors.c sonar.c drive.c turn.c maze.c debug.c
OBJ = $(SRC:.c=.o)
TARGET = agv

all: $(TARGET).hex size

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(TARGET).elf
	avr-size --format=avr --mcu=$(MCU) $(TARGET).elf

flash: $(TARGET).hex
	avrdude -c usbasp -p m32 -U flash:w:$(TARGET).hex

clean:
	rm -f *.o *.elf *.hex

.PHONY: all size flash clean
