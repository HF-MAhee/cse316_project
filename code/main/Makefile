MCU     = atmega32
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy

# ---------------------------------------------------------------------------
#  The two-run maze solver. One image, no build modes:
#
#      make            build it
#      make flash      build it and flash it (USBasp)
#      make test       host-side unit tests -- no hardware needed
#      make boot-test  boot the real image in the simavr simulator
# ---------------------------------------------------------------------------

# -MMD -MP generate a .d file of header dependencies per object, so editing
# config.h rebuilds everything that reads it. Every tunable lives in that one
# header; without this a "rebuilt" image could silently keep old constants.
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall -Wextra \
          -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections \
          -I. -MMD -MP
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

SRC = main.c solver.c wallmem.c drive.c turn.c heading.c sonar.c motors.c \
      mpu6050.c i2c.c timer.c panel.c power.c resetlog.c telemetry.c debug.c
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
	@avr-size --format=avr --mcu=$(MCU) $(TARGET).elf

flash: $(TARGET).hex
	avrdude -c usbasp -p m32 -U flash:w:$(TARGET).hex

# Host-side tests. Both compile the SAME source the firmware links against:
# wallmem.c against a model of the demo maze (explore -> collapse -> replay),
# and panel.c against shimmed pins and clock (debounce, short vs long press).
test:
	@cc -I. -Wall -Wextra -O1 -o /tmp/test_wallmem test/test_wallmem.c wallmem.c
	@/tmp/test_wallmem
	@cc -I. -Wall -Wextra -O1 -std=gnu99 -o /tmp/test_panel test/test_panel.c panel.c
	@/tmp/test_panel

# Boot the real image in simavr and check it gets as far as arming for run 1,
# and that no start-up text was lost to the 192-byte debug TX ring overflowing.
# This is the check that was missing when Power_Init() hung every build at
# boot: everything compiled, every host test passed, and not one byte ever
# left the chip. Compiling proves nothing about whether main() reaches its
# first line of output; running it does.
#
# Needs simavr (Linux: apt install simavr, macOS: brew install simavr). Skips
# cleanly without it rather than failing, since it is not part of the build.
boot-test: $(TARGET).elf
	@command -v simavr >/dev/null 2>&1 || { echo "boot-test: simavr not installed, skipping"; exit 0; }; \
	timeout 8 simavr -m $(MCU) -f 16000000 $(TARGET).elf > /tmp/boot.log 2>&1; \
	if ! grep -q "READY FOR RUN 1" /tmp/boot.log; then \
	  echo "boot-test: FAILED -- did not reach READY FOR RUN 1. Last output:"; \
	  sed 's/\x1b\[[0-9;]*m//g' /tmp/boot.log | tail -8; exit 1; \
	fi; \
	drop=$$(sed 's/\x1b\[[0-9;]*m//g; s/\.\.$$//' /tmp/boot.log | grep -m1 '^0,' | awk -F, '{print $$NF}'); \
	if [ "$$drop" != "0" ]; then \
	  echo "boot-test: FAILED -- $$drop debug bytes dropped during start-up (TX buffer overflow: add a Debug_Flush)"; \
	  exit 1; \
	fi; \
	echo "boot-test: boots, calibrates, arms for run 1, no debug bytes dropped"

clean:
	rm -f *.o *.d *.elf *.hex

-include $(OBJ:.o=.d)

.PHONY: all size flash test boot-test clean
