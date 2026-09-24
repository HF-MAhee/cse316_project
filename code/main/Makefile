MCU     = atmega32
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy

# ---------------------------------------------------------------------------
#  BUILD MODE SELECTION
#
#  Exactly one file from modes/ is compiled into the image. Pick it here or on
#  the command line:
#
#      make                      build the default mode below
#      make MODE=deadend         build modes/mode_deadend.c
#      make MODE=turndbg flash   build that mode and flash it
#      make list-modes           show every mode with what it does
#
#  This replaced a #define BUILD_MODE in main.c, which meant editing a source
#  file to change what you were flashing -- easy to forget, and invisible in
#  the build output. The mode now appears in `make` output and in the .elf name
#  is checked below, so flashing the wrong image is much harder to do silently.
# ---------------------------------------------------------------------------
MODE ?= maze

MODE_SRC = modes/mode_$(MODE).c
ifeq ($(wildcard $(MODE_SRC)),)
$(error Unknown MODE '$(MODE)'. Run 'make list-modes' to see the options)
endif

# -MMD -MP generate a .d file of header dependencies per object. Without them
# editing config.h rebuilt NOTHING -- every tunable in the project lives in that
# one header, so a "rebuilt" image could silently keep old constants. That is the
# stale-.o trap the project notes warn about; this closes it properly so
# `make clean` is no longer required after every header edit.
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall -Wextra \
          -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections \
          -I. -MMD -MP -DBUILD_MODE_NAME='"$(MODE)"'
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

# Everything here is mode-independent: drivers, control, and the runner.
CORE = main.c timer.c i2c.c mpu6050.c heading.c motors.c sonar.c drive.c \
       turn.c maze.c debug.c gyrodiag.c power.c resetlog.c telemetry.c \
       wallmem.c panel.c

SRC = $(CORE) $(MODE_SRC)
OBJ = $(SRC:.c=.o)
TARGET = agv

all: banner $(TARGET).hex size

banner:
	@echo "=== building MODE=$(MODE)  ($(MODE_SRC)) ==="

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

# Re-state the mode at flash time too. The most expensive mistake with this
# project is flashing a mode you did not mean to and reading the resulting
# behaviour as a bug.
flash: $(TARGET).hex
	@echo "=== flashing MODE=$(MODE) ==="
	avrdude -c usbasp -p m32 -U flash:w:$(TARGET).hex

list-modes:
	@echo "Available modes (make MODE=<name>):"
	@echo "  sonar       sonar telemetry only, motors never move"
	@echo "  corridor    wall centring in a straight corridor"
	@echo "  turn        one 90 degree pivot, then halt"
	@echo "  maze        the full maze solver            [default]"
	@echo "  cone        sonar cone test, rotate by hand, motors off"
	@echo "  square      four timed legs + four turns, no corridor"
	@echo "  turndbg     repeated pivots with the full yaw-rate profile"
	@echo "  avoid       corridor drive -> obstacle -> right turn -> leg"
	@echo "  gyrodiag    gyro / I2C connection diagnostic, runs on a cold bus"
	@echo "  openavoid   open-space obstacle avoidance, no corridor"
	@echo "  deadend     dead-end 180, direction chosen from the nearer wall"
	@echo "  panel       LED + button test: press to blink, press again to stop"
	@echo "  wallmem     wall follower WITH MEMORY: explore, then replay the"
	@echo "              collapsed route (two separate power-ups, EEPROM)"

# Build every mode in turn. Catches the failure modularisation invites: a mode
# that stops compiling because a header it relied on is now only included
# somewhere else.
check-all:
	@fail=0; \
	for m in sonar corridor turn maze cone square turndbg avoid gyrodiag openavoid deadend wallmem panel; do \
	  $(MAKE) --no-print-directory clean >/dev/null; \
	  if $(MAKE) --no-print-directory MODE=$$m >/tmp/mk.$$m.log 2>&1; then \
	    printf "  %-11s %s\n" "$$m" "$$(grep Program /tmp/mk.$$m.log | awk '{print $$2" flash, "}')$$(grep Data /tmp/mk.$$m.log | awk '{print $$2" ram"}')"; \
	  else \
	    printf "  %-11s BUILD FAILED\n" "$$m"; grep -m3 -E 'error|undefined' /tmp/mk.$$m.log; fail=1; \
	  fi; \
	done; \
	$(MAKE) --no-print-directory clean >/dev/null; \
	if [ $$fail -ne 0 ]; then echo "check-all: FAILED"; exit 1; fi; \
	echo "check-all: all 13 modes build"

# Host-side tests. Both compile the SAME source the firmware links against:
# wallmem.c against a model of the demo maze (explore -> collapse -> replay),
# and panel.c against shimmed pins and clock (debounce, short vs long press).
# Neither needs hardware, so there is no excuse for not running them before a
# flash -- a button that double-fires costs a lab session to find on the bench.
test:
	@cc -I. -Wall -Wextra -O1 -o /tmp/test_wallmem test/test_wallmem.c wallmem.c
	@/tmp/test_wallmem
	@cc -I. -Wall -Wextra -O1 -std=gnu99 -o /tmp/test_panel test/test_panel.c panel.c
	@/tmp/test_panel

# Standalone LED check. Depends on NOTHING in this project -- not config.h, not
# the timer, not the USART. If this blinks and MODE=panel does not, the hardware
# is fine and the fault is in the firmware; if this does not blink, stop reading
# code and check the LED, the resistor, the pin and the power.
blink:
	$(CC) -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra \
	      -o blink_pb0.elf test/blink_pb0.c
	$(OBJCOPY) -O ihex -R .eeprom blink_pb0.elf blink_pb0.hex
	@avr-size --format=avr --mcu=$(MCU) blink_pb0.elf | grep -E "Program|Data"

blink-flash: blink
	@echo "=== flashing the standalone PB0 blink test ==="
	avrdude -c usbasp -p m32 -U flash:w:blink_pb0.hex

clean:
	rm -f *.o *.d modes/*.o modes/*.d *.elf *.hex

-include $(OBJ:.o=.d)

.PHONY: all banner size flash clean list-modes check-all test blink blink-flash
