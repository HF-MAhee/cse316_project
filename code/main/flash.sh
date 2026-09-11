#!/bin/bash

# 1. Validate Input
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <source_file.c>"
    echo "Example: $0 main.c"
    exit 1
fi

INPUT_FILE=$1

if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: File '$INPUT_FILE' not found."
    exit 1
fi

# 2. Configuration Variables
MCU="atmega32"
F_CPU="16000000UL"
PROGRAMMER="usbasp" # -B flag removed to prevent firmware SCK errors

# Extract filename without the .c extension (e.g., 'main.c' becomes 'main')
TARGET=$(basename "$INPUT_FILE" .c)

# 3. Compile C to ELF
echo "--- Compiling $INPUT_FILE ---"
avr-gcc -Os -DF_CPU=$F_CPU -mmcu=$MCU -Wall -o "$TARGET.elf" "$INPUT_FILE"

# Check if compilation succeeded
if [ $? -ne 0 ]; then
    echo "Error: Compilation failed. Aborting flash."
    exit 1
fi

# 4. Extract IHEX from ELF
echo "--- Generating $TARGET.hex ---"
avr-objcopy -O ihex -R .eeprom "$TARGET.elf" "$TARGET.hex"

# 5. Flash to ATmega32
echo "--- Flashing to ATmega32 ---"
avrdude -c $PROGRAMMER -p $MCU -U flash:w:"$TARGET.hex":i

# 6. Final Status Check
if [ $? -eq 0 ]; then
    echo "--- Success: $TARGET.hex successfully flashed! ---"
else
    echo "--- Error: avrdude failed to flash the chip. Check wiring and power. ---"
    exit 1
fi