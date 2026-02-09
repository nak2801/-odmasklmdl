#******************************************************************************
# Makefile for FU1 Bootrom
#
# Builds bootrom.S into binary and hex files for simulation
#******************************************************************************

# Toolchain
CROSS_COMPILE ?= arm-none-eabi-
AS      = $(CROSS_COMPILE)as
LD      = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE    = $(CROSS_COMPILE)size

# Flags
ASFLAGS  = -mcpu=cortex-m0 -mthumb -g
LDFLAGS  = -T bootrom.ld -nostdlib -nostartfiles

# Output files
TARGET   = bootrom
OBJS     = bootrom.o

# Default target
all: $(TARGET).bin $(TARGET).hex $(TARGET).lst

# Assemble
%.o: %.S
	$(AS) $(ASFLAGS) -o $@ $<

# Link
$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS)
	$(SIZE) $@

# Generate binary
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Generate hex for Verilog $readmemh
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O verilog $< $@

# Generate listing
$(TARGET).lst: $(TARGET).elf
	$(OBJDUMP) -d -S $< > $@

# Generate 32-bit word hex file for cmsdk_mcu_rom
$(TARGET)_32.hex: $(TARGET).bin
	@echo "Generating 32-bit word hex file..."
	@hexdump -v -e '1/4 "%08X\n"' $< > $@

# Clean
clean:
	rm -f *.o *.elf *.bin *.hex *.lst

# Phony targets
.PHONY: all clean

# Info
info:
	@echo "FU1 Bootrom Build"
	@echo "================="
	@echo "Toolchain: $(CROSS_COMPILE)"
	@echo "Target:    $(TARGET)"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build all outputs"
	@echo "  clean   - Remove build files"
	@echo "  info    - Show this message"
