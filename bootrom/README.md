# FU1 Bootrom with SDIO Support

## Overview

This bootrom supports 4 boot modes selected via GPIO0[1:0]:

| GPIO0[1:0] | Boot Mode | Description |
|------------|-----------|-------------|
| 00 | QSPI Flash | Execute-in-Place from QSPI Flash @ 0x10000000 |
| 01 | UART | Receive binary via UART0 @ 115200 baud |
| 10 | SWD | Halt and wait for debugger |
| 11 | SDIO | Load from SD Card via SDIO Controller |

## Memory Map

| Address | Size | Description |
|---------|------|-------------|
| 0x00000000 | 8KB | Mask ROM (Bootrom) |
| 0x10000000 | 16MB | QSPI Flash XIP |
| 0x20000000 | 64KB | SRAM |
| 0x40004000 | 4KB | UART0 |
| 0x40010000 | 4KB | GPIO0 |
| 0x48000000 | 4KB | SDIO Controller |

## Building

```bash
cd /root/fu1/systems/cortex_m0_mcu/rtl_sim/bootrom
make clean all
```

Output files:
- `bootrom.bin` - Raw binary
- `bootrom.hex` - Verilog hex format
- `bootrom.lst` - Disassembly listing
- `bootrom_32.hex` - 32-bit word hex for $readmemh

## Boot Protocols

### UART Boot (Mode 01)

Protocol:
1. Send magic word: `0xF1F1F1F1` (4 bytes, big-endian)
2. Send image size: N bytes (4 bytes, big-endian)
3. Send application binary: N bytes

The bootrom copies the binary to SRAM @ 0x20000000 and jumps to it.

### SDIO Boot (Mode 11)

SD Card Layout:
```
Sector 0: (Reserved - MBR)
Sector 1: Boot Image Header
  [0x00] Stack Pointer (MSP)
  [0x04] Reset Handler address
  [0x08] Image size (bytes)
  [0x0C] Application code starts...
Sector 2+: Rest of application code
```

The bootrom:
1. Initializes SDIO controller at 400kHz
2. Sends CMD0 → CMD8 → ACMD41 → CMD2 → CMD3 → CMD7
3. Switches to high-speed clock
4. Reads boot sectors starting at sector 1
5. Jumps to application in SRAM

### QSPI Flash Boot (Mode 00)

Reads MSP and Reset_Handler from QSPI XIP @ 0x10000000 and jumps directly.

## Creating SD Card Boot Image

```bash
# Compile your application to ELF
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -T app.ld -o app.elf app.c

# Extract binary
arm-none-eabi-objcopy -O binary app.elf app.bin

# Create boot image with header
# Header: [SP][Reset][Size][...code...]
python3 create_sdboot.py app.bin sdboot.img

# Write to SD card at sector 1
dd if=sdboot.img of=/dev/sdX bs=512 seek=1
```

## SDIO Controller Registers

| Offset | Name | Description |
|--------|------|-------------|
| 0x00 | CMD | Command register |
| 0x04 | ARG | Argument register / Response |
| 0x08 | FIFOA | Data FIFO A |
| 0x0C | FIFOB | Data FIFO B |
| 0x10 | PHY | PHY control (clock, width, DDR) |

## Simulation

To test SDIO boot in simulation, you need:
1. SD Card model (e.g., sdModel.v)
2. Testbench with GPIO0[1:0] = 2'b11
3. SD card image file with boot data

Example testbench setup:
```verilog
// Set SDIO boot mode
assign GPIO0_IN[1:0] = 2'b11;

// Instantiate SD card model
sdModel u_sdcard (
    .sdClk(sdio_clk),
    .cmd(sdio_cmd),
    .dat(sdio_dat)
);
```

## File Structure

```
bootrom/
├── bootrom.S       # Assembly source
├── bootrom.ld      # Linker script
├── Makefile        # Build system
└── README.md       # This file
```

## License

Based on ARM Cortex-M0 DesignStart, following ARM license terms.
