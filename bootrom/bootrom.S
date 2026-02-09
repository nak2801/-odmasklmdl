/*******************************************************************************
 * Bootrom for FU1 MCU Design
 * 
 * Supports 4 boot modes via GPIO0[1:0]:
 *   0b00 - QSPI Flash Boot (XIP)
 *   0b01 - UART Boot
 *   0b10 - SWD Debug Boot
 *   0b11 - SDIO Boot (SD Card)
 *
 * SDIO Controller Base: 0x48000000
 * Register Map (Wishbone interface):
 *   +0x00: CMD   - Command register
 *   +0x04: ARG   - Argument register
 *   +0x08: FIFOA - Data FIFO A
 *   +0x0C: FIFOB - Data FIFO B
 *   +0x10: PHY   - PHY control
 *
 * SD Card Commands used:
 *   CMD0  (0) - GO_IDLE_STATE
 *   CMD8  (8) - SEND_IF_COND
 *   ACMD41(41)- SD_SEND_OP_COND (requires CMD55 first)
 *   CMD2  (2) - ALL_SEND_CID
 *   CMD3  (3) - SEND_RELATIVE_ADDR
 *   CMD7  (7) - SELECT_CARD
 *   CMD17 (17)- READ_SINGLE_BLOCK
 *
 ******************************************************************************/

.syntax unified
.arch armv6-m

/*============================================================================
 * Memory Map & Constants
 *============================================================================*/
.equ GPIO_0,            0x40010000
.equ GPIO0_DATA_REG,    (GPIO_0 + 0x000)
.equ GPIO0_OUTENSET,    (GPIO_0 + 0x010)
.equ GPIO0_ALTFUNC,     (GPIO_0 + 0x018)

.equ SRAM_ADDR,         0x20000000
.equ STACK_TOP,         0x20010000

/* UART0 Registers */
.equ UART_BASE,         0x40004000
.equ UART_DATA,         (UART_BASE + 0x00)
.equ UART_STAT,         (UART_BASE + 0x04)
.equ UART_CTRL,         (UART_BASE + 0x08)
.equ UART_BAUD,         (UART_BASE + 0x10)

/* QSPI XIP Address */
.equ QSPI_XIP_BASE,     0x10000000

/* SDIO Controller Registers */
.equ SDIO_BASE,         0x48000000
.equ SDIO_CMD,          (SDIO_BASE + 0x00)
.equ SDIO_ARG,          (SDIO_BASE + 0x04)
.equ SDIO_FIFOA,        (SDIO_BASE + 0x08)
.equ SDIO_FIFOB,        (SDIO_BASE + 0x0C)
.equ SDIO_PHY,          (SDIO_BASE + 0x10)

/* UART Boot Protocol */
.equ MAGIC_VALUE,       0xF1F1F1F1
.equ MAX_STAGE1_SZ,     0x00008000

/* SD Card Constants */
.equ SD_BLOCK_SIZE,     512
.equ SD_BOOT_SECTOR,    1           /* Boot image starts at sector 1 */

/* SDIO Command bits (based on sdwb.v) */
/* CMD register format:
 * [5:0]   - Command index (0-63)
 * [6]     - Command prefix (1=valid command)
 * [9:8]   - Response type: 00=none, 10=R2, 11=R1B
 * [10]    - FIFO write (write to SD card)
 * [11]    - Use FIFO
 * [12]    - FIFO ID (A or B)
 * [13]    - Use DMA
 * [15]    - Error bit (write 1 to clear)
 */
.equ CMD_PREFIX,        0x0040      /* Valid command prefix */
.equ CMD_R1_RESP,       0x0100      /* Expect R1 response */
.equ CMD_R2_RESP,       0x0200      /* Expect R2 response */
.equ CMD_USE_FIFO,      0x0800      /* Use FIFO for data */
.equ CMD_READ_DATA,     0x0900      /* Read data (USE_FIFO + R1) */

/* Standard SD Commands */
.equ SD_CMD0,           (0  | CMD_PREFIX)                    /* GO_IDLE */
.equ SD_CMD8,           (8  | CMD_PREFIX | CMD_R1_RESP)      /* SEND_IF_COND */
.equ SD_CMD55,          (55 | CMD_PREFIX | CMD_R1_RESP)      /* APP_CMD */
.equ SD_ACMD41,         (41 | CMD_PREFIX | CMD_R1_RESP)      /* SD_SEND_OP_COND */
.equ SD_CMD2,           (2  | CMD_PREFIX | CMD_R2_RESP)      /* ALL_SEND_CID */
.equ SD_CMD3,           (3  | CMD_PREFIX | CMD_R1_RESP)      /* SEND_RCA */
.equ SD_CMD7,           (7  | CMD_PREFIX | CMD_R1_RESP)      /* SELECT_CARD */
.equ SD_CMD17,          (17 | CMD_PREFIX | CMD_READ_DATA)    /* READ_SINGLE_BLOCK */

/*============================================================================
 * Vector Table
 *============================================================================*/
.section .isr_vector
.align 2
.globl __isr_vector
__isr_vector:
    .long   STACK_TOP               /* Top of Stack (0x0000) */
    .long   Reset_Handler           /* Reset Handler (0x0004) */
    .size   __isr_vector, . - __isr_vector

/*============================================================================
 * Reset Handler
 *============================================================================*/
    .text
    .thumb
    .thumb_func
    .align 2
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:

/*----------------------------------------------------------------------------
 * boot_mode: Read GPIO0[1:0] to determine boot mode
 *----------------------------------------------------------------------------*/
boot_mode:
    /* Configure GPIO_0[0] and GPIO_0[1] as inputs */
    ldr     r0, =GPIO0_OUTENSET
    ldr     r1, =GPIO0_ALTFUNC
    ldr     r2, =0xFFFFFFFC         /* All pins output except [1:0] */
    str     r2, [r0]
    str     r2, [r1]

    /* Read GPIO0 data register */
    ldr     r0, =GPIO0_DATA_REG
    ldr     r1, [r0]
    movs    r2, #0x03               /* Mask bits [1:0] */
    ands    r1, r2

    /* Branch based on boot mode */
    cmp     r1, #0
    beq     boot_flash              /* 00: QSPI Flash boot */

    cmp     r1, #1
    beq     boot_uart               /* 01: UART boot */

    cmp     r1, #2
    beq     boot_swd                /* 10: SWD debug boot */

    cmp     r1, #3
    beq     boot_sdio               /* 11: SDIO boot */

    b       boot_flash              /* Default: Flash boot */

/*============================================================================
 * boot_flash: Boot from QSPI XIP Flash
 *============================================================================*/
boot_flash:
    ldr     r0, =QSPI_XIP_BASE
    ldr     r1, [r0]                /* Load MSP from flash */
    ldr     r0, [r0, #4]            /* Load Reset vector from flash */
    msr     msp, r1
    bx      r0                      /* Jump to application */

/*============================================================================
 * boot_swd: Debug boot - halt and wait for debugger
 *============================================================================*/
boot_swd:
    bkpt    #0                      /* Breakpoint for debugger */
    b       boot_swd                /* Loop if no debugger attached */

/*============================================================================
 * boot_uart: Boot from UART (receive binary via serial)
 * Protocol: MAGIC(4B) + SIZE(4B) + DATA(SIZE bytes)
 *============================================================================*/
boot_uart:
    /* Configure UART: 115200 baud, 8N1 */
    ldr     r0, =UART_CTRL
    movs    r4, #0
    str     r4, [r0]                /* Disable UART */

    ldr     r0, =UART_BAUD
    ldr     r4, =278                /* 32MHz / 115200 ≈ 278 */
    str     r4, [r0]

    ldr     r0, =UART_CTRL
    movs    r4, #0x02               /* Enable RX only */
    str     r4, [r0]

    /* Read MAGIC word */
    bl      uart_read_word
    ldr     r3, =MAGIC_VALUE
    cmp     r5, r3
    beq     uart_magic_ok
    b       boot_fail
uart_magic_ok:

    /* Read SIZE word */
    bl      uart_read_word
    cmp     r5, #0
    bne     uart_size_nz
    b       boot_fail
uart_size_nz:
    ldr     r4, =MAX_STAGE1_SZ
    cmp     r5, r4
    bls     uart_size_ok
    b       boot_fail
uart_size_ok:

    /* Copy data to SRAM */
    mov     r6, r5                  /* r6 = byte count */
    ldr     r7, =SRAM_ADDR          /* r7 = destination */

uart_copy_loop:
    ldr     r0, =UART_STAT
uart_wait_rx:
    ldr     r4, [r0]
    movs    r3, #0x02               /* RX data available bit */
    tst     r4, r3
    beq     uart_wait_rx

    ldr     r0, =UART_DATA
    ldr     r4, [r0]
    movs    r3, #0xFF
    ands    r4, r3                  /* Mask to 8 bits */
    strb    r4, [r7]                /* Store byte to SRAM */
    adds    r7, #1
    subs    r6, #1
    bne     uart_copy_loop

    /* Jump to loaded application */
    ldr     r0, =SRAM_ADDR
    ldr     r1, [r0]                /* Load MSP */
    ldr     r0, [r0, #4]            /* Load Reset vector */
    msr     msp, r1
    bx      r0

/*----------------------------------------------------------------------------
 * uart_read_word: Read 4 bytes from UART into r5 (big-endian)
 *----------------------------------------------------------------------------*/
uart_read_word:
    push    {lr}
    movs    r5, #0
    movs    r6, #4                  /* 4 bytes to read */

uart_word_loop:
    ldr     r0, =UART_STAT
uart_word_wait:
    ldr     r4, [r0]
    movs    r3, #0x02
    tst     r4, r3
    beq     uart_word_wait

    ldr     r0, =UART_DATA
    ldr     r4, [r0]
    movs    r3, #0xFF
    ands    r4, r3
    lsls    r5, r5, #8
    orrs    r5, r4
    subs    r6, #1
    bne     uart_word_loop

    pop     {pc}

/*============================================================================
 * boot_sdio: Boot from SD Card via SDIO Controller
 * 
 * Initialization sequence:
 * 1. Reset PHY (set slow clock ~400kHz)
 * 2. Send CMD0 (GO_IDLE_STATE)
 * 3. Send CMD8 (SEND_IF_COND) - check voltage
 * 4. Send ACMD41 (SD_SEND_OP_COND) until ready
 * 5. Send CMD2 (ALL_SEND_CID)
 * 6. Send CMD3 (SEND_RELATIVE_ADDR) - get RCA
 * 7. Send CMD7 (SELECT_CARD)
 * 8. Read boot sectors using CMD17
 * 9. Jump to loaded application
 *============================================================================*/
boot_sdio:
    /* ---- Step 1: Initialize SDIO PHY ---- */
    /* Set PHY: 1-bit mode, slow clock for init (~400kHz from 50MHz) */
    /* PHY register: bits[7:0]=clock divider, bit[8]=DDR mode */
    ldr     r0, =SDIO_PHY
    ldr     r1, =0x0000007F         /* Clock divider = 127 (slow clock) */
    str     r1, [r0]

    /* Small delay for clock to stabilize */
    ldr     r2, =0x1000
sdio_delay1:
    subs    r2, #1
    bne     sdio_delay1

    /* ---- Step 2: Send CMD0 (GO_IDLE_STATE) ---- */
    ldr     r0, =SDIO_ARG
    movs    r1, #0
    str     r1, [r0]                /* Argument = 0 */

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD0
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Delay after CMD0 */
    ldr     r2, =0x1000
sdio_delay2:
    subs    r2, #1
    bne     sdio_delay2

    /* ---- Step 3: Send CMD8 (SEND_IF_COND) ---- */
    /* Argument: 0x000001AA (voltage=2.7-3.6V, check pattern=0xAA) */
    ldr     r0, =SDIO_ARG
    ldr     r1, =0x000001AA
    str     r1, [r0]

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD8
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Check response (should echo back 0x1AA) */
    ldr     r0, =SDIO_ARG
    ldr     r1, [r0]
    ldr     r2, =0x000001AA
    ands    r1, r2
    cmp     r1, r2
    beq     sdio_cmd8_ok
    b       boot_fail               /* Card doesn't support voltage */
sdio_cmd8_ok:

    /* ---- Step 4: Send ACMD41 until card is ready ---- */
    movs    r4, #100                /* Max retries */

acmd41_loop:
    /* Send CMD55 (APP_CMD) first */
    ldr     r0, =SDIO_ARG
    movs    r1, #0                  /* RCA = 0 during init */
    str     r1, [r0]

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD55
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Send ACMD41 */
    /* Argument: HCS=1 (High Capacity Support), voltage window */
    ldr     r0, =SDIO_ARG
    ldr     r1, =0x40300000         /* HCS=1, 3.2-3.4V */
    str     r1, [r0]

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_ACMD41
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Check if card is ready (busy bit cleared) */
    ldr     r0, =SDIO_ARG           /* Read OCR response */
    ldr     r1, [r0]
    movs    r2, #0x80
    lsls    r2, r2, #24             /* r2 = 0x80000000 (ready bit) */
    tst     r1, r2
    bne     acmd41_done

    /* Not ready, retry */
    subs    r4, #1
    bne     acmd41_retry
    b       boot_fail               /* Timeout */
acmd41_retry:

    /* Small delay between retries */
    ldr     r2, =0x2000
acmd41_delay:
    subs    r2, #1
    bne     acmd41_delay
    b       acmd41_loop

acmd41_done:
    /* Save card capacity type (CCS bit) */
    movs    r2, #0x40
    lsls    r2, r2, #24             /* r2 = 0x40000000 (CCS bit) */
    tst     r1, r2
    bne     sd_hc_card
    movs    r4, #0                  /* Standard capacity: byte addressing */
    b       sd_continue
sd_hc_card:
    movs    r4, #1                  /* High capacity: block addressing */

sd_continue:
    push    {r4}                    /* Save card type on stack */

    /* ---- Step 5: Send CMD2 (ALL_SEND_CID) ---- */
    ldr     r0, =SDIO_ARG
    movs    r1, #0
    str     r1, [r0]

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD2
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* ---- Step 6: Send CMD3 (SEND_RELATIVE_ADDR) ---- */
    ldr     r0, =SDIO_ARG
    movs    r1, #0
    str     r1, [r0]

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD3
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Get RCA from response (bits [31:16]) */
    ldr     r0, =SDIO_ARG
    ldr     r6, [r0]                /* r6 = RCA in upper 16 bits */
    lsrs    r6, r6, #16
    lsls    r6, r6, #16             /* r6 = RCA << 16 */

    /* ---- Step 7: Send CMD7 (SELECT_CARD) ---- */
    ldr     r0, =SDIO_ARG
    str     r6, [r0]                /* Argument = RCA << 16 */

    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD7
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* ---- Switch to faster clock after init ---- */
    ldr     r0, =SDIO_PHY
    ldr     r1, =0x00000002         /* Fast clock divider */
    str     r1, [r0]

    /* ---- Step 8: Read boot image from SD card ---- */
    /* 
     * Boot image format at sector SD_BOOT_SECTOR:
     *   Offset 0: Stack pointer (MSP)
     *   Offset 4: Reset handler address
     *   Offset 8: Image size (in bytes)
     *   Offset 12+: Application code
     */
    pop     {r4}                    /* Restore card type (HC or SC) */
    
    ldr     r7, =SRAM_ADDR          /* Destination address */
    ldr     r5, =SD_BOOT_SECTOR     /* Start sector */

    /* Read first sector to get image size */
    bl      sdio_read_sector

    /* Parse header from SRAM */
    ldr     r0, =SRAM_ADDR
    ldr     r1, [r0, #8]            /* Image size */
    
    /* Validate size */
    cmp     r1, #0
    bne     sdio_size_nz
    b       boot_fail
sdio_size_nz:
    ldr     r2, =MAX_STAGE1_SZ
    cmp     r1, r2
    bls     sdio_size_ok
    b       boot_fail
sdio_size_ok:

    /* Calculate remaining sectors to read */
    ldr     r2, =SD_BLOCK_SIZE
    subs    r1, r2                  /* Subtract first sector */
    cmp     r1, #0
    ble     sdio_boot_jump          /* No more sectors needed */

    /* r1 = remaining bytes, calculate sectors */
    adds    r1, r2
    subs    r1, #1
    lsrs    r1, r1, #9              /* Divide by 512 (sector size) */

    /* Read remaining sectors */
    adds    r5, #1                  /* Next sector */
    adds    r7, r2                  /* Next destination (SRAM + 512) */

sdio_read_loop:
    cmp     r1, #0
    beq     sdio_boot_jump
    
    push    {r1}
    bl      sdio_read_sector
    pop     {r1}
    
    adds    r5, #1                  /* Next sector */
    ldr     r2, =SD_BLOCK_SIZE
    adds    r7, r2                  /* Next destination */
    subs    r1, #1
    b       sdio_read_loop

sdio_boot_jump:
    /* Jump to loaded application */
    ldr     r0, =SRAM_ADDR
    ldr     r1, [r0]                /* Load MSP */
    ldr     r0, [r0, #4]            /* Load Reset vector */
    msr     msp, r1
    bx      r0

/*----------------------------------------------------------------------------
 * sdio_read_sector: Read single 512-byte sector from SD card
 *   Input:  r4 = card type (0=SC, 1=HC)
 *           r5 = sector number
 *           r6 = RCA << 16
 *           r7 = destination address in SRAM
 *   Uses:   r0, r1, r2, r3
 *----------------------------------------------------------------------------*/
sdio_read_sector:
    push    {lr}

    /* Set up argument (address) */
    ldr     r0, =SDIO_ARG
    cmp     r4, #0
    bne     sdio_hc_addr
    /* Standard Capacity: byte address = sector * 512 */
    lsls    r1, r5, #9
    b       sdio_send_cmd17
sdio_hc_addr:
    /* High Capacity: sector address */
    mov     r1, r5
sdio_send_cmd17:
    str     r1, [r0]

    /* Send CMD17 (READ_SINGLE_BLOCK) */
    ldr     r0, =SDIO_CMD
    ldr     r1, =SD_CMD17
    str     r1, [r0]
    bl      sdio_wait_cmd

    /* Read 512 bytes from FIFO */
    ldr     r0, =SDIO_FIFOA
    mov     r1, r7                  /* Destination */
    ldr     r2, =128                /* 512 bytes / 4 bytes per word = 128 words */

sdio_fifo_read_loop:
    ldr     r3, [r0]                /* Read word from FIFO */
    str     r3, [r1]                /* Store to SRAM */
    adds    r1, #4
    subs    r2, #1
    bne     sdio_fifo_read_loop

    pop     {pc}

/*----------------------------------------------------------------------------
 * sdio_wait_cmd: Wait for SDIO command to complete
 *   Checks CMD register bit 15 (busy) and bit 14 (error)
 *----------------------------------------------------------------------------*/
sdio_wait_cmd:
    ldr     r0, =SDIO_CMD
sdio_wait_loop:
    ldr     r1, [r0]
    
    /* Check for error (bit 15) */
    ldr     r2, =0x8000             /* ERR_BIT = 15 */
    tst     r1, r2
    bne     boot_fail
    
    /* Check if busy (command in progress) */
    /* The CMD register indicates busy when command is processing */
    ldr     r2, =0x4000             /* Check done bit or similar */
    tst     r1, r2
    beq     sdio_wait_loop          /* Still busy */
    
    bx      lr

/*============================================================================
 * boot_fail: Infinite loop on error
 *============================================================================*/
boot_fail:
    b       boot_fail

    .pool
    .size Reset_Handler, . - Reset_Handler
    .end
