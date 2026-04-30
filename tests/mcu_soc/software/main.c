// Jacquard mcu_soc/sky130 firmware.
//
// Trimmed-down variant of chipflow-examples mcu_soc's main.c (preserved
// alongside as `main.c.original`). The Jacquard test fixture's synthesis
// only kept ports for soc_uart_0, soc_gpio_0_gpio, soc_flash, and the
// JTAG interface — gpio_1, uart_1, motor PWM, user_spi_0/1/2, and i2c_0
// peripherals don't exist in the netlist, so the original firmware's CSR
// reads/writes to those addresses hang the bus.
//
// This variant exercises only what the fixture supports:
// - UART_0 boot banner + SoC info + Flash ID + Quad mode (autonomous
//   boot output, exactly what Jacquard already reproduced).
// - GPIO_0 read (echoes back what input.json drives).
// - UART_0 RX echo loop (lets `uart_0 tx` actions exercise the GPU
//   UART decoder + CPU UART RX driver round-trip).
//
// Rebuild via chipflow-examples/mcu_soc's doit task:
//   cp tests/mcu_soc/software/main.c \
//      ../ChipFlow/chipflow-examples/mcu_soc/design/software/main.c
//   (cd ../ChipFlow/chipflow-examples/mcu_soc && pdm run chipflow software)
//   cp ../ChipFlow/chipflow-examples/mcu_soc/build/software/software.bin \
//      tests/mcu_soc/software.bin

#include <stdint.h>
#include "generated/soc.h"

char uart_getch_block(volatile uart_regs_t *uart) {
    while (!(uart->rx.status & 0x1))
        ;
    return uart->rx.data;
}

void main() {
    uart_init(UART_0, 25000000/115200);

    puts("🐱: nyaa~!\r\n");

    puts("SoC type: ");
    puthex(SOC_ID->type);
    puts("\r\n");

    // SPI Flash config (the GPU has a SpiFlashModel; this exercises it)
    puts("Flash ID: ");
    puthex(spiflash_read_id(SPIFLASH));
    puts("\n");
    spiflash_set_qspi_flag(SPIFLASH);
    spiflash_set_quad_mode(SPIFLASH);
    puts("Quad mode\n");

    // Read GPIO_0 inputs and echo. Jacquard's GpioModel can drive these
    // via input.json `gpio_0 set "<bits>"` actions.
    puts("GPIO0: ");
    puthex(GPIO_0->input);
    puts("\n");

    // UART_0 echo loop. Each character received is echoed back. This
    // exercises both the GPU UART TX decoder (Jacquard captures bytes)
    // and the CPU UART RX driver (Jacquard's UartRxDriver drives the
    // design's RX line in response to `uart_0 tx` actions).
    puts("Echo: ");
    while (1) {
        char c = uart_getch_block(UART_0);
        putc(c);
        if (c == '\n' || c == '\r') {
            puts("\nEcho: ");
        }
    }
}
