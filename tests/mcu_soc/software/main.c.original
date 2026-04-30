#include <stdint.h>
#include "generated/soc.h"

char uart_getch_block(volatile uart_regs_t *uart) {
    while (!(uart->rx.status & 0x1))
        ;
    return uart->rx.data;
}

void main() {
    uart_init(UART_0, 25000000/115200);
    uart_init(UART_1, 25000000/115200);

    puts("🐱: nyaa~!\r\n");

    puts("SoC type: ");
    puthex(SOC_ID->type);
    // This would make the golden reference output change every commit
    // puts(" ");
    // puthex(SOC_ID->version);
    puts("\r\n");

    // SPI Flash config
    puts("Flash ID: ");
    puthex(spiflash_read_id(SPIFLASH));
    puts("\n");
    spiflash_set_qspi_flag(SPIFLASH);
    spiflash_set_quad_mode(SPIFLASH);
    puts("Quad mode\n");

    //

    GPIO_1->mode = GPIO_PIN4_PUSH_PULL | GPIO_PIN5_PUSH_PULL \
                 | GPIO_PIN6_PUSH_PULL | GPIO_PIN7_PUSH_PULL;
    GPIO_1->output = 0x50;
    GPIO_1->setclr = GPIO_PIN4_CLEAR | GPIO_PIN5_SET \
                   | GPIO_PIN6_CLEAR | GPIO_PIN7_SET;
    GPIO_1->mode = GPIO_PIN4_INPUT_ONLY | GPIO_PIN5_INPUT_ONLY \
		 | GPIO_PIN6_INPUT_ONLY | GPIO_PIN7_INPUT_ONLY;

    uart_puts(UART_1, "ABCD");

    MOTOR_PWM0->numr = 0x1F;
    MOTOR_PWM0->denom = 0xFF;
    MOTOR_PWM0->conf = 0x3;

    MOTOR_PWM1->numr = 0x3F;
    MOTOR_PWM1->denom = 0xFF;
    MOTOR_PWM1->conf = 0x3;

    MOTOR_PWM9->numr = 0x7F;
    MOTOR_PWM9->denom = 0xFF;
    MOTOR_PWM9->conf = 0x3;

    /*
    PDM0->outval = 0xFF;
    PDM1->outval = 0x7F;
    PDM2->outval = 0x3F;

    PDM0->conf = 0x1;
    PDM1->conf = 0x1;
    PDM2->conf = 0x0;
    */

    puts("GPIO: ");
    puthex(GPIO_1->input);
    puts(" ");
    puthex(GPIO_1->input);
    puts("\n");

    puts("UART1: ");
    putc(uart_getch_block(UART_1));
    puts(" ");
    putc(uart_getch_block(UART_1));
    puts("\n");

    puts("SPI: ");

    spi_init(USER_SPI_0, 2);

    // test 8 bit transfer
    puthex(spi_xfer(USER_SPI_0, 0x5A, 8, true));
    puts(" ");
    // test an odd 21 bit transfer
    puthex(spi_xfer(USER_SPI_0, 0x123456, 21, true));
    puts("\n");

    i2c_init(I2C_0, 2);

    i2c_start(I2C_0);

    puts("I2C: ");
    putc(i2c_write(I2C_0, 0xA0) ? 'a' : 'n');
    putc(i2c_write(I2C_0, 0x33) ? 'a' : 'n');
    i2c_start(I2C_0);
    putc(i2c_write(I2C_0, 0xA1) ? 'a' : 'n');
    puts(" ");
    puthex(i2c_read(I2C_0));
    puts("\n");
    i2c_stop(I2C_0);

    while (1) {
        // // Listen for button presses
        // next_buttons = BTN_GPIO->in;
        // if ((next_buttons & 1U) && !(last_buttons & 1U))
        // 	puts("button 1 pressed!\n");
        // if ((next_buttons & 2U) && !(last_buttons & 2U))
        // 	puts("button 2 pressed!\n");
        // last_buttons = next_buttons;
    };
}
