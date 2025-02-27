
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

#define PLL_SYS_KHZ             (133000UL)

#define BUF_LEN         2048

#define SPI_MASTER_WRITE_CMD     0xA0  //Master Write
#define SPI_MASTER_READ_LEN_CMD  0xB0  //Master Read Data Length
#define SPI_SLAVE_WRITE_LEN_CMD  0xB1  //Slave Write Data Length

#define SPI_DUMMY       0xFF
#define SPI_ACK         0x0A
#define SPI_NACK        0x0B
#define TIMEOUT_MS      1000

#define PICO_SPI_RX_PIN    4
#define PICO_SPI_SCK_PIN   2
#define PICO_SPI_TX_PIN    3
#define PICO_SPI_CSN_PIN   5
#define SPI_RECV_PIN       26

#define SPI_HW_CS 1

static uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

static void RP2040_Init(void);
void data_send(uint8_t *data, uint16_t data_len);
uint16_t data_read(uint8_t *data);
int check_ack_blocking(void);
uint16_t read_data_len_blocking(void);
uint16_t atcmd_get(uint8_t *data);
void atcmd_set(uint8_t *data);

void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        putchar('\n');
    }
}

int main() {
    uint16_t read_length;
    // Enable UART so we can print
    RP2040_Init();
    stdio_init_all();

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    printf("spi clock = %d\r\n", spi_init(spi_default, PLL_SYS_KHZ * 1000 / 12));
    gpio_set_function(PICO_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_TX_PIN, GPIO_FUNC_SPI);
    
    gpio_init(SPI_RECV_PIN);
    gpio_set_dir(SPI_RECV_PIN, GPIO_IN);

#if SPI_HW_CS    
    gpio_set_function(PICO_SPI_CSN_PIN, GPIO_FUNC_SPI);
#else
    gpio_init(PICO_SPI_CSN_PIN);
    gpio_set_dir(PICO_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif

    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_SPI_RX_PIN, PICO_SPI_TX_PIN, PICO_SPI_SCK_PIN, PICO_SPI_CSN_PIN, GPIO_FUNC_SPI));

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        out_buf[i] = (i % 10) + 0x30;
        if(!(i % 100))
            out_buf[i] = '\n';
    }

#if 1   //ATCMD test
    for (size_t i = 0; ; ++i) {
        atcmd_get("LM\r\n");
    }
#endif

#if 0   //data test
#if 0   //for test TCP_MIXED Client Mode
    data_send(out_buf, BUF_LEN);
    sleep_ms(1000);
#endif

    for (size_t i = 0; ; ++i) {
        while(gpio_get(SPI_RECV_PIN) == 1);
        read_length = data_read(in_buf);
        while(gpio_get(SPI_RECV_PIN) == 0);
        if (read_length != 0)
            data_send(in_buf, read_length);
    }

#endif
}

static void RP2040_Init(void)
{
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
    sleep_ms(10);
}

void data_send(uint8_t *data, uint16_t data_len) {
    uint8_t header[4];

    header[0] = SPI_MASTER_WRITE_CMD;
    header[1] = (data_len) & 0xFF;
    header[2] = (data_len >> 8) & 0xFF;
    header[3] = SPI_DUMMY;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);
    check_ack_blocking();
    spi_write_blocking(spi_default, data, data_len);
    check_ack_blocking();
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
}

uint16_t data_read(uint8_t *data) {
    uint8_t header[4];
    uint16_t length;

    header[0] = SPI_MASTER_READ_LEN_CMD;
    header[1] = SPI_DUMMY;
    header[2] = SPI_DUMMY;
    header[3] = SPI_DUMMY;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);
    length = read_data_len_blocking();
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
    if (length != 0) {
        spi_read_blocking(spi_default, 0xFF, data, length);
        //printf("length = %d\r\n", length);
        //printbuf(data, length);
    }
    return length;
}

int check_ack_blocking(void) {
    uint8_t header;
    uint8_t is_ack;

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_ACK || header == SPI_NACK){
            break;
        }
    }

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_DUMMY){
            break;
        }
    }

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_DUMMY){
            break;
        }
    }

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_DUMMY){
            break;
        }
    }
}

uint16_t read_data_len_blocking(void) {
    uint8_t header;
    uint16_t length;

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_SLAVE_WRITE_LEN_CMD){
            break;
        }
    }
    spi_read_blocking(spi_default, 0xFF, (uint8_t *)&length, 2);

    while(1) {
        spi_read_blocking(spi_default, 0xFF, &header, 1);
        if(header == SPI_DUMMY){
            break;
        }
    }
    return length;
}


void atcmd_set(uint8_t *data) {
    uint8_t header[4];
    uint16_t data_len = strlen(data) - 2;

    header[0] = data[0];
    header[1] = data[1];
    header[2] = (data_len) & 0xFF;
    header[3] = (data_len >> 8) & 0xFF;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);
    check_ack_blocking();
    spi_write_blocking(spi_default, data+2, data_len);
    check_ack_blocking();
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
}

uint16_t atcmd_get(uint8_t *data) {
    uint8_t header[4];
    uint16_t length;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, data, 4);
    while(gpio_get(SPI_RECV_PIN) == 1);
    length = read_data_len_blocking();
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
    if (length != 0) {
        spi_read_blocking(spi_default, 0xFF, in_buf, length);
        printf("length = %d\r\n", length);
        printbuf(in_buf, length);
        printf("%.*s\n", length, in_buf);
        while(gpio_get(SPI_RECV_PIN) == 0);
    }
    return length;
}
