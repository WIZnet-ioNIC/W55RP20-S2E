
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#define PLL_SYS_KHZ (133000UL)

#define BUF_LEN 1024

#define SPI_MASTER_WRITE_CMD 0xA0    // Master Write
#define SPI_MASTER_READ_LEN_CMD 0xB0 // Master Read Data Length
#define SPI_SLAVE_WRITE_LEN_CMD 0xB1 // Slave Write Data Length

#define SPI_DUMMY 0xFF
#define SPI_ACK 0x0A
#define SPI_NACK 0x0B
#define TIMEOUT_MS 2000

#define PICO_SPI_RX_PIN 4
#define PICO_SPI_SCK_PIN 2
#define PICO_SPI_TX_PIN 3
#define PICO_SPI_CSN_PIN 5
#define SPI_RECV_PIN 26

#define SPI_HW_CS 1

#define SUCCESS 0
#define ERR_NACK -1
#define ERR_TIMEOUT -2
#define ERR_INVALID_HEADER -3

static uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

static void RP2040_Init(void);
int data_send(uint8_t *data, uint16_t data_len);
int16_t data_read(uint8_t *data);
int check_ack_blocking(void);
int16_t read_data_len_blocking(void);
int16_t atcmd_get(uint8_t *data);
int atcmd_set(uint8_t *data);
int spi_read_with_timeout(uint8_t *data, size_t len, uint8_t _header);

void printbuf(uint8_t buf[], size_t len)
{
    int i;
    for (i = 0; i < len; ++i)
    {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16)
    {
        putchar('\n');
    }
}
volatile bool spi_read_triggered = false;

void spi_irq_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        spi_read_triggered = true;
    }
}

int main()
{
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
    gpio_pull_up(SPI_RECV_PIN);
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
    for (size_t i = 0; i < BUF_LEN; ++i)
    {
        out_buf[i] = (i % 10) + 0x30;
        if (!(i % 100))
            out_buf[i] = '\n';
    }

#if 0 // ATCMD test
    for (size_t i = 0; ; ++i) {
        atcmd_set("LI192.168.11.189\r\n");
        atcmd_get("LI\r\n");
        sleep_ms(1000);
    }
#endif

#if 0 // for test TCP_MIXED Client Mode
    data_send(out_buf, BUF_LEN);
    sleep_ms(1000);
#endif

    gpio_set_irq_enabled_with_callback(SPI_RECV_PIN, GPIO_IRQ_EDGE_FALL, true, &spi_irq_callback);

    while (1) {
        if (spi_read_triggered)
        {
            read_length = data_read(in_buf);
            data_send(in_buf, read_length);
            spi_read_triggered = false;
            //printf("%d ", read_length);
        } 
    }
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

int data_send(uint8_t *data, uint16_t data_len)
{
    uint8_t header[4];
    int ret = 0;
    header[0] = SPI_MASTER_WRITE_CMD;
    header[1] = (data_len) & 0xFF;
    header[2] = (data_len >> 8) & 0xFF;
    header[3] = SPI_DUMMY;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);

    ret = check_ack_blocking();
    if (ret < 0)
    {
        printf("<data send> ERROR: %d\n", ret);
        return 0;
    }
    spi_write_blocking(spi_default, data, data_len);
    ret = check_ack_blocking();
    if (ret < 0)
    {
        printf("<data send> ERROR: %d\n", ret);
        return 0;
    }
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
    return 1;
}

int16_t data_read(uint8_t *data)
{
    uint8_t header[4];
    int16_t length;

    header[0] = SPI_MASTER_READ_LEN_CMD;
    header[1] = SPI_DUMMY;
    header[2] = SPI_DUMMY;
    header[3] = SPI_DUMMY;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);
    length = read_data_len_blocking();
    // printf("length = %d\r\n", length);
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
    if (length > 0)
    {
        spi_read_blocking(spi_default, 0xFF, data, length);
        // printf("length = %d\r\n", length);
        // printbuf(data, length);
        while(gpio_get(SPI_RECV_PIN) == 0);
    }
    else if (length < 0)
    {
        printf("<data read> ERROR: %d\n", length);
        return ERR_TIMEOUT;
    }
    return length;
}

int check_ack_blocking(void)
{
    uint8_t header;
    uint8_t is_ack = 0;
    int ret = 0;

    ret = spi_read_with_timeout(&header, 1, SPI_ACK);

    if (ret == SUCCESS)
    {
        is_ack = 1;
    }
    else if (ret == ERR_NACK)
    {
        is_ack = 0;
    }
    else if (ret == ERR_TIMEOUT)
    {
        return ERR_TIMEOUT;
    }

    for (int i = 0; i < 3; i++)
    {
        if (spi_read_with_timeout(&header, 1, SPI_DUMMY) == ERR_TIMEOUT)
        {
            return ERR_TIMEOUT;
        }
    }

    return is_ack ? SUCCESS : ERR_NACK;
}

int16_t read_data_len_blocking(void)
{
    uint8_t header;
    uint16_t length;

    if (spi_read_with_timeout(&header, 1, SPI_SLAVE_WRITE_LEN_CMD) == ERR_TIMEOUT)
    {
        return ERR_TIMEOUT;
    }
    spi_read_blocking(spi_default, 0xFF, (uint8_t *)&length, 2);

    if (spi_read_with_timeout(&header, 1, SPI_DUMMY) == ERR_TIMEOUT)
    {
        return ERR_TIMEOUT;
    }
    return length;
}

int atcmd_set(uint8_t *data)
{
    uint8_t header[4];
    uint16_t data_len = strlen(data) - 2;
    int ret;

    header[0] = data[0];
    header[1] = data[1];
    header[2] = (data_len) & 0xFF;
    header[3] = (data_len >> 8) & 0xFF;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, header, 4);
    ret = check_ack_blocking();
    if (ret < 0)
    {
        printf("<AT CMD set> ERROR: %d\n", ret);
        return ret;
    }
    spi_write_blocking(spi_default, data + 2, data_len);
    ret = check_ack_blocking();
    if (ret < 0)
    {
        printf("<AT CMD set> ERROR: %d\n", ret);
        return ret;
    }
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
}

int16_t atcmd_get(uint8_t *data)
{
    uint8_t header[4];
    int16_t length;

#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 0);
#endif
    spi_write_blocking(spi_default, data, 4);
    while (gpio_get(SPI_RECV_PIN) == 1)
        ;
    length = read_data_len_blocking();
#if !SPI_HW_CS
    gpio_put(PICO_SPI_CSN_PIN, 1);
#endif
    if (length > 0)
    {
        spi_read_blocking(spi_default, 0xFF, in_buf, length);
        printf("length = %d\r\n", length);
        printbuf(in_buf, length);
        printf("%.*s\n", length, in_buf);
        while (gpio_get(SPI_RECV_PIN) == 0)
            ;
    }
    else if (length < 0)
    {
        printf("<AT CMD get> ERROR: %d\n", length);
        return ERR_TIMEOUT;
    }
    return length;
}
int spi_read_with_timeout(uint8_t *data, size_t len, uint8_t _header)
{

    absolute_time_t start_time = get_absolute_time();
    const int timeout_us = TIMEOUT_MS * 1000;

    while (absolute_time_diff_us(start_time, get_absolute_time()) < timeout_us)
    {
        spi_read_blocking(spi_default, 0xFF, data, len);

        if (*data == _header)
        {
            return SUCCESS;
        }
        else if (*data == SPI_NACK)
        {
            return ERR_NACK;
        }
    }
    printf("ERROR: Timeout occurred while waiting for 0x%02X\n", _header);
    return ERR_TIMEOUT;
}