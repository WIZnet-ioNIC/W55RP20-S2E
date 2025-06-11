#ifndef SPIHANDLER_H_
#define SPIHANDLER_H_

#include <stdint.h>
#include "port_common.h"
#include "common.h"

#define SPI_ID spi0
#define SPI_MASTER_WRITE_CMD     0xA0  //Master Write
#define SPI_MASTER_READ_LEN_CMD  0xB0  //Master Read Data Length
#define SPI_SLAVE_WRITE_LEN_CMD  0xB1  //Slave Write Data Length

#define SPI_ACK                  0x0A
#define SPI_NACK                 0x0B
#define SPI_DUMMY                0xFF  //Dummy Data

typedef enum {
    STATE_COMMAND = 0,
    STATE_LENGTH = 1,
    STATE_DUMMY = 2,
    STATE_ATCMD = 3,
    STATE_SLAVE_DATA_READ = 4,
    STATE_SLAVE_DATA_WRITE = 5,
    STATE_NACK = 6
} spi_slave_state_t;

void on_spi_rx(void);
void platform_spi_write(uint8_t *data, uint16_t data_len);
void platform_spi_read(uint8_t *data, uint16_t data_len);
void platform_spi_write_dma(uint8_t *data, uint16_t data_len);
void platform_spi_read_dma(uint8_t *data, uint16_t data_len);
void platform_spi_reset(void);
void DATA0_SPI_Configuration(uint32_t main_clock);

static void spi_slave_read_dma(uint8_t *pBuf, uint16_t len);
static void spi_slave_write_dma(uint8_t *pBuf, uint16_t len);
void platform_spi_transfer(uint8_t *write_data, uint8_t *read_data, uint16_t data_len);
void spi_data_transfer_task(void *argument);
void spi_send_ack(void);
void spi_send_nack(void);
void spi_reset_timer_callback(TimerHandle_t xTimer);

#endif /* SPIHANDLER_H_ */
