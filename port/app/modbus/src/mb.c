#include "pico/stdlib.h"

#include "mb.h"
#include "mbtcp.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "mbserial.h"

#include "uartHandler.h"
#include "socket.h"
#include "common.h"
#include "ConfigData.h"
#include "seg.h"

extern volatile uint8_t* pucASCIIBufferCur[DEVICE_UART_CNT];
extern volatile uint16_t usASCIIBufferPos[DEVICE_UART_CNT];

void mbTCPtoRTU(uint8_t sock, int channel) {
    if (MBtcp2rtuFrame(sock, channel) == TRUE) {
        while (usRTUBufferPos[channel]) {
            UART_write((uint8_t*)pucRTUBufferCur[channel], 1, channel);
            pucRTUBufferCur[channel]++;
            usRTUBufferPos[channel]--;
        }
    }
}

void mbRTUtoTCP(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);

    if (MBrtu2tcpFrame(channel) == TRUE) {
        switch (get_device_status(channel)) {
        case ST_UDP :
            sendto(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel], network_connection->remote_ip, network_connection->remote_port);
            break;
        case ST_CONNECT:
            send(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel]);
            break;
        default:
            break;
        }
    }
}

void mbTCPtoASCII(uint8_t sock, int channel) {
    uint8_t ucByte;

    if (MBtcp2asciiFrame(sock, channel) != FALSE) {
        ucByte = MB_ASCII_START;
        UART_write(&ucByte, 1, channel);

        while (usASCIIBufferPos[channel]) {
            ucByte = prvucMBBIN2CHAR(*((uint8_t*)pucASCIIBufferCur[channel]) >> 4);
            UART_write(&ucByte, 1, channel);

            ucByte = prvucMBBIN2CHAR(*((uint8_t*)pucASCIIBufferCur[channel]) & 0x0F);
            UART_write(&ucByte, 1, channel);

            pucASCIIBufferCur[channel]++;
            usASCIIBufferPos[channel]--;
        }
        ucByte = MB_ASCII_DEFAULT_CR;
        UART_write(&ucByte, 1, channel);
        ucByte = MB_ASCII_DEFAULT_LF;
        UART_write(&ucByte, 1, channel);
    }
}

void mbASCIItoTCP(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);

    if (MBascii2tcpFrame(channel) != FALSE) {
        switch (getSn_SR(sock)) {
        case SOCK_UDP :
            sendto(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel], network_connection->remote_ip, network_connection->remote_port);
            break;
        case SOCK_ESTABLISHED:
        case SOCK_CLOSE_WAIT:
            send(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel]);
            break;
        default:
            break;
        }
    }
}
