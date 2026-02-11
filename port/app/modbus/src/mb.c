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

static uint8_t *pucRTULastFrame[DEVICE_UART_CNT];
static uint16_t usRTULastFrameLen[DEVICE_UART_CNT];


int mbTCPtoRTU(uint8_t sock, int channel) {
    if (MBtcp2rtuFrame(sock, channel) == TRUE) {

        pucRTULastFrame[channel] = pucRTUBufferCur[channel];
        usRTULastFrameLen[channel] = usRTUBufferPos[channel];

        while (usRTUBufferPos[channel]) {
            UART_write((uint8_t*)pucRTUBufferCur[channel], 1, channel);
            pucRTUBufferCur[channel]++;
            usRTUBufferPos[channel]--;
        }
        return TRUE;
    }
    return FALSE;
}

void mbRTURetransmit(int channel) {
    uint8_t *ptr = pucRTULastFrame[channel];
    uint16_t len = usRTULastFrameLen[channel];



    while (len--) {
        UART_write((uint8_t*)ptr, 1, channel);
        ptr++;
    }
}

int mbRTUtoTCP(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    // PRT_INFO("[%d] > MB RTU to TCP start \r\n:", channel);  //hoon

    if (MBrtu2tcpFrame(channel) == TRUE) {
        switch (get_device_status(channel)) {
        case ST_UDP :
            sendto(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel], network_connection->remote_ip, network_connection->remote_port);
            break;
        case ST_CONNECT:
#if 0
            PRT_INFO("[%d] > MB RTU to TCP Send len: %d Data: ", channel, usTCPBufferPos[channel]);  //hoon
            for (int i = 0; i < usTCPBufferPos[channel]; i++) {
                printf("%02X ", pucTCPBufferCur[channel][i]);
            }
            printf("\r\n");
            if (usTCPBufferPos[channel] != 29) {
                PRT_INFO("usTCPBufferPos[%d] != 29 %d\r\n", channel, usTCPBufferPos[channel]);
                //vTaskSuspendAll();
                //while (1) {
                //    vTaskDelay(100000);
                //}
            }
#endif
            send(sock, (uint8_t*)pucTCPBufferCur[channel], usTCPBufferPos[channel]);
            break;
        default:
            break;
        }
        return TRUE;
    }
    return FALSE;
}

int mbTCPtoASCII(uint8_t sock, int channel) {
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
        return TRUE;
    }
    return FALSE;
}

int mbASCIItoTCP(uint8_t sock, int channel) {
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
        return TRUE;
    }
    return FALSE;
}
