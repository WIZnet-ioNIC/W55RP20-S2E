#include <string.h>
#include "socket.h"
#include "mbrtu.h"
#include "mbcrc.h"
#include "mbascii.h"
#include "common.h"
#include "ConfigData.h"
#include "WIZ5XXSR-RP_Debug.h" //hoon

#define MB_TCP_BUF_SIZE     ( 256 + 7 ) /* Must hold a complete Modbus TCP frame. */

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- MBAP Header --------------------------------------*/
/*

    <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
                <----------- MODBUS PDU (1') ---------------->
    +-----------+---------------+------------------------------------------+
    | TID | PID | Length | UID  |Code | Data                               |
    +-----------+---------------+------------------------------------------+
    |     |     |        |      |
    (2)   (3)   (4)      (5)    (6)

    (2)  ... MB_TCP_TID          = 0 (Transaction Identifier - 2 Byte)
    (3)  ... MB_TCP_PID          = 2 (Protocol Identifier - 2 Byte)
    (4)  ... MB_TCP_LEN          = 4 (Number of bytes - 2 Byte)
    (5)  ... MB_TCP_UID          = 6 (Unit Identifier - 1 Byte)
    (6)  ... MB_TCP_FUNC         = 7 (Modbus Function Code)

    (1)  ... Modbus TCP/IP Application Data Unit
    (1') ... Modbus Protocol Data Unit
*/
#define MB_TCP_TID1         0
#define MB_TCP_TID2         1
#define MB_TCP_PID          2
#define MB_TCP_LEN          4
#define MB_TCP_UID          6
#define MB_TCP_FUNC         7

#define MB_UDP_CRC16_SIZE   2

#define MB_TCP_PROTOCOL_ID  0   /* 0 = Modbus Protocol */

uint8_t mbTCPtid1[DEVICE_UART_CNT], mbTCPtid2[DEVICE_UART_CNT];
static uint8_t    aucTCPBuf[DEVICE_UART_CNT][MB_TCP_BUF_SIZE];

volatile uint8_t* pucRTUBufferCur[DEVICE_UART_CNT];
volatile uint16_t usRTUBufferPos[DEVICE_UART_CNT];

extern volatile uint8_t* pucASCIIBufferCur[DEVICE_UART_CNT];
extern volatile uint16_t usASCIIBufferPos[DEVICE_UART_CNT];

static bool mbTCPGet(uint8_t sock, uint8_t ** ppucMBTCPFrame, uint16_t * usTCPLength, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);

    uint16_t len;
    int usTCPBufPos;
    uint8_t  peerip[4];
    uint16_t peerport;

    len = getSn_RX_RSR(sock);

    if (len > 0) {
        if (network_connection->working_mode == UDP_MODE) {
            usTCPBufPos = recvfrom(sock, aucTCPBuf[channel], len, peerip, &peerport);

            if (usTCPBufPos < 0) {
                usTCPBufPos = 0;
            }

            if (memcmp(peerip, network_connection->remote_ip, sizeof(peerip)) || network_connection->remote_port != peerport) {
                network_connection->remote_ip[0] = peerip[0];
                network_connection->remote_ip[1] = peerip[1];
                network_connection->remote_ip[2] = peerip[2];
                network_connection->remote_ip[3] = peerip[3];
                network_connection->remote_port = peerport;
            }
        } else {
            usTCPBufPos = recv(sock, aucTCPBuf[channel], len);
#if 0
            PRT_INFO("[%d] > MB TCP Received len: %d Data: ", channel, usTCPBufPos);  //hoon
            for (int i = 0; i < usTCPBufPos; i++) {
                printf("%02X ", aucTCPBuf[channel][i]);
            }
            printf("\r\n");
#if 0
            if (usTCPBufPos != 12) {
                PRT_INFO("usTCPBufPos[%d] != 12 %d\r\n", channel, usTCPBufPos);
                //vTaskSuspendAll();
                while (1) {
                    vTaskDelay(100000);
                }
            }
#endif
#endif
            if (usTCPBufPos < 0) {
                usTCPBufPos = 0;
            }
        }
        *ppucMBTCPFrame = aucTCPBuf[channel];
        *usTCPLength = usTCPBufPos;
        return TRUE;
    }
    return FALSE;
}

static bool mbTCPPackage(uint8_t sock, uint8_t* pucRcvAddress, uint8_t** ppucFrame, uint16_t * pusLength, int channel) {
    uint8_t		*pucMBTCPFrame;
    uint16_t	usLength;
    uint16_t	usPID;

    if (mbTCPGet(sock, &pucMBTCPFrame, &usLength, channel) != FALSE) {
        usPID = pucMBTCPFrame[MB_TCP_PID] << 8U;
        usPID |= pucMBTCPFrame[MB_TCP_PID + 1];

        if (usPID == MB_TCP_PROTOCOL_ID) {
            /*  Modbus TCP does not use any addresses. Fake the source address such
                that the processing part deals with this frame.
            */
            *pucRcvAddress = pucMBTCPFrame[MB_TCP_UID];
            mbTCPtid1[channel] = pucMBTCPFrame[MB_TCP_TID1];
            mbTCPtid2[channel] = pucMBTCPFrame[MB_TCP_TID2];

            *ppucFrame = &pucMBTCPFrame[MB_TCP_FUNC];
            *pusLength = usLength - MB_TCP_FUNC;
            return TRUE;
        }
    }
    return FALSE;
}

bool MBtcp2rtuFrame(uint8_t sock, int channel) {
    uint8_t pucRcvAddress;
    uint16_t pusLength;
    uint8_t* ppucFrame;
    uint16_t usCRC16;

    if (mbTCPPackage(sock, &pucRcvAddress, &ppucFrame, &pusLength, channel) == TRUE) {
        pucRTUBufferCur[channel] = ppucFrame - 1;
        pucRTUBufferCur[channel][MB_SER_PDU_ADDR_OFF] = (uint8_t)pucRcvAddress;
        usRTUBufferPos[channel] = pusLength + MB_RTU_ADDR_SIZE;
        usCRC16 = usMBCRC16((uint8_t *) pucRTUBufferCur[channel], usRTUBufferPos[channel]);
        pucRTUBufferCur[channel][usRTUBufferPos[channel]++] = (uint8_t)(usCRC16 & 0xFF);
        pucRTUBufferCur[channel][usRTUBufferPos[channel]++] = (uint8_t)(usCRC16 >> 8);
        return TRUE;
    }

    return FALSE;
}

bool MBtcp2asciiFrame(uint8_t sock, int channel) {
    uint8_t pucRcvAddress;
    uint16_t pusLength;
    uint8_t* ppucFrame;
    uint8_t usLRC;

    if (mbTCPPackage(sock, &pucRcvAddress, &ppucFrame, &pusLength, channel) != FALSE) {
        pucASCIIBufferCur[channel] = ppucFrame - 1;
        pucASCIIBufferCur[channel][MB_SER_PDU_ADDR_OFF] = pucRcvAddress;

        usASCIIBufferPos[channel] = pusLength + MB_RTU_ADDR_SIZE;

        usLRC = prvucMBLRC((uint8_t *) pucASCIIBufferCur[channel], usASCIIBufferPos[channel]);
        pucASCIIBufferCur[channel][usASCIIBufferPos[channel]++] = usLRC;

        return TRUE;
    }
    return FALSE;
}

