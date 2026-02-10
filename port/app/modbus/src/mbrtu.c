#include "pico/stdlib.h"
#include "common.h"
#include "ConfigData.h"
#include "uartHandler.h"
#include "deviceHandler.h"

#include "mbcrc.h"
#include "mbrtu.h"
#include "mbtcp.h"
#include "mbtimer.h"
#include "mbserial.h"

extern uint8_t g_send_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
volatile uint8_t *ucRTUBuf[DEVICE_UART_CNT];
static volatile uint16_t usRcvBufferPos[DEVICE_UART_CNT];

volatile uint8_t *pucTCPBufferCur[DEVICE_UART_CNT];
volatile uint16_t usTCPBufferPos[DEVICE_UART_CNT];
extern volatile uint8_t mb_state_rtu_finish[DEVICE_UART_CNT];

void eMBRTUInit(uint32_t ulBaudRate, int channel) {
    uint32_t usTimerT35_50us;
    uint32_t t35_time_us;

    ucRTUBuf[channel] = g_send_buf[channel] + 7;
    /* Modbus RTU uses 8 databits. */

    /*  If baud rate > 19200, use fixed timer value: t35 = 1750us.
    */
    if (baud_table[ulBaudRate] > 19200) {
        t35_time_us = 1750; // Fixed value: 1750µs
        usTimerT35_50us = 35; // 1750us / 50us = 35
    } else {
        /* Calculate 1 bit time (µs) = 1,000,000 / baudrate */
        uint32_t bit_time_us = 1000000UL / baud_table[ulBaudRate];
        /* Calculate 1 character time (µs) = 11 * bit_time_us */
        uint32_t char_time_us = bit_time_us * 11;
        /* Calculate T3.5 = 3.5 * character time (in µs) */
        t35_time_us = (char_time_us * 35) / 10; // 3.5x calculation

        /* Ensure minimum value to prevent zero */
        if (t35_time_us < 1) {
            t35_time_us = 1;
        }

        /* Convert to 50µs units with ceiling */
        usTimerT35_50us = (t35_time_us + 49) / 50;

        /* Limit usTimerT35_50us to prevent timer overflow (e.g., 16-bit timer max = 65535) */
        if (usTimerT35_50us > 65535) {
            usTimerT35_50us = 65535; // Max value for 16-bit timer
            t35_time_us = usTimerT35_50us * 50; // Adjust t35_time_us accordingly
        }
    }

    PRT_INFO("Baud Rate: %u, usTimerT35_50us = %u\r\n", baud_table[ulBaudRate], usTimerT35_50us);
    xMBPortTimersInit(usTimerT35_50us, channel); // Initialize timer
}


static bool mbRTUPackage(uint8_t * pucRcvAddress, uint8_t ** pucFrame, uint16_t * pusLength, int channel) {
    //	uint8_t i;
    //	for(i=0; i<usRcvBufferPos; i++){printf("%d ",ucRTUBuf[i]);}

    /*  Save the address field. All frames are passed to the upper layed
        and the decision if a frame is used is done there.
    */
#if 1
    if (usMBCRC16(ucRTUBuf[channel], usRcvBufferPos[channel]) != 0) {
        PRT_INFO("CRC FAIL [%d]: len=%d\r\n", channel, usRcvBufferPos[channel]);
        return FALSE;  // CRC 실패 → 이 프레임 버림
    }
#endif
    *pucRcvAddress = ucRTUBuf[channel][MB_SER_PDU_ADDR_OFF];

    /*  Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
        size of address field and CRC checksum.
    */
    *pusLength = (uint16_t)(usRcvBufferPos[channel] - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC);

    /* Return the start of the Modbus PDU to the caller. */
    *pucFrame = (uint8_t *) & ucRTUBuf[channel][MB_SER_PDU_PDU_OFF];

    return TRUE;
}

bool MBrtu2tcpFrame(int channel) {
    uint8_t pucRcvAddress;
    uint16_t pusLength;
    uint8_t* ppucFrame;

    if (mbRTUPackage(&pucRcvAddress, &ppucFrame, &pusLength, channel) != FALSE) {
        pucTCPBufferCur[channel] = ppucFrame - 7;

        pucTCPBufferCur[channel][0] = mbTCPtid1[channel];
        pucTCPBufferCur[channel][1] = mbTCPtid2[channel];

        pucTCPBufferCur[channel][2] = 0;
        pucTCPBufferCur[channel][3] = 0;

        pucTCPBufferCur[channel][4] = (pusLength + 1) >> 8U;
        pucTCPBufferCur[channel][5] = (pusLength + 1) & 0xFF;

        pucTCPBufferCur[channel][6] = pucRcvAddress;

        usTCPBufferPos[channel] = pusLength + 7;

        return TRUE;
    }
    return FALSE;
}

void RTU_Uart_RX(int channel) {
    uint8_t ucByte;

    //printf(" 1> RTU_Uart_RX %d \r\n", eRcvState[channel]);

    while (1) {
        device_wdt_reset();
        /* Always read the character. */
        if (UART_read(&ucByte, 1, channel) <= 0) {
            return;
        }

        //printf(" 2> RTU_Uart_RX %d \r\n", eRcvState[channel]);

        switch (eRcvState[channel]) {
        /*  If we have received a character in the init state we have to
            wait until the frame is finished.
        */
        case STATE_RX_INIT:
            //printf(" > case STATE_RX_INIT:\r\n");
            vMBPortTimersEnable(channel);
            break;

        /*  In the error state we wait until all characters in the
            damaged frame are transmitted.
        */
        case STATE_RX_ERROR:
            vMBPortTimersEnable(channel);
            break;

        /*  In the idle state we wait for a new character. If a character
            is received the t1.5 and t3.5 timers are started and the
            receiver is in the state STATE_RX_RECEIVCE.
        */
        case STATE_RX_IDLE:
            usRcvBufferPos[channel] = 0;
            ucRTUBuf[channel][usRcvBufferPos[channel]++] = ucByte;
            eRcvState[channel] = STATE_RX_RCV;

            //printf("%d ",ucByte);
            /* Enable t3.5 timers. */
            vMBPortTimersEnable(channel);
            break;

        /*  We are currently receiving a frame. Reset the timer after
            every character received. If more than the maximum possible
            number of bytes in a modbus frame is received the frame is
            ignored.
        */
        case STATE_RX_RCV:
            if (usRcvBufferPos[channel] < MB_SER_PDU_SIZE_MAX) {
                ucRTUBuf[channel][usRcvBufferPos[channel]++] = ucByte;
            } else {
                PRT_ERR(" > STATE_RX_RCV: usRcvBufferPos[%d] = %d\r\n", channel, usRcvBufferPos[channel]);
                eRcvState[channel] = STATE_RX_ERROR;
            }

            vMBPortTimersEnable(channel);
            //IWDG_ReloadCounter();
            break;
        }
        //if (mb_state_rtu_finish[channel] == TRUE) {
        //    return;
        //}
    }
}

