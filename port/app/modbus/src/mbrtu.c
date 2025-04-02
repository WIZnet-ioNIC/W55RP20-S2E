#include "pico/stdlib.h"
#include "common.h"
#include "ConfigData.h"
#include "uartHandler.h"

#include "mbcrc.h"
#include "mbrtu.h"
#include "mbtcp.h"
#include "mbtimer.h"
#include "mbserial.h"

volatile uint8_t ucRTUBuf[MB_SER_PDU_SIZE_MAX];
static volatile uint16_t usRcvBufferPos;

volatile uint8_t *pucTCPBufferCur;
volatile uint16_t usTCPBufferPos;
extern volatile uint8_t mb_state_rtu_finish;

void eMBRTUInit(uint32_t ulBaudRate)
{
    uint32_t usTimerT35_50us;
    uint32_t t35_time_us;

    /* Modbus RTU uses 8 databits. */

    /* If baud rate > 19200, use fixed timer value: t35 = 1750us.
     * Otherwise, t35 must be 3.5 times the character time.
     */
    if (baud_table[ulBaudRate] > 19200)
    {
        t35_time_us = 1750; // Fixed value: 1750¢®Iis
        usTimerT35_50us = 35; // 1750us / 50us = 35
    }
    else
    {
        /* Calculate 1 bit time (¢®Iis) = 1,000,000 / baudrate */
        uint32_t bit_time_us = 1000000UL / baud_table[ulBaudRate];
        /* Calculate 1 character time (¢®Iis) = 11 * bit_time_us */
        uint32_t char_time_us = bit_time_us * 11;
        /* Calculate T3.5 = 3.5 * character time (in ¢®Iis) */
        t35_time_us = (char_time_us * 35) / 10; // 3.5x calculation

        /* Ensure minimum value to prevent zero */
        if (t35_time_us < 1)
            t35_time_us = 1;

        /* Convert to 50¢®Iis units with ceiling */
        usTimerT35_50us = (t35_time_us + 49) / 50;

        /* Limit usTimerT35_50us to prevent timer overflow (e.g., 16-bit timer max = 65535) */
        if (usTimerT35_50us > 65535)
        {
            usTimerT35_50us = 65535; // Max value for 16-bit timer
            t35_time_us = usTimerT35_50us * 50; // Adjust t35_time_us accordingly
        }
    }

    PRT_INFO("Baud Rate: %u, usTimerT35_50us = %u\r\n", baud_table[ulBaudRate], usTimerT35_50us);
    xMBPortTimersInit(usTimerT35_50us); // Initialize timer
}

static bool mbRTUPackage( uint8_t * pucRcvAddress, uint8_t ** pucFrame, uint16_t * pusLength )
{
	//	uint8_t i;
	//	for(i=0; i<usRcvBufferPos; i++){printf("%d ",ucRTUBuf[i]);}

	/* Save the address field. All frames are passed to the upper layed
	* and the decision if a frame is used is done there.
	*/
	*pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

	/* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
	* size of address field and CRC checksum.
	*/
	*pusLength = ( uint16_t )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

	/* Return the start of the Modbus PDU to the caller. */
	*pucFrame = ( uint8_t * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
	
	return TRUE;
}

bool MBrtu2tcpFrame(void)
{
	uint8_t pucRcvAddress;
	uint16_t pusLength;
	uint8_t* ppucFrame;
	
	if(mbRTUPackage( &pucRcvAddress, &ppucFrame, &pusLength ) != FALSE)
	{
		pucTCPBufferCur = ppucFrame-7;

		pucTCPBufferCur[0] = mbTCPtid1;
		pucTCPBufferCur[1] = mbTCPtid2;

		pucTCPBufferCur[2] = 0;
		pucTCPBufferCur[3] = 0;
	
		pucTCPBufferCur[4] = ( pusLength + 1 ) >> 8U;
		pucTCPBufferCur[5] = ( pusLength + 1 ) & 0xFF;
	
		pucTCPBufferCur[6] = pucRcvAddress;	
		
		usTCPBufferPos = pusLength + 7;

		return TRUE;
	}
	return FALSE;
}

void RTU_Uart_RX(void)
{
	uint8_t ucByte;

	//printf(" 1> RTU_Uart_RX %d \r\n", eRcvState);

  while (1) {
    /* Always read the character. */
    if(UART_read(&ucByte, 1) <= 0) return;

    //printf(" 2> RTU_Uart_RX %d \r\n", eRcvState);

    switch ( eRcvState ) {
    	/* If we have received a character in the init state we have to
    	* wait until the frame is finished.
    	*/
    	case STATE_RX_INIT:
    	//printf(" > case STATE_RX_INIT:\r\n");
    		vMBPortTimersEnable(  );
    		break;

    	/* In the error state we wait until all characters in the
    	* damaged frame are transmitted.
    	*/
    	case STATE_RX_ERROR:
    		vMBPortTimersEnable(  );
    		break;

    	/* In the idle state we wait for a new character. If a character
    	* is received the t1.5 and t3.5 timers are started and the
    	* receiver is in the state STATE_RX_RECEIVCE.
    	*/
    	case STATE_RX_IDLE:
    		usRcvBufferPos = 0;
    		ucRTUBuf[usRcvBufferPos++] = ucByte;
    		eRcvState = STATE_RX_RCV;

    		//printf("%d ",ucByte);
    		/* Enable t3.5 timers. */
    		vMBPortTimersEnable(  );
    		break;

    	/* We are currently receiving a frame. Reset the timer after
    	* every character received. If more than the maximum possible
    	* number of bytes in a modbus frame is received the frame is
    	* ignored.
    	*/
    	case STATE_RX_RCV:
    		if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX ) {
    			ucRTUBuf[usRcvBufferPos++] = ucByte;
    			//printf("%d ",ucByte);
    		}
    		else {
    			eRcvState = STATE_RX_ERROR;
    		}

    		vMBPortTimersEnable();
    		//IWDG_ReloadCounter();
    		break;
    }
    if (mb_state_rtu_finish == TRUE) return;
  }
}

