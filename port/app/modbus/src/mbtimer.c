#include "port_common.h"
#include "mbtimer.h"
#include "common.h"

volatile eMBRcvState eRcvState;
repeating_timer_t g_mb_timer;

volatile uint8_t mb_state_rtu_finish;
volatile uint32_t mb_timeout;

extern xSemaphoreHandle seg_u2e_sem;

bool vMBPortTimersCallback(struct repeating_timer *t)
{
  xMBRTUTimerT35Expired();
  return true;
}

void xMBPortTimersInit(uint32_t usTim1Timerout50us)
{
    /* Calculate mb_timeout in μs: T3.5 + 50ms response timeout */
    uint32_t t35_time_us = usTim1Timerout50us * 50;
    if (usTim1Timerout50us > (0xFFFFFFFFUL / 50))
        mb_timeout = 0xFFFFFFFFUL; // Prevent overflow
    else
        mb_timeout = t35_time_us + 50000; // T3.5 + 50ms

    /* Check for overflow */
    if (mb_timeout < t35_time_us)
        mb_timeout = 0xFFFFFFFFUL;
	PRT_INFO("mb_timeout = %d us\r\n", mb_timeout);
}

void vMBPortTimersEnable( void )
{
	cancel_repeating_timer(&g_mb_timer);
	add_repeating_timer_us(mb_timeout, vMBPortTimersCallback, NULL, &g_mb_timer);
}

void vMBPortTimersDisable( void )
{
	cancel_repeating_timer(&g_mb_timer);
}

void xMBRTUTimerT35Expired( void )
{
  signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	switch ( eRcvState ) {
		/* Timer t35 expired. Startup phase is finished. */
		case STATE_RX_INIT:
			break;

		/* A frame was received and t35 expired. Notify the listener that
		* a new frame was received. */
		case STATE_RX_RCV:
			mb_state_rtu_finish = TRUE;  
            xSemaphoreGiveFromISR(seg_u2e_sem, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
			break;
		
		/* An error occured while receiving the frame. */
		case STATE_RX_ERROR:
			break;

		/* Function called in an illegal state. */
		default:
			break;
	}
	vMBPortTimersDisable(  );
	eRcvState = STATE_RX_IDLE;

	//printf("tim3\r\n");
}
