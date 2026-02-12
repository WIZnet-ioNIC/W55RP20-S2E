#include "port_common.h"
#include "mbtimer.h"
#include "common.h"
#include "seg.h"

volatile eMBRcvState eRcvState[DEVICE_UART_CNT];
repeating_timer_t g_mb_timer[DEVICE_UART_CNT];
volatile uint8_t mb_state_rtu_finish[DEVICE_UART_CNT];
volatile uint32_t mb_timeout[DEVICE_UART_CNT];

extern xSemaphoreHandle seg_u2e_sem[];

bool vMBPortTimersCallbackCh0(struct repeating_timer *t) {
    xMBRTUTimerT35Expired(SEG_DATA0_CH);
    return false; // Indicate that the timer callback was successful
}

bool vMBPortTimersCallbackCh1(struct repeating_timer *t) {
    xMBRTUTimerT35Expired(SEG_DATA1_CH);
    return false; // Indicate that the timer callback was successful
}

void vMBPortTimersCallback(struct repeating_timer *t) {
    if (t == &g_mb_timer[SEG_DATA0_CH]) {
        xMBRTUTimerT35Expired(SEG_DATA0_CH);
    } else if (t == &g_mb_timer[SEG_DATA1_CH]) {
        xMBRTUTimerT35Expired(SEG_DATA1_CH);
    }
}

void xMBPortTimersInit(uint32_t usTim1Timerout50us, int channel) {
    /* Calculate mb_timeout in ¥ìs: T3.5 + 50ms response timeout */
    uint32_t t35_time_us = usTim1Timerout50us * 50;
    if (usTim1Timerout50us > (0xFFFFFFFFUL / 50)) {
        mb_timeout[channel] = 0xFFFFFFFFUL;    // Prevent overflow
    } else {
        mb_timeout[channel] = t35_time_us ;    // T3.5 + 50ms // by Lihan
        // mb_timeout[channel] = t35_time_us + 50000;    // T3.5 + 50ms
    }

    /* Check for overflow */
    if (mb_timeout[channel] < t35_time_us) {
        mb_timeout[channel] = 0xFFFFFFFFUL;
    }
    // mb_timeout[channel] = mb_timeout[channel] * 8; // by Lihan
    PRT_INFO("mb_timeout = %d us\r\n", mb_timeout[channel]);
}

void vMBPortTimersEnable(int channel) {
    cancel_repeating_timer(&g_mb_timer[channel]);
#if 1
    if (channel == SEG_DATA0_CH) {
        add_repeating_timer_us(mb_timeout[channel], vMBPortTimersCallbackCh0, NULL, &g_mb_timer[channel]);
    } else if (channel == SEG_DATA1_CH) {
        add_repeating_timer_us(mb_timeout[channel], vMBPortTimersCallbackCh1, NULL, &g_mb_timer[channel]);
    }
#else
    add_repeating_timer_us(mb_timeout[channel], vMBPortTimersCallback, NULL, &g_mb_timer[channel]);
#endif
}

void vMBPortTimersDisable(int channel) {
    cancel_repeating_timer(&g_mb_timer[channel]);
}

void xMBRTUTimerT35Expired(int channel) {
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    switch (eRcvState[channel]) {
    /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        break;

    /*  A frame was received and t35 expired. Notify the listener that
        a new frame was received. */
    case STATE_RX_RCV:
        mb_state_rtu_finish[channel] = TRUE;
        xSemaphoreGiveFromISR(seg_u2e_sem[channel], &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
        break;

    /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

    /* Function called in an illegal state. */
    default:
        break;
    }
    vMBPortTimersDisable(channel);
    eRcvState[channel] = STATE_RX_IDLE;

}
