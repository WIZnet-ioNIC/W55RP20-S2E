/**
    Copyright (c) 2022 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_gpio_irq.h"

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
/* GPIO */
void wizchip_gpio_interrupt_initialize(uint8_t socket, uint16_t reg_val) {
    //uint16_t reg_val;
    int ret_val;

    //reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | /*SIK_RECEIVED |*/ SIK_TIMEOUT); // except SendOK
    //reg_val = SIK_RECEIVED;
    ret_val = ctlsocket(socket, CS_SET_INTMASK, (void *)&reg_val);

    ret_val = ctlwizchip(CW_GET_INTRMASK, (void *)&reg_val);
#if (_WIZCHIP_ == W5100S)
    reg_val = (1 << socket);
#elif (_WIZCHIP_ == W5500)
    reg_val = ((1 << socket) << 8) | reg_val;
#endif

    ret_val = ctlwizchip(CW_SET_INTRMASK, (void *)&reg_val);
}

