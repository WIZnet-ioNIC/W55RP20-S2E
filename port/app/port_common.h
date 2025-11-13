/**
    Copyright (c) 2022 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _PORT_COMMON_H_
#define _PORT_COMMON_H_

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/
/* Common */
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include "pico/stdio/driver.h"
#include "pico/rand.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "hardware/uart.h"
#include "hardware/resets.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"

#if ((DEVICE_BOARD_NAME == W55RP20_S2E) || (DEVICE_BOARD_NAME == W232N) || (DEVICE_BOARD_NAME == IP20))
#include "hardware/pio.h"
#endif

#include "RP2040.h"
#include "WIZ5XXSR-RP_Debug.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

#endif /* _PORT_COMMON_H_ */
