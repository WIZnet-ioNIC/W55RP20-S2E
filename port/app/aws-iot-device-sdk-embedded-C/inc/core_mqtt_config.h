/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CORE_MQTT_CONFIG_H_
#define _CORE_MQTT_CONFIG_H_

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Buffer */
#define MQTT_BUF_MAX_SIZE (1024 * 2)
#define MQTT_DOMAIN_MAX_SIZE 128

/* Timeout */
#define MQTT_TIMEOUT (1000 * 10)    // 10 seconds
#define MQTT_KEEP_ALIVE (10 * 6)    // 60 seconds

/* Subscription number */
#define MQTT_SUBSCRIPTION_MAX_NUM 3
#define AWS_MQTT_DEBUG 0


#if AWS_MQTT_DEBUG
#define LogError( message ) printf ("[AWS_MQTT_E] %s %d ", __func__, __LINE__); \
                            printf message; \
                            printf("\r\n");
#define LogWarn( message )  printf ("[AWS_MQTT_W] %s %d ", __func__, __LINE__); \
                            printf message; \
                            printf("\r\n");
#define LogInfo( message )  printf ("[AWS_MQTT_I] %s %d ", __func__, __LINE__); \
                            printf message; \
                            printf("\r\n");
#define LogDebug( message ) printf ("[AWS_MQTT_D] %s %d ", __func__, __LINE__); \
                            printf message; \
                            printf("\r\n");
#endif


/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

#endif /* _CORE_MQTT_CONFIG_H_ */
