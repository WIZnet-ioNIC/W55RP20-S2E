/**
    Copyright (c) 2021 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _MQTT_TRANSPORT_INTERFACE_H_
#define _MQTT_TRANSPORT_INTERFACE_H_

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/

#include "transport_interface.h"
#include "core_mqtt.h"
#include "core_mqtt_state.h"
#include "core_mqtt_config.h"
#include "WIZnet_board.h"

/**
    ----------------------------------------------------------------------------------------------------
    Macros
    ----------------------------------------------------------------------------------------------------
*/

/**
    ----------------------------------------------------------------------------------------------------
    Structures
    ----------------------------------------------------------------------------------------------------
*/

typedef struct __mqtt_config {
    MQTTContext_t mqtt_context;
    MQTTConnectInfo_t mqtt_connect_info;
    MQTTFixedBuffer_t mqtt_fixed_buf;
    MQTTPublishInfo_t mqtt_publish_info;
    MQTTSubscribeInfo_t mqtt_subscribe_info[MQTT_SUBSCRIPTION_MAX_NUM];
    uint8_t subscribe_count;
    uint8_t mqtt_ip[4];
    uint8_t ssl_flag;
} mqtt_config_t;

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
void mqtt_event_callback(MQTTContext_t *pContext, MQTTPacketInfo_t *pPacketInfo, MQTTDeserializedInfo_t *pDeserializedInfo);
int mqtt_transport_yield(mqtt_config_t *mqtt_config);
int8_t mqtt_transport_init(uint8_t sock, mqtt_config_t *mqtt_config, uint8_t cleanSession, uint8_t ssl_flag, uint8_t *recv_buf,
                           uint32_t recv_buf_len, TransportInterface_t *transport_interface, NetworkContext_t *network_context,
                           uint8_t *ClientId, uint8_t *userName, uint8_t *password, uint32_t keepAlive,
                           void (*sub_callback)(uint8_t *, uint32_t), int channel);
int mqtt_transport_subscribe(mqtt_config_t *mqtt_config, uint8_t qos, char *subscribe_topic);
int8_t mqtt_transport_connect(mqtt_config_t *mqtt_config, uint32_t mqtt_conn_timeout);
int mqtt_transport_close(uint8_t sock, mqtt_config_t *mqtt_config, int channel);
int mqtt_transport_publish(mqtt_config_t *mqtt_config, uint8_t *pub_topic, uint8_t *pub_data, uint32_t pub_data_len, uint8_t qos);
int32_t mqtt_write(NetworkContext_t *pNetworkContext, const void *pBuffer, size_t bytesToSend);
int32_t mqtt_read(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv);

#ifdef __USE_S2E_OVER_TLS__
int32_t mqtts_write(NetworkContext_t *pNetworkContext, const void *pBuffer, size_t bytesToSend);
int32_t mqtts_read(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv);
#endif

#endif /* _MQTT_TRANSPORT_INTERFACE_H_ */
