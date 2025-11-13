/**
    Copyright (c) 2021 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <string.h>

#include "port_common.h"
#include "common.h"
#include "socket.h"
#include "mqtt_transport_interface.h"
#include "SSLInterface.h"
#include "timerHandler.h"
#include "util.h"
#include "core_mqtt_state.h"
#include "WIZnet_board.h"

/**
    ----------------------------------------------------------------------------------------------------
    Macros
    ----------------------------------------------------------------------------------------------------
*/
#define MQTT_INCOMING_PUB_RECORD_MAX 16
#define MQTT_OUTGOING_PUBREL_RECORD_MAX 16

/**
    ----------------------------------------------------------------------------------------------------
    Variables
    ----------------------------------------------------------------------------------------------------
*/
extern wiz_tls_context s2e_tlsContext[];
void (*user_sub_callback)(uint8_t *, uint32_t);

MQTTPubAckInfo_t incomingPubAckRecords[DEVICE_UART_CNT][MQTT_INCOMING_PUB_RECORD_MAX];
MQTTPubAckInfo_t outgoingPubRelRecords[DEVICE_UART_CNT][MQTT_OUTGOING_PUBREL_RECORD_MAX];

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
void mqtt_event_callback(MQTTContext_t *pContext, MQTTPacketInfo_t *pPacketInfo, MQTTDeserializedInfo_t *pDeserializedInfo) {
    /*  Handle incoming publish. The lower 4 bits of the publish packet
        type is used for the dup, QoS, and retain flags. Hence masking
        out the lower bits to check if the packet is publish. */
    if ((pPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH) {
        /* Handle incoming publish. */
        if (pDeserializedInfo->pPublishInfo->payloadLength) {
            printf("%.*s,%d,%.*s\n", pDeserializedInfo->pPublishInfo->topicNameLength, pDeserializedInfo->pPublishInfo->pTopicName,
                   pDeserializedInfo->pPublishInfo->payloadLength,
                   pDeserializedInfo->pPublishInfo->payloadLength, pDeserializedInfo->pPublishInfo->pPayload);
            user_sub_callback(pDeserializedInfo->pPublishInfo->pPayload, pDeserializedInfo->pPublishInfo->payloadLength);
        }
    } else {
        /* Handle other packets. */
        switch (pPacketInfo->type) {
        case MQTT_PACKET_TYPE_SUBACK: {
            printf("Received SUBACK: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        case MQTT_PACKET_TYPE_PINGRESP: {
            /*  Nothing to be done from application as library handles
                                      PINGRESP. */
            printf("Received PINGRESP\n");
            break;
        }

        case MQTT_PACKET_TYPE_UNSUBACK: {
            printf("Received UNSUBACK: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        case MQTT_PACKET_TYPE_PUBACK: {
            printf("Received PUBACK: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        case MQTT_PACKET_TYPE_PUBREC: {
            printf("Received PUBREC: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        case MQTT_PACKET_TYPE_PUBREL: {
            /*  Nothing to be done from application as library handles
                                      PUBREL. */
            printf("Received PUBREL: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        case MQTT_PACKET_TYPE_PUBCOMP: {
            /*  Nothing to be done from application as library handles
                                          PUBCOMP. */
            printf("Received PUBCOMP: PacketID=%u\n", pDeserializedInfo->packetIdentifier);
            break;
        }

        /* Any other packet type is invalid. */
        default: {
            printf("Unknown packet type received:(%02x).\n", pPacketInfo->type);
        }
        }
    }
}

int mqtt_transport_yield(mqtt_config_t *mqtt_config) {
    int ret;

    ret = MQTT_ProcessLoop(&mqtt_config->mqtt_context);
    if (ret != 0) {
        printf("MQTT process loop error : %d\n", ret);
    }
    return ret;
}

int8_t mqtt_transport_init(uint8_t sock, mqtt_config_t *mqtt_config, uint8_t cleanSession, uint8_t ssl_flag, uint8_t *recv_buf,
                           uint32_t recv_buf_len, TransportInterface_t *transport_interface, NetworkContext_t *network_context,
                           uint8_t *ClientId, uint8_t *userName, uint8_t *password, uint32_t keepAlive,
                           void (*sub_callback)(uint8_t *, uint32_t), int channel) {
    int ret;

    if (ClientId == NULL) {
        return -1;
    }
    memset((void *)mqtt_config, 0x00, sizeof(mqtt_config_t));

    /* Set MQTT connection information */
    mqtt_config->mqtt_connect_info.cleanSession = cleanSession;
    // Client ID must be unique to broker. This field is required.

    mqtt_config->mqtt_connect_info.pClientIdentifier = ClientId;
    mqtt_config->mqtt_connect_info.clientIdentifierLength = strlen(mqtt_config->mqtt_connect_info.pClientIdentifier);

    // The following fields are optional.
    // Value for keep alive.
    mqtt_config->mqtt_connect_info.keepAliveSeconds = keepAlive;
    // Optional username and password.

    mqtt_config->mqtt_connect_info.pUserName = userName;
    if (userName == NULL || userName[0] == NULL) {
        mqtt_config->mqtt_connect_info.userNameLength = 0;
    } else {
        mqtt_config->mqtt_connect_info.userNameLength = strlen(userName);
    }

    mqtt_config->mqtt_connect_info.pPassword = password;
    if (password == NULL || password[0] == NULL) {
        mqtt_config->mqtt_connect_info.passwordLength = 0;
    } else {
        mqtt_config->mqtt_connect_info.passwordLength = strlen(password);
    }

    mqtt_config->ssl_flag = ssl_flag;
    if (ssl_flag == 0) {
        transport_interface->send = mqtt_write;
        transport_interface->recv = mqtt_read;
    }

#ifdef __USE_S2E_OVER_TLS__
    else {
        transport_interface->send = mqtts_write;
        transport_interface->recv = mqtts_read;
    }
#endif

    mqtt_config->mqtt_fixed_buf.pBuffer = recv_buf;
    mqtt_config->mqtt_fixed_buf.size = recv_buf_len;
    mqtt_config->subscribe_count = 0;

    network_context->socketDescriptor = sock;
    network_context->channel = channel;
    transport_interface->pNetworkContext = network_context;

    user_sub_callback = sub_callback;
    /* Initialize MQTT context */
    ret = MQTT_Init(&mqtt_config->mqtt_context,
                    transport_interface,
                    (MQTTGetCurrentTimeFunc_t)xTaskGetTickCount,
                    //                    (MQTTGetCurrentTimeFunc_t)millis,
                    mqtt_event_callback,
                    &mqtt_config->mqtt_fixed_buf);

    if (ret != 0) {
        printf("MQTT initialization is error : %d\n", ret);
        return -1;
    } else {
        printf("MQTT initialization is success\n");
    }

    ret = MQTT_InitStatefulQoS(
              &mqtt_config->mqtt_context,
              incomingPubAckRecords[channel], MQTT_INCOMING_PUB_RECORD_MAX,
              outgoingPubRelRecords[channel], MQTT_OUTGOING_PUBREL_RECORD_MAX
          );

    if (ret != 0) {
        printf("MQTT_InitStatefulQoS error : %d\n", ret);
        return -1;
    }
    return 0;
}

int mqtt_transport_subscribe(mqtt_config_t *mqtt_config, uint8_t qos, char *subscribe_topic) {
    int packet_id = 0;
    packet_id = MQTT_GetPacketId(&mqtt_config->mqtt_context);
    uint32_t ret;

    if (mqtt_config->subscribe_count > MQTT_SUBSCRIPTION_MAX_NUM) {
        printf("MQTT subscription count error : %d\n", mqtt_config->subscribe_count);
        return -1;
    }

    mqtt_config->mqtt_subscribe_info[mqtt_config->subscribe_count].qos = qos;
    mqtt_config->mqtt_subscribe_info[mqtt_config->subscribe_count].pTopicFilter = subscribe_topic;
    mqtt_config->mqtt_subscribe_info[mqtt_config->subscribe_count].topicFilterLength = strlen(subscribe_topic);

    /* Receive message */
    ret = MQTT_Subscribe(&mqtt_config->mqtt_context, &mqtt_config->mqtt_subscribe_info[mqtt_config->subscribe_count], 1, packet_id);

    if (ret != 0) {
        printf("MQTT subscription is error : %d\n", ret);
        return -1;
    } else {
        printf("MQTT subscription is success\n");
    }
    mqtt_config->subscribe_count++;
    return 0;
}

int8_t mqtt_transport_connect(mqtt_config_t *mqtt_config, uint32_t mqtt_conn_timeout) {
    bool session_present;
    int ret = -1;

    /* Connect to the MQTT broker */
    if (mqtt_conn_timeout == 0) {
        mqtt_conn_timeout = MQTT_TIMEOUT;
    }

    ret = MQTT_Connect(&mqtt_config->mqtt_context, &mqtt_config->mqtt_connect_info, NULL, mqtt_conn_timeout, &session_present);
    if (ret != 0) {
        printf("MQTT connection is error : %d\n", ret);
        return -1;
    } else {
        printf("MQTT connection is success\n");
    }
    return 0;
}

int mqtt_transport_close(uint8_t sock, mqtt_config_t *mqtt_config, int channel) {

#ifdef __USE_S2E_OVER_TLS__
    if (mqtt_config->ssl_flag == true) {
        wiz_tls_close_notify(&s2e_tlsContext[channel]);
        wiz_tls_session_reset(&s2e_tlsContext[channel]);
        wiz_tls_deinit(&s2e_tlsContext[channel]);
        set_wiz_tls_init_state(DISABLE, channel);
    }
#endif
    mqtt_config->subscribe_count = 0;
    //    ret = disconnect(sock);
    close(sock);

#ifdef __USE_S2E_OVER_TLS__
    set_wiz_tls_init_state(DISABLE, channel);
#endif

    return 0;
}

int mqtt_transport_publish(mqtt_config_t *mqtt_config, uint8_t *pub_topic, uint8_t *pub_data, uint32_t pub_data_len, uint8_t qos) {
    int packet_id;
    int ret;

    mqtt_config->mqtt_publish_info.qos = qos;

    mqtt_config->mqtt_publish_info.pTopicName = pub_topic;
    mqtt_config->mqtt_publish_info.topicNameLength = strlen(pub_topic);

    mqtt_config->mqtt_publish_info.pPayload = pub_data;
    mqtt_config->mqtt_publish_info.payloadLength = pub_data_len;

    packet_id = MQTT_GetPacketId(&mqtt_config->mqtt_context);
    /* Send message */
    ret = MQTT_Publish(&mqtt_config->mqtt_context, &mqtt_config->mqtt_publish_info, packet_id);

    if (ret != 0) {
        printf("MQTT pulishing is error : %d\n", ret);
        printf("PUBLISH FAILED\n");

        return -1;
    } else {
        printf("MQTT pulishing is success\n");
        printf("PUBLISH OK\n");

        return 0;
    }
}

int32_t mqtt_write(NetworkContext_t *pNetworkContext, const void *pBuffer, size_t bytesToSend) {
    int32_t size = 0;

    if (getSn_SR(pNetworkContext->socketDescriptor) == SOCK_ESTABLISHED) {
        size = send(pNetworkContext->socketDescriptor, (uint8_t *)pBuffer, bytesToSend);
    }

    return size;
}

int32_t mqtt_read(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv) {
    int32_t size = 0;

    if (getSn_RX_RSR(pNetworkContext->socketDescriptor) > 0) {
        size = recv(pNetworkContext->socketDescriptor, pBuffer, bytesToRecv);
    }
    return size;
}

#ifdef __USE_S2E_OVER_TLS__
int32_t mqtts_write(NetworkContext_t *pNetworkContext, const void *pBuffer, size_t bytesToSend) {
    int32_t size = 0;

    if (getSn_SR(pNetworkContext->socketDescriptor) == SOCK_ESTABLISHED) {
        size = wiz_tls_write(&s2e_tlsContext[pNetworkContext->channel], (uint8_t *)pBuffer, bytesToSend);
    }

    return size;
}

int32_t mqtts_read(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv) {
    int32_t size = 0;

    if (getSn_SR(pNetworkContext->socketDescriptor) == SOCK_ESTABLISHED) {
        size = wiz_tls_read(&s2e_tlsContext[pNetworkContext->channel], pBuffer, bytesToRecv);
    }

    return size;
}
#endif