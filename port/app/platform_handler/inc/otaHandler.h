#ifndef _OTA_HANDLER_H_
#define _OTA_HANDLER_H_

#include <stdint.h>

/*
 * TODO: Fleet Provisioning 적용 시 하드코딩 제거 필요
 */
#define OTA_BROKER_ENDPOINT "a3uz5t2azg1xdz-ats.iot.ap-northeast-2.amazonaws.com"
#define OTA_BROKER_PORT     8883
#define OTA_TOPIC_BUF_SIZE  256

/* MQTT topic prefix - Thing Name appended at runtime */
#define OTA_NOTIFY_TOPIC_FMT     "$aws/things/%s/jobs/notify-next"
#define OTA_JOB_UPDATE_TOPIC_FMT "$aws/things/%s/jobs/%s/update"
#define OTA_PING_PUB_TOPIC_FMT   "$aws/things/%s/jobs/get"
#define OTA_PING_ACC_TOPIC_FMT   "$aws/things/%s/jobs/get/accepted"
#define OTA_PING_REJ_TOPIC_FMT   "$aws/things/%s/jobs/get/rejected"

/* Socket number used for HTTPS firmware download (dedicated, not used by HTTP server) */
#define SOCK_OTA_HTTP           7

/* Return codes */
#define OTA_RET_SUCCESS         0
#define OTA_RET_FAILED          -1

/**
 * @brief Subscribe to OTA notification topic.
 *        Call this after MQTT connection is established.
 *
 * @param mqtt_config  Pointer to the existing mqtt_config_t (g_mqtt_config)
 * @return 0 on success, -1 on failure
 */
int ota_init(void *mqtt_config);

/**
 * @brief Send a ping to AWS IoT Jobs to verify cloud connectivity.
 *        Publishes {} to $aws/things/{id}/jobs/get
 *        Response arrives via mqtt_event_callback:
 *          accepted → " > OTA:PING:OK"
 *          rejected → " > OTA:PING:REJECTED"
 *
 * @return 0 on publish success, -1 on failure
 */
int ota_ping(void);

/**
 * @brief Check if a received MQTT topic is an OTA topic.
 *
 * @param topic     Topic string (not null-terminated)
 * @param topicLen  Length of topic string
 * @return 1 if OTA topic, 0 otherwise
 */
int ota_is_ota_topic(const char *topic, uint16_t topicLen);

/**
 * @brief Handle an incoming OTA MQTT message.
 *        Parses job document, downloads firmware, verifies SHA256, reports status.
 *
 * @param topic      Topic string
 * @param topicLen   Topic string length
 * @param payload    MQTT payload
 * @param payloadLen Payload length
 */
void ota_mqtt_handle(const char *topic, uint16_t topicLen,
                     const uint8_t *payload, uint32_t payloadLen);

#endif /* _OTA_HANDLER_H_ */
