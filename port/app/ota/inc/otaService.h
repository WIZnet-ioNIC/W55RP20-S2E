#ifndef _OTA_SERVICE_H_
#define _OTA_SERVICE_H_

/*
 * Standalone OTA background service.
 *
 * Runs as its own FreeRTOS task with its own MQTT-over-TLS connection to
 * AWS IoT, independent of the S2E working mode. All parameters and embedded
 * credentials come from otaConfig.h.
 *
 * Register from start_task():
 *     xTaskCreate(ota_service_task, "OTA_Service", ...);
 */
void ota_service_task(void *argument);

#endif /* _OTA_SERVICE_H_ */
