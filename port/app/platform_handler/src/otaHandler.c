#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "common.h"
#include "port_common.h"
#include "ConfigData.h"
#include "deviceHandler.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "otaHandler.h"

#include "socket.h"

/* AWS coreMQTT transport */
#include "mqtt_transport_interface.h"
#include "seg.h"

/* AWS coreJSON */
#include "core_json.h"

/* mbedTLS for SHA256 */
#include "mbedtls/sha256.h"

/* HTTPS / TLS */
#include "SSLInterface.h"
#include "dnsHandler.h"

/* -------------------------------------------------------------------------
 * Internal constants
 * ---------------------------------------------------------------------- */
#define OTA_JOB_ID_MAX_LEN      64
#define OTA_URL_MAX_LEN         512
#define OTA_SHA256_HEX_LEN      64   /* 32 bytes * 2 hex chars */
#define OTA_VERSION_MAX_LEN     32

#define OTA_HTTP_BUF_SIZE       DATA_BUF_SIZE
#define OTA_FLASH_SECTOR_SIZE   FLASH_SECTOR_SIZE

/* -------------------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------------- */
static mqtt_config_t *s_mqtt_config = NULL;
static char s_thing_name[MQTT_CLIENT_ID_SIZE + 1];

static char s_job_id[OTA_JOB_ID_MAX_LEN + 1];
static char s_fw_url[OTA_URL_MAX_LEN + 1];
static uint32_t s_fw_size;
static char s_fw_sha256[OTA_SHA256_HEX_LEN + 1];

/* Runtime topic buffers (built from client_id at ota_init time) */
static char s_notify_topic[OTA_TOPIC_BUF_SIZE];
static char s_ping_acc_topic[OTA_TOPIC_BUF_SIZE];
static char s_ping_rej_topic[OTA_TOPIC_BUF_SIZE];

/* TLS context for HTTPS firmware download */
static wiz_tls_context s_ota_tls_ctx;

/* OTA task: separate task to avoid stack overflow in MQTT yield task */
static SemaphoreHandle_t s_ota_job_sem = NULL;
static TaskHandle_t      s_ota_task_handle = NULL;

/* -------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------- */
static int  ota_parse_job_document(const uint8_t *payload, uint32_t len);
static void ota_report_status(const char *status, const char *reason);
static int  ota_download_and_flash(void);
static int  ota_verify_sha256(uint32_t fw_size, const char *expected_sha256_hex);
static int  hex_char_to_nibble(char c);
static int  sha256_hex_to_bytes(const char *hex, uint8_t *out, size_t out_len);
static void ota_task(void *pvParameters);
static int  ota_copy_thing_name(mqtt_config_t *mqtt_config);
static int  ota_subscribe_topic_if_needed(mqtt_config_t *mqtt_config,
                                          const char *topic,
                                          uint8_t qos);

/* =========================================================================
 * Public API
 * ====================================================================== */

int ota_init(void *mqtt_config) {
    uint8_t qos;

    s_mqtt_config = (mqtt_config_t *)mqtt_config;
    if (s_mqtt_config == NULL) {
        printf(" > OTA:INIT:Invalid MQTT context\r\n");
        return OTA_RET_FAILED;
    }

    if (ota_copy_thing_name(s_mqtt_config) != OTA_RET_SUCCESS) {
        printf(" > OTA:INIT:Missing client_id\r\n");
        return OTA_RET_FAILED;
    }

    /* Build topic strings for OTA message routing.
     * NOTE: The notify-next subscription is managed by the S2E config
     * (sub_topic_0 = "$aws/things/{id}/jobs/notify-next").
     * ota_init does NOT subscribe here — that would create a duplicate.
     * Incoming messages are routed to ota_mqtt_handle() via ota_is_ota_topic()
     * in mqtt_event_callback. */
    snprintf(s_notify_topic, sizeof(s_notify_topic),
             OTA_NOTIFY_TOPIC_FMT, s_thing_name);
    snprintf(s_ping_acc_topic, sizeof(s_ping_acc_topic),
             OTA_PING_ACC_TOPIC_FMT, s_thing_name);
    snprintf(s_ping_rej_topic, sizeof(s_ping_rej_topic),
             OTA_PING_REJ_TOPIC_FMT, s_thing_name);

    qos = get_DevConfig_pointer()->mqtt_option.qos;
    if (ota_subscribe_topic_if_needed(s_mqtt_config, s_notify_topic, qos) != OTA_RET_SUCCESS ||
        ota_subscribe_topic_if_needed(s_mqtt_config, s_ping_acc_topic, qos) != OTA_RET_SUCCESS ||
        ota_subscribe_topic_if_needed(s_mqtt_config, s_ping_rej_topic, qos) != OTA_RET_SUCCESS) {
        return OTA_RET_FAILED;
    }

    /* Create OTA task and semaphore (only once) */
    if (s_ota_job_sem == NULL) {
        s_ota_job_sem = xSemaphoreCreateBinary();
        xTaskCreate(ota_task, "OTA_TASK", 2048, NULL, tskIDLE_PRIORITY + 2,
                    &s_ota_task_handle);
        printf(" > OTA:INIT:Task created\r\n");
    }

    return OTA_RET_SUCCESS;
}

int ota_ping(void) {
    char topic[OTA_TOPIC_BUF_SIZE];
    uint8_t payload[] = "{}";

    if (s_mqtt_config == NULL) {
        printf(" > OTA:PING:Not initialized\r\n");
        return OTA_RET_FAILED;
    }

    snprintf(topic, sizeof(topic),
             OTA_PING_PUB_TOPIC_FMT, s_thing_name);

    printf(" > OTA:PING:Sending to %s\r\n", topic);
    return mqtt_transport_publish(s_mqtt_config,
                                  (uint8_t *)topic,
                                  payload,
                                  sizeof(payload) - 1,
                                  0 /* QoS0 */);
}

int ota_is_ota_topic(const char *topic, uint16_t topicLen) {
    /* Any topic starting with "$aws/things/" is treated as an OTA topic */
    const char prefix[] = "$aws/things/";
    if (topicLen < (uint16_t)(sizeof(prefix) - 1)) {
        return 0;
    }
    return (strncmp(topic, prefix, sizeof(prefix) - 1) == 0) ? 1 : 0;
}

void ota_mqtt_handle(const char *topic, uint16_t topicLen,
                     const uint8_t *payload, uint32_t payloadLen) {
    int ret;

    printf(" > OTA:MQTT:Topic=%.*s\r\n", topicLen, topic);

    /* ping accepted: cloud is reachable */
    if (strncmp(topic, s_ping_acc_topic, topicLen) == 0) {
        printf(" > OTA:PING:OK - Cloud connected\r\n");
        return;
    }

    /* ping rejected */
    if (strncmp(topic, s_ping_rej_topic, topicLen) == 0) {
        printf(" > OTA:PING:REJECTED - %.*s\r\n", (int)payloadLen, payload);
        return;
    }

    /* Parse the job document */
    ret = ota_parse_job_document(payload, payloadLen);
    if (ret != OTA_RET_SUCCESS) {
        printf(" > OTA:PARSE:FAILED\r\n");
        return;
    }

    printf(" > OTA:JOB_ID=%s\r\n", s_job_id);
    printf(" > OTA:VERSION=URL=%s\r\n", s_fw_url);
    printf(" > OTA:SIZE=%lu\r\n", (unsigned long)s_fw_size);
    printf(" > OTA:SHA256=%s\r\n", s_fw_sha256);

    /* Validate firmware size */
    if (s_fw_size == 0 || s_fw_size > FLASH_APP_BANK_SIZE) {
        printf(" > OTA:SIZE:INVALID\r\n");
        ota_report_status("FAILED", "INVALID_SIZE");
        return;
    }

    /* Signal OTA task to start download (don't block MQTT yield task) */
    if (s_ota_job_sem != NULL) {
        xSemaphoreGive(s_ota_job_sem);
        printf(" > OTA:JOB:Signaled OTA task\r\n");
    } else {
        printf(" > OTA:JOB:Task not ready\r\n");
    }
}

/* =========================================================================
 * Internal helpers
 * ====================================================================== */

/*
 * Parse AWS IoT Jobs notify-next payload:
 *
 * {
 *   "execution": {
 *     "jobId": "ota-test-xxx",
 *     "jobDocument": {
 *       "operation": "ota_update",
 *       "firmware": {
 *         "url":     "https://...",
 *         "version": "1.2.0",
 *         "size":    524288,
 *         "sha256":  "abcdef..."
 *       }
 *     }
 *   }
 * }
 */
static int ota_parse_job_document(const uint8_t *payload, uint32_t len) {
    JSONStatus_t json_ret;
    char *val;
    size_t val_len;
    char tmp[32];

    /* Validate JSON */
    json_ret = JSON_Validate((const char *)payload, len);
    if (json_ret != JSONSuccess) {
        printf(" > OTA:JSON:Invalid (%d)\r\n", json_ret);
        return OTA_RET_FAILED;
    }

    /* jobId */
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobId",
                           sizeof("execution.jobId") - 1,
                           &val, &val_len);
    if (json_ret != JSONSuccess || val_len == 0 || val_len > OTA_JOB_ID_MAX_LEN) {
        printf(" > OTA:JSON:jobId not found\r\n");
        return OTA_RET_FAILED;
    }
    memcpy(s_job_id, val, val_len);
    s_job_id[val_len] = '\0';

    /* firmware.url */
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobDocument.firmware.url",
                           sizeof("execution.jobDocument.firmware.url") - 1,
                           &val, &val_len);
    if (json_ret != JSONSuccess || val_len == 0 || val_len > OTA_URL_MAX_LEN) {
        printf(" > OTA:JSON:url not found\r\n");
        return OTA_RET_FAILED;
    }
    memcpy(s_fw_url, val, val_len);
    s_fw_url[val_len] = '\0';

    /* firmware.size */
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobDocument.firmware.size",
                           sizeof("execution.jobDocument.firmware.size") - 1,
                           &val, &val_len);
    if (json_ret != JSONSuccess || val_len == 0 || val_len >= sizeof(tmp)) {
        printf(" > OTA:JSON:size not found\r\n");
        return OTA_RET_FAILED;
    }
    memcpy(tmp, val, val_len);
    tmp[val_len] = '\0';
    s_fw_size = (uint32_t)atol(tmp);

    /* firmware.sha256 */
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobDocument.firmware.sha256",
                           sizeof("execution.jobDocument.firmware.sha256") - 1,
                           &val, &val_len);
    if (json_ret != JSONSuccess || val_len == 0 || val_len > OTA_SHA256_HEX_LEN) {
        printf(" > OTA:JSON:sha256 not found\r\n");
        return OTA_RET_FAILED;
    }
    memcpy(s_fw_sha256, val, val_len);
    s_fw_sha256[val_len] = '\0';

    return OTA_RET_SUCCESS;
}

static int ota_copy_thing_name(mqtt_config_t *mqtt_config) {
    size_t thing_name_len;

    if (mqtt_config == NULL ||
        mqtt_config->mqtt_connect_info.pClientIdentifier == NULL) {
        return OTA_RET_FAILED;
    }

    thing_name_len = mqtt_config->mqtt_connect_info.clientIdentifierLength;
    if (thing_name_len == 0 || thing_name_len > MQTT_CLIENT_ID_SIZE) {
        return OTA_RET_FAILED;
    }

    memcpy(s_thing_name,
           mqtt_config->mqtt_connect_info.pClientIdentifier,
           thing_name_len);
    s_thing_name[thing_name_len] = '\0';

    return OTA_RET_SUCCESS;
}

static int ota_subscribe_topic_if_needed(mqtt_config_t *mqtt_config,
                                         const char *topic,
                                         uint8_t qos) {
    uint8_t i;
    size_t topic_len;

    if (mqtt_config == NULL || topic == NULL) {
        return OTA_RET_FAILED;
    }

    topic_len = strlen(topic);
    for (i = 0; i < mqtt_config->subscribe_count; i++) {
        if (mqtt_config->mqtt_subscribe_info[i].topicFilterLength == topic_len &&
            strncmp(mqtt_config->mqtt_subscribe_info[i].pTopicFilter,
                    topic,
                    topic_len) == 0) {
            printf(" > OTA:SUB:Already subscribed %s\r\n", topic);
            return OTA_RET_SUCCESS;
        }
    }

    if (mqtt_transport_subscribe(mqtt_config, qos, (char *)topic) < 0) {
        printf(" > OTA:SUB:Failed %s\r\n", topic);
        return OTA_RET_FAILED;
    }

    printf(" > OTA:SUB:OK %s\r\n", topic);
    return OTA_RET_SUCCESS;
}

/*
 * Publish OTA job status update to AWS IoT Jobs.
 *
 * Topic: $aws/things/{thingName}/jobs/{jobId}/update
 * Body:  {"status":"IN_PROGRESS"} or {"status":"SUCCEEDED"} etc.
 */
static void ota_report_status(const char *status, const char *reason) {
    char topic[OTA_TOPIC_BUF_SIZE];
    char body[128];

    if (s_mqtt_config == NULL || s_job_id[0] == '\0') {
        return;
    }

    snprintf(topic, sizeof(topic),
             OTA_JOB_UPDATE_TOPIC_FMT,
             s_thing_name, s_job_id);

    if (reason != NULL) {
        snprintf(body, sizeof(body),
                 "{\"status\":\"%s\",\"statusDetails\":{\"reason\":\"%s\"}}",
                 status, reason);
    } else {
        snprintf(body, sizeof(body), "{\"status\":\"%s\"}", status);
    }

    printf(" > OTA:REPORT:%s\r\n", body);
    mqtt_transport_publish(s_mqtt_config,
                           (uint8_t *)topic,
                           (uint8_t *)body,
                           strlen(body),
                           1 /* QoS1 */);
}

/*
 * Download firmware from S3 presigned HTTPS URL and write directly to Bank1.
 *
 * Uses manual TLS + raw HTTP GET so the response body can be streamed
 * to flash in FLASH_SECTOR_SIZE chunks without buffering the entire binary.
 *
 * URL format: https://{host}/{path}?X-Amz-...
 */
/* Simple URL parser: extracts host and path from https://host/path?query */
static int ota_parse_url(const char *url, char *host_out, size_t host_max,
                         char *path_out, size_t path_max) {
    const char *p = url;

    /* Skip scheme (https:// or http://) */
    const char *scheme_end = strstr(p, "://");
    if (scheme_end == NULL) return -1;
    p = scheme_end + 3;

    /* Find end of host (first '/' after scheme) */
    const char *host_end = strchr(p, '/');
    if (host_end == NULL) {
        /* No path - just host */
        if ((size_t)(strlen(p)) >= host_max) return -1;
        strcpy(host_out, p);
        strcpy(path_out, "/");
        return 0;
    }

    size_t host_len = (size_t)(host_end - p);
    if (host_len >= host_max) return -1;
    memcpy(host_out, p, host_len);
    host_out[host_len] = '\0';

    /* Path is everything from '/' onwards */
    size_t path_len = strlen(host_end);
    if (path_len >= path_max) return -1;
    memcpy(path_out, host_end, path_len);
    path_out[path_len] = '\0';

    return 0;
}

static int ota_download_and_flash(void) {
    char host_str[128];
    char path_str[OTA_URL_MAX_LEN];

    /* Parse URL */
    if (ota_parse_url(s_fw_url, host_str, sizeof(host_str),
                      path_str, sizeof(path_str)) != 0) {
        printf(" > OTA:URL:Parse FAILED\r\n");
        return OTA_RET_FAILED;
    }
    printf(" > OTA:HTTPS:Host=%s\r\n", host_str);

    /* DNS resolve S3 hostname to IP */
    uint8_t s3_ip[4] = {0};
    printf(" > OTA:DNS:Resolving %s\r\n", host_str);
    if (get_ipaddr_from_dns((uint8_t *)host_str, s3_ip, 5000) != 1) {
        printf(" > OTA:DNS:FAILED\r\n");
        return OTA_RET_FAILED;
    }
    printf(" > OTA:DNS:Resolved %d.%d.%d.%d\r\n",
           s3_ip[0], s3_ip[1], s3_ip[2], s3_ip[3]);

    /* Init TLS - pass socket number as int* (matches seg.c pattern) */
    uint8_t sock = (uint8_t)SOCK_OTA_HTTP;
    int ret = wiz_tls_init(&s_ota_tls_ctx, (int *)sock);
    if (ret <= 0) {
        printf(" > OTA:TLS:Init FAILED (%d)\r\n", ret);
        return OTA_RET_FAILED;
    }

    /* Open TCP socket */
    ret = wiz_tls_socket(&s_ota_tls_ctx, sock, 0);
    if (ret != (int)sock) {
        printf(" > OTA:TLS:Socket FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* TCP connect to resolved IP */
    ret = connect(sock, s3_ip, 443);
    if (ret != SOCK_OK) {
        printf(" > OTA:TCP:Connect FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* TLS handshake */
    ret = wiz_tls_connect(&s_ota_tls_ctx, host_str, 443);
    if (ret != 0) {
        printf(" > OTA:TLS:Connect FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }
    printf(" > OTA:TLS:Connected\r\n");

    /* Build HTTP GET request */
    char request[OTA_URL_MAX_LEN + 128];
    int req_len = snprintf(request, sizeof(request),
                           "GET %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Connection: close\r\n"
                           "\r\n",
                           path_str, host_str);

    ret = (int)wiz_tls_write(&s_ota_tls_ctx, (uint8_t *)request, req_len);
    if (ret < 0) {
        printf(" > OTA:HTTP:Send FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* Read and skip HTTP response headers */
    uint8_t *http_buf = pvPortMalloc(OTA_HTTP_BUF_SIZE);
    if (http_buf == NULL) {
        printf(" > OTA:MALLOC:FAILED\r\n");
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* Read until double CRLF (end of headers) */
    uint32_t header_buf_len = 0;
    int header_done = 0;
    while (!header_done) {
        int n = (int)wiz_tls_read(&s_ota_tls_ctx,
                                  http_buf + header_buf_len,
                                  1);
        if (n <= 0) break;
        header_buf_len += n;
        if (header_buf_len >= 4) {
            if (memcmp(http_buf + header_buf_len - 4, "\r\n\r\n", 4) == 0) {
                header_done = 1;
            }
        }
        if (header_buf_len >= OTA_HTTP_BUF_SIZE - 1) {
            /* Header buffer full - shouldn't happen with normal S3 responses */
            break;
        }
    }

    if (!header_done) {
        printf(" > OTA:HTTP:Headers incomplete\r\n");
        vPortFree(http_buf);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* Check HTTP status code (first line) */
    if (strncmp((char *)http_buf, "HTTP/1.1 200", 12) != 0 &&
        strncmp((char *)http_buf, "HTTP/1.0 200", 12) != 0) {
        http_buf[header_buf_len < 32 ? header_buf_len : 32] = '\0';
        printf(" > OTA:HTTP:Non-200 response: %s\r\n", http_buf);
        vPortFree(http_buf);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    printf(" > OTA:HTTP:200 OK - Starting flash write\r\n");

    /* Stream firmware body directly to Bank1 flash */
    uint32_t f_addr = FLASH_START_ADDR_BANK1_OFFSET;
    uint8_t *sector_buf = pvPortMalloc(OTA_FLASH_SECTOR_SIZE);
    if (sector_buf == NULL) {
        printf(" > OTA:MALLOC:sector_buf FAILED\r\n");
        vPortFree(http_buf);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }
    memset(sector_buf, 0xFF, OTA_FLASH_SECTOR_SIZE);

    uint32_t total_recv = 0;
    uint32_t buf_fill   = 0;
    int download_ok     = 1;

    while (total_recv < s_fw_size) {
#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
        uint32_t want = OTA_HTTP_BUF_SIZE;
        if (total_recv + want > s_fw_size) {
            want = s_fw_size - total_recv;
        }

        int n = (int)wiz_tls_read(&s_ota_tls_ctx, http_buf, (unsigned int)want);
        if (n <= 0) {
            printf(" > OTA:RECV:EOF at %lu/%lu\r\n",
                   (unsigned long)total_recv, (unsigned long)s_fw_size);
            download_ok = 0;
            break;
        }

        /* Fill sector buffer, flush to flash when full */
        uint32_t offset = 0;
        while (offset < (uint32_t)n) {
            uint32_t space = OTA_FLASH_SECTOR_SIZE - buf_fill;
            uint32_t copy  = ((uint32_t)n - offset < space) ? ((uint32_t)n - offset) : space;

            memcpy(sector_buf + buf_fill, http_buf + offset, copy);
            buf_fill += copy;
            offset   += copy;

            if (buf_fill == OTA_FLASH_SECTOR_SIZE) {
                printf(" > OTA:FLASH:0x%08lX\r\n", (unsigned long)f_addr);
                write_flash(f_addr, sector_buf, OTA_FLASH_SECTOR_SIZE);
                f_addr  += OTA_FLASH_SECTOR_SIZE;
                buf_fill = 0;
                memset(sector_buf, 0xFF, OTA_FLASH_SECTOR_SIZE);
            }
        }
        total_recv += (uint32_t)n;
    }

    /* Flush last partial sector */
    if (download_ok && buf_fill > 0) {
        printf(" > OTA:FLASH:0x%08lX (last %lu bytes)\r\n",
               (unsigned long)f_addr, (unsigned long)buf_fill);
        write_flash(f_addr, sector_buf, OTA_FLASH_SECTOR_SIZE);
    }

    vPortFree(sector_buf);
    vPortFree(http_buf);
    wiz_tls_deinit(&s_ota_tls_ctx);
    close(sock);

    if (!download_ok || total_recv != s_fw_size) {
        printf(" > OTA:DOWNLOAD:Incomplete %lu/%lu\r\n",
               (unsigned long)total_recv, (unsigned long)s_fw_size);
        return OTA_RET_FAILED;
    }

    printf(" > OTA:DOWNLOAD:Complete %lu bytes\r\n", (unsigned long)total_recv);
    return OTA_RET_SUCCESS;
}

/*
 * Verify SHA256 of firmware written to Bank1.
 */
static int ota_verify_sha256(uint32_t fw_size, const char *expected_sha256_hex) {
    uint8_t expected[32];
    uint8_t computed[32];

    if (sha256_hex_to_bytes(expected_sha256_hex, expected, sizeof(expected)) != 0) {
        printf(" > OTA:SHA256:Invalid hex string\r\n");
        return OTA_RET_FAILED;
    }

    /* Compute SHA256 of firmware in Bank1 flash */
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0); /* 0 = SHA-256 */

    uint8_t *ptr = (uint8_t *)(XIP_BASE + FLASH_START_ADDR_BANK1_OFFSET);
    uint32_t remaining = fw_size;
    uint32_t chunk = 256;

    while (remaining > 0) {
        if (chunk > remaining) chunk = remaining;
        mbedtls_sha256_update(&ctx, ptr, chunk);
        ptr += chunk;
        remaining -= chunk;
    }

    mbedtls_sha256_finish(&ctx, computed);
    mbedtls_sha256_free(&ctx);

    if (memcmp(computed, expected, 32) != 0) {
        printf(" > OTA:SHA256:MISMATCH\r\n");
        printf(" > OTA:SHA256:Expected=%s\r\n", expected_sha256_hex);
        return OTA_RET_FAILED;
    }

    printf(" > OTA:SHA256:OK\r\n");
    return OTA_RET_SUCCESS;
}

static int hex_char_to_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int sha256_hex_to_bytes(const char *hex, uint8_t *out, size_t out_len) {
    if (strlen(hex) != out_len * 2) return -1;
    for (size_t i = 0; i < out_len; i++) {
        int hi = hex_char_to_nibble(hex[i * 2]);
        int lo = hex_char_to_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return 0;
}

/*
 * OTA task: runs separately from MQTT yield task to avoid stack overflow.
 * Waits for a job signal, then performs download (and eventually flash+reboot).
 */
static void ota_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        /* Wait indefinitely for a job to arrive */
        if (xSemaphoreTake(s_ota_job_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        printf(" > OTA:TASK:Starting download\r\n");

        ota_report_status("IN_PROGRESS", NULL);
        set_device_status(ST_UPGRADE);

        int ret = ota_download_and_flash();
        if (ret != OTA_RET_SUCCESS) {
            printf(" > OTA:TASK:Download FAILED\r\n");
            set_device_status(ST_OPEN);
            ota_report_status("FAILED", "DOWNLOAD_ERROR");
            continue;
        }

        /* Verify SHA256 of written firmware */
        int vret = ota_verify_sha256(s_fw_size, s_fw_sha256);
        if (vret != OTA_RET_SUCCESS) {
            printf(" > OTA:TASK:SHA256 MISMATCH\r\n");
            set_device_status(ST_OPEN);
            ota_report_status("FAILED", "SHA256_MISMATCH");
            continue;
        }

        printf(" > OTA:TASK:SHA256 OK - applying firmware\r\n");

        /* Set firmware update flag → bootloader will copy Bank1 to Bank0 */
        struct __firmware_update *fwupdate =
            (struct __firmware_update *)&(get_DevConfig_pointer()->firmware_update);
        fwupdate->fwup_size      = s_fw_size;
        fwupdate->fwup_copy_flag = 1;
        save_DevConfig_to_storage();

        ota_report_status("SUCCEEDED", NULL);
        vTaskDelay(pdMS_TO_TICKS(500)); /* wait for MQTT publish */

        device_reboot();
    }
}
