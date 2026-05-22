/*
 * ============================================================================
 * Standalone OTA background service
 * ----------------------------------------------------------------------------
 * Owns a dedicated MQTT-over-TLS connection to AWS IoT, independent of the S2E
 * working mode. All parameters / credentials come from otaConfig.h.
 *
 * BUILD STAGE 4+5: full pipeline.
 *   TLS + coreMQTT CONNECT/SUBSCRIBE + $next/get poll, and on a received job:
 *   report IN_PROGRESS -> drive otaDownload.c (download/verify/flash) ->
 *   report SUCCEEDED/FAILED -> reboot on success.
 *
 * Connection resources are fully separate from the S2E MQTT connection:
 *   - own W5500 socket           (OTA_CFG_MQTT_SOCK)
 *   - own TLS context            (s_ota_tls)
 *   - own coreMQTT context       (s_ota_mqtt)
 *   - own client id              (OTA_CFG_CLIENT_ID)
 *
 * Dependency: otaService.c -> otaDownload.h (one-way). The download engine
 * never calls back into this file.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

#include "otaConfig.h"
#include "otaService.h"
#include "otaDownload.h"

#include "common.h"        /* MAJOR_VER / MINOR_VER / MAINTENANCE_VER */
#include "socket.h"
#include "dnsHandler.h"
#include "deviceHandler.h" /* FLASH_OTA_INFO_ADDR */
#include "flashHandler.h"  /* read_flash / write_flash */
#include "SSLInterface.h"

/* coreMQTT + the mqtt_config_t aggregate (pulls core_mqtt.h / transport types) */
#include "mqtt_transport_interface.h"

/* coreJSON — used here only to pull the jobId out for the IN_PROGRESS report */
#include "core_json.h"

#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"

/* -------------------------------------------------------------------------
 * Tunables
 * ---------------------------------------------------------------------- */
#define OTA_SVC_RETRY_MS       10000   /* delay after a connection failure   */
#define OTA_SVC_YIELD_MS       200     /* MQTT_ProcessLoop cadence            */
#define OTA_SVC_RECV_BUF_SIZE  2048    /* coreMQTT fixed buffer (static BSS)  */
#define OTA_SVC_JOBDOC_BUF     2048    /* copy of an incoming job document    */
#define OTA_SVC_QOS_RECORD_MAX 8       /* QoS1/2 in-flight record slots       */
#define OTA_SVC_TOPIC_LEN      160

/* -------------------------------------------------------------------------
 * Internal state  (mqtt_config_t / buffers are static BSS, not heap)
 * ---------------------------------------------------------------------- */
static wiz_tls_context      s_ota_tls;
static mqtt_config_t        s_ota_mqtt;
static NetworkContext_t     s_ota_net;
static TransportInterface_t s_ota_transport;
static uint8_t              s_ota_recv_buf[OTA_SVC_RECV_BUF_SIZE];
static MQTTPubAckInfo_t     s_ota_incoming[OTA_SVC_QOS_RECORD_MAX];
static MQTTPubAckInfo_t     s_ota_outgoing[OTA_SVC_QOS_RECORD_MAX];

/* Runtime topic strings (thing name substituted at startup) */
static char s_topic_notify[OTA_SVC_TOPIC_LEN];
static char s_topic_nextget[OTA_SVC_TOPIC_LEN];
static char s_topic_nextget_acc[OTA_SVC_TOPIC_LEN];
static char s_topic_shadow[OTA_SVC_TOPIC_LEN];

/* Job hand-off: the MQTT callback copies the document here and raises the
 * flag; the task loop performs the (blocking) download outside the callback.
 * Both run in the same task, so no locking is needed. */
static uint8_t  s_job_doc[OTA_SVC_JOBDOC_BUF];
static uint32_t s_job_doc_len;
static int      s_job_pending;

/* ==========================================================================
 * TLS layer
 * ======================================================================= */

/*
 * Mirror of wiz_tls_init() but sources the X.509 material from the embedded
 * strings in otaConfig.h instead of the S2E FLASH region / ssl_option.
 *
 * @return 1 on success, -1 on failure
 */
static int ota_tls_init(wiz_tls_context *ctx, uint8_t sock) {
    const char *pers = "ota_client";
    int ret;

#if defined(MBEDTLS_ENTROPY_C)
    ctx->entropy = pvPortMalloc(sizeof(mbedtls_entropy_context));
#endif
    ctx->ctr_drbg = pvPortMalloc(sizeof(mbedtls_ctr_drbg_context));
    ctx->ssl      = pvPortMalloc(sizeof(mbedtls_ssl_context));
    ctx->conf     = pvPortMalloc(sizeof(mbedtls_ssl_config));
    ctx->cacert   = pvPortMalloc(sizeof(mbedtls_x509_crt));
    ctx->clicert  = pvPortMalloc(sizeof(mbedtls_x509_crt));
    ctx->pkey     = pvPortMalloc(sizeof(mbedtls_pk_context));

    if (ctx->ctr_drbg == NULL || ctx->ssl == NULL || ctx->conf == NULL ||
        ctx->cacert == NULL || ctx->clicert == NULL || ctx->pkey == NULL
#if defined(MBEDTLS_ENTROPY_C)
        || ctx->entropy == NULL
#endif
       ) {
        printf(" > OTA_SVC:TLS:malloc FAILED\r\n");
        return -1;
    }

#if defined(MBEDTLS_ENTROPY_C)
    mbedtls_entropy_init(ctx->entropy);
#endif
    mbedtls_ctr_drbg_init(ctx->ctr_drbg);
    mbedtls_ssl_init(ctx->ssl);
    mbedtls_ssl_config_init(ctx->conf);
    mbedtls_x509_crt_init(ctx->cacert);
    mbedtls_x509_crt_init(ctx->clicert);
    mbedtls_pk_init(ctx->pkey);

#if defined(MBEDTLS_ENTROPY_C)
    ret = mbedtls_ctr_drbg_seed(ctx->ctr_drbg, mbedtls_entropy_func,
                                ctx->entropy,
                                (const unsigned char *)pers, strlen(pers));
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:drbg_seed FAILED -0x%x\r\n", -ret);
        return -1;
    }
#endif

    ret = mbedtls_x509_crt_parse(ctx->cacert,
                                 (const unsigned char *)OTA_CFG_ROOT_CA,
                                 sizeof(OTA_CFG_ROOT_CA));
    if (ret < 0) {
        printf(" > OTA_SVC:TLS:root CA parse FAILED -0x%x\r\n", -ret);
        return -1;
    }

    ret = mbedtls_ssl_set_hostname(ctx->ssl, OTA_CFG_ENDPOINT);
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:set_hostname FAILED %d\r\n", ret);
        return -1;
    }

    ret = mbedtls_x509_crt_parse(ctx->clicert,
                                 (const unsigned char *)OTA_CFG_CLIENT_CRT,
                                 sizeof(OTA_CFG_CLIENT_CRT));
    if (ret < 0) {
        printf(" > OTA_SVC:TLS:client cert parse FAILED -0x%x\r\n", -ret);
        return -1;
    }

    ret = mbedtls_pk_parse_key(ctx->pkey,
                               (const unsigned char *)OTA_CFG_CLIENT_KEY,
                               sizeof(OTA_CFG_CLIENT_KEY),
                               NULL, 0,
                               mbedtls_ctr_drbg_random, ctx->ctr_drbg);
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:private key parse FAILED -0x%x\r\n", -ret);
        return -1;
    }

    ret = mbedtls_ssl_config_defaults(ctx->conf,
                                      MBEDTLS_SSL_IS_CLIENT,
                                      MBEDTLS_SSL_TRANSPORT_STREAM,
                                      MBEDTLS_SSL_PRESET_DEFAULT);
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:config_defaults FAILED %d\r\n", ret);
        return -1;
    }

    /* VERIFY_OPTIONAL: verify the server cert and log the result, but do not
     * abort the handshake on failure (tolerates a wrong device clock). */
    mbedtls_ssl_conf_authmode(ctx->conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
    mbedtls_ssl_conf_ca_chain(ctx->conf, ctx->cacert, NULL);
    mbedtls_ssl_conf_rng(ctx->conf, mbedtls_ctr_drbg_random, ctx->ctr_drbg);

    ret = mbedtls_ssl_conf_own_cert(ctx->conf, ctx->clicert, ctx->pkey);
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:conf_own_cert FAILED %d\r\n", ret);
        return -1;
    }

    mbedtls_ssl_conf_endpoint(ctx->conf, MBEDTLS_SSL_IS_CLIENT);
    mbedtls_ssl_conf_read_timeout(ctx->conf, 2000);

    ret = mbedtls_ssl_setup(ctx->ssl, ctx->conf);
    if (ret != 0) {
        printf(" > OTA_SVC:TLS:ssl_setup FAILED -0x%x\r\n", -ret);
        return -1;
    }

    ctx->socket_fd = sock;
    mbedtls_ssl_set_bio(ctx->ssl, (void *)(uintptr_t)sock,
                        WIZnetSend, WIZnetRecv, WIZnetRecvTimeOut);
    return 1;
}

/* ==========================================================================
 * coreMQTT transport — bound to the OTA TLS context
 * ======================================================================= */

static int32_t ota_tls_send(NetworkContext_t *nc, const void *buf, size_t len) {
    int32_t size = 0;
    if (getSn_SR(nc->socketDescriptor) == SOCK_ESTABLISHED) {
        size = wiz_tls_write(&s_ota_tls, (uint8_t *)buf, len);
    }
    return size;
}

static int32_t ota_tls_recv(NetworkContext_t *nc, void *buf, size_t len) {
    int32_t size = 0;
    if (getSn_SR(nc->socketDescriptor) == SOCK_ESTABLISHED) {
        size = wiz_tls_read(&s_ota_tls, buf, len);
        if (size < 0) {
            size = 0;   /* WANT_READ / no data — not fatal */
        }
    }
    return size;
}

/* ==========================================================================
 * coreMQTT event callback
 *  Runs inside MQTT_ProcessLoop (same task). Keep it short: copy the job
 *  document out and raise a flag — the actual download runs in the task loop.
 * ======================================================================= */

static void ota_mqtt_event_cb(MQTTContext_t *ctx,
                              MQTTPacketInfo_t *pkt,
                              MQTTDeserializedInfo_t *info) {
    (void)ctx;

    if ((pkt->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH) {
        if (info->pPublishInfo == NULL || info->pPublishInfo->payloadLength == 0) {
            return;
        }
        printf(" > OTA_SVC:RX topic=%.*s\r\n",
               info->pPublishInfo->topicNameLength,
               info->pPublishInfo->pTopicName);

        uint32_t plen = info->pPublishInfo->payloadLength;
        if (plen >= sizeof(s_job_doc)) {
            printf(" > OTA_SVC:payload too large (%lu) — dropped\r\n",
                   (unsigned long)plen);
            return;
        }
        memcpy(s_job_doc, info->pPublishInfo->pPayload, plen);
        s_job_doc[plen] = '\0';
        s_job_doc_len   = plen;

        /* A real job carries a jobDocument; empty $next/get replies don't. */
        if (strstr((char *)s_job_doc, "jobDocument") != NULL) {
            printf(" > OTA_SVC:job document received (%lu bytes)\r\n",
                   (unsigned long)plen);
            s_job_pending = 1;
        } else {
            printf(" > OTA_SVC:no job pending\r\n");
        }
        return;
    }

    switch (pkt->type) {
    case MQTT_PACKET_TYPE_SUBACK:
        printf(" > OTA_SVC:SUBACK id=%u\r\n", info->packetIdentifier);
        break;
    case MQTT_PACKET_TYPE_PUBACK:
        printf(" > OTA_SVC:PUBACK id=%u\r\n", info->packetIdentifier);
        break;
    case MQTT_PACKET_TYPE_PINGRESP:
        break;
    default:
        break;
    }
}

/* ==========================================================================
 * MQTT helpers
 * ======================================================================= */

static int ota_mqtt_subscribe(const char *topic) {
    MQTTSubscribeInfo_t sub;
    uint16_t pid = MQTT_GetPacketId(&s_ota_mqtt.mqtt_context);

    sub.qos               = MQTTQoS0;
    sub.pTopicFilter      = topic;
    sub.topicFilterLength = (uint16_t)strlen(topic);

    if (MQTT_Subscribe(&s_ota_mqtt.mqtt_context, &sub, 1, pid) != MQTTSuccess) {
        printf(" > OTA_SVC:SUBSCRIBE FAILED %s\r\n", topic);
        return -1;
    }
    printf(" > OTA_SVC:SUBSCRIBE %s\r\n", topic);
    return 0;
}

static int ota_mqtt_publish(const char *topic, const char *payload) {
    MQTTPublishInfo_t pub;
    uint16_t pid = MQTT_GetPacketId(&s_ota_mqtt.mqtt_context);

    memset(&pub, 0, sizeof(pub));
    pub.qos             = MQTTQoS0;
    pub.pTopicName      = topic;
    pub.topicNameLength = (uint16_t)strlen(topic);
    pub.pPayload        = payload;
    pub.payloadLength   = strlen(payload);

    if (MQTT_Publish(&s_ota_mqtt.mqtt_context, &pub, pid) != MQTTSuccess) {
        printf(" > OTA_SVC:PUBLISH FAILED %s\r\n", topic);
        return -1;
    }
    return 0;
}

/* ==========================================================================
 * Job status reporting  (AWS IoT Jobs + Device Shadow)
 * ======================================================================= */

static const char *ota_reason_str(ota_dl_result_t r) {
    switch (r) {
    case OTA_DL_ERR_PARSE:    return "PARSE_ERROR";
    case OTA_DL_ERR_SIZE:     return "INVALID_SIZE";
    case OTA_DL_ERR_DOWNLOAD: return "DOWNLOAD_ERROR";
    case OTA_DL_ERR_SHA256:   return "SHA256_MISMATCH";
    case OTA_DL_ERR_FLASH:    return "INVALID_FLASH";
    default:                  return "UNKNOWN";
    }
}

/* Publish a Jobs execution update: $aws/things/{thing}/jobs/{jobId}/update */
static void ota_report_job(const char *job_id, const char *status,
                           const char *reason, const char *version) {
    char topic[OTA_SVC_TOPIC_LEN];
    char body[192];

    snprintf(topic, sizeof(topic),
             OTA_CFG_TOPIC_JOB_UPDATE, OTA_CFG_THING_NAME, job_id);

    if (version != NULL && version[0] != '\0') {
        snprintf(body, sizeof(body),
                 "{\"status\":\"%s\",\"statusDetails\":{\"version\":\"%s\"}}",
                 status, version);
    } else if (reason != NULL && reason[0] != '\0') {
        snprintf(body, sizeof(body),
                 "{\"status\":\"%s\",\"statusDetails\":{\"reason\":\"%s\"}}",
                 status, reason);
    } else {
        snprintf(body, sizeof(body), "{\"status\":\"%s\"}", status);
    }

    printf(" > OTA_SVC:REPORT job=%s %s\r\n", job_id, body);
    ota_mqtt_publish(topic, body);
}

/* Publish the live OTA status to the Device Shadow.
 *
 * Owns exactly one shadow field: the lowercase "status"
 * (idle / installing / success / failed). It deliberately does NOT write
 * "fw_version" — that field is owned solely by ota_report_status() and means
 * "the version actually running", so the two functions never collide. */
static void ota_report_shadow(const char *status) {
    char body[96];

    snprintf(body, sizeof(body),
             "{\"state\":{\"reported\":{\"status\":\"%s\"}}}", status);

    printf(" > OTA_SVC:SHADOW %s\r\n", body);
    ota_mqtt_publish(s_topic_shadow, body);
}

/* ==========================================================================
 * Device status report
 *  One-shot snapshot published to the shadow on the first successful connect:
 *  firmware / chip / network identity, uptime and the last OTA outcome.
 * ======================================================================= */

/* OTA record — persisted in a dedicated flash sector (FLASH_OTA_INFO_ADDR) so
 * it survives the reboot the OTA path performs. It drives a two-phase commit:
 *
 *   Phase 1 (old fw):  download + verify + arm bootloader -> store PENDING ->
 *                      reboot.  SUCCEEDED is NOT reported here.
 *   Phase 2 (new fw):  this code is running, so the update worked -> report
 *                      SUCCEEDED and store DONE.
 *
 * Loaded into the RAM cache once at task start; an erased / never-written
 * sector (magic mismatch) means "no record" — every field reports "N/A". */
#define OTA_LAST_MAGIC  0x4F544132u        /* 'OTA2' — record validity marker */

#define OTA_REC_PENDING 1u                 /* flashed, awaiting boot confirm  */
#define OTA_REC_DONE    2u                 /* outcome finalized and reported  */

typedef struct {
    uint32_t magic;
    uint32_t state;                        /* OTA_REC_PENDING / OTA_REC_DONE  */
    uint32_t timestamp;                    /* AWS job timestamp (epoch sec)   */
    char     job_id[OTA_JOB_ID_MAX_LEN + 1];
    char     version[OTA_VERSION_MAX_LEN + 1];
    char     result[12];                   /* "pending"/"success"/"failed"    */
} ota_last_record_t;

static struct {
    int      valid;
    uint32_t state;
    char     job_id[OTA_JOB_ID_MAX_LEN + 1];
    char     version[OTA_VERSION_MAX_LEN + 1];
    char     result[12];
    uint32_t timestamp;
} s_last_ota;

/* Load the persisted OTA record from flash into the RAM cache. */
static void ota_last_load(void) {
    ota_last_record_t rec;

    read_flash(FLASH_OTA_INFO_ADDR, (uint8_t *)&rec, sizeof(rec));
    if (rec.magic != OTA_LAST_MAGIC) {
        s_last_ota.valid = 0;              /* erased / never written */
        printf(" > OTA_SVC:no last-OTA record\r\n");
        return;
    }
    rec.job_id[OTA_JOB_ID_MAX_LEN]       = '\0';
    rec.version[OTA_VERSION_MAX_LEN]     = '\0';
    rec.result[sizeof(rec.result) - 1]   = '\0';

    s_last_ota.valid     = 1;
    s_last_ota.state     = rec.state;
    s_last_ota.timestamp = rec.timestamp;
    strncpy(s_last_ota.job_id,  rec.job_id,  sizeof(s_last_ota.job_id));
    strncpy(s_last_ota.version, rec.version, sizeof(s_last_ota.version));
    strncpy(s_last_ota.result,  rec.result,  sizeof(s_last_ota.result));
    s_last_ota.job_id[sizeof(s_last_ota.job_id) - 1]   = '\0';
    s_last_ota.version[sizeof(s_last_ota.version) - 1] = '\0';
    s_last_ota.result[sizeof(s_last_ota.result) - 1]   = '\0';
    printf(" > OTA_SVC:last-OTA loaded (state=%lu %s %s)\r\n",
           (unsigned long)s_last_ota.state, s_last_ota.job_id,
           s_last_ota.result);
}

/* Persist the OTA record to flash and refresh the RAM cache. */
static void ota_last_store(uint32_t state, const char *job_id,
                           const char *version, const char *result,
                           uint32_t timestamp) {
    ota_last_record_t rec;

    memset(&rec, 0, sizeof(rec));
    rec.magic     = OTA_LAST_MAGIC;
    rec.state     = state;
    rec.timestamp = timestamp;
    strncpy(rec.job_id,  job_id  ? job_id  : "", OTA_JOB_ID_MAX_LEN);
    strncpy(rec.version, version ? version : "", OTA_VERSION_MAX_LEN);
    strncpy(rec.result,  result  ? result  : "", sizeof(rec.result) - 1);

    write_flash(FLASH_OTA_INFO_ADDR, (uint8_t *)&rec, sizeof(rec));
    printf(" > OTA_SVC:last-OTA stored (state=%lu %s %s)\r\n",
           (unsigned long)state, rec.job_id, rec.result);

    s_last_ota.valid     = 1;
    s_last_ota.state     = state;
    s_last_ota.timestamp = timestamp;
    strncpy(s_last_ota.job_id,  rec.job_id,  sizeof(s_last_ota.job_id));
    strncpy(s_last_ota.version, rec.version, sizeof(s_last_ota.version));
    strncpy(s_last_ota.result,  rec.result,  sizeof(s_last_ota.result));
    s_last_ota.job_id[sizeof(s_last_ota.job_id) - 1]   = '\0';
    s_last_ota.version[sizeof(s_last_ota.version) - 1] = '\0';
    s_last_ota.result[sizeof(s_last_ota.result) - 1]   = '\0';
}

/* Phase 2 of the two-phase OTA commit. The download path (Phase 1) flashes the
 * image, stores a PENDING record and reboots WITHOUT reporting SUCCEEDED. If
 * execution reaches here the new firmware has booted and is running this very
 * code — proof the update worked — so finalize the job: report SUCCEEDED and
 * mark the record DONE. No-op unless a PENDING record is waiting; idempotent
 * (once DONE it does nothing on later reconnects).
 *
 * @return 1 if a pending OTA was confirmed, 0 if there was nothing to do. */
static int ota_confirm_pending(void) {
    if (!s_last_ota.valid || s_last_ota.state != OTA_REC_PENDING) {
        return 0;
    }
    printf(" > OTA_SVC:post-boot confirm job=%s v=%s\r\n",
           s_last_ota.job_id, s_last_ota.version);
    ota_report_job(s_last_ota.job_id, "SUCCEEDED", NULL, s_last_ota.version);
    ota_report_shadow("success");
    ota_last_store(OTA_REC_DONE, s_last_ota.job_id, s_last_ota.version,
                   "success", s_last_ota.timestamp);
    return 1;
}

static void ota_report_status(void) {
    uint8_t mac[6] = {0};
    uint8_t ip[4]  = {0};
    char    last_ota[160];
    char    body[384];

    getSHAR(mac);
    getSIPR(ip);

    if (s_last_ota.valid) {
        snprintf(last_ota, sizeof(last_ota),
                 "{\"job_id\":\"%s\",\"result\":\"%s\",\"timestamp\":%lu}",
                 s_last_ota.job_id, s_last_ota.result,
                 (unsigned long)s_last_ota.timestamp);
    } else {
        snprintf(last_ota, sizeof(last_ota),
                 "{\"job_id\":\"N/A\",\"result\":\"N/A\",\"timestamp\":\"N/A\"}");
    }

    /* Note: this snapshot does NOT write "status" — the live OTA status is
     * owned by ota_report_shadow(). It writes the device-identity fields and
     * "fw_version" (the version actually running). */
    snprintf(body, sizeof(body),
             "{\"state\":{\"reported\":{"
             "\"fw_version\":\"%d.%d.%d\","
             "\"chip\":\"%s\","
             "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
             "\"ip\":\"%d.%d.%d.%d\","
             "\"uptime_sec\":%lu,"
             "\"last_ota\":%s"
             "}}}",
             MAJOR_VER, MINOR_VER, MAINTENANCE_VER,
             OTA_CFG_CHIP_NAME,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             ip[0], ip[1], ip[2], ip[3],
             (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
             last_ota);

    printf(" > OTA_SVC:STATUS %s\r\n", body);
    ota_mqtt_publish(s_topic_shadow, body);
}

/* Lightweight job-document peek: the few fields the service needs to decide
 * what to do, before the download engine parses the rest. */
typedef struct {
    char job_id[OTA_JOB_ID_MAX_LEN + 1];
    char status[16];                       /* "QUEUED" / "IN_PROGRESS"  */
    char version[OTA_VERSION_MAX_LEN + 1]; /* target firmware version   */
} ota_job_peek_t;

/* Copy one JSON string value into out[] (NUL-terminated). 0 on success. */
static int ota_json_str(const uint8_t *doc, uint32_t len,
                        const char *query, char *out, size_t out_sz) {
    char *val;
    size_t val_len;
    if (JSON_Search((const char *)doc, len, query, strlen(query),
                    &val, &val_len) != JSONSuccess) {
        return -1;
    }
    if (val_len == 0 || val_len >= out_sz) {
        return -1;
    }
    memcpy(out, val, val_len);
    out[val_len] = '\0';
    return 0;
}

/* Extract jobId (required), status and target version (best-effort). */
static int ota_peek_job(const uint8_t *doc, uint32_t len, ota_job_peek_t *p) {
    memset(p, 0, sizeof(*p));

    if (JSON_Validate((const char *)doc, len) != JSONSuccess) {
        return -1;
    }
    if (ota_json_str(doc, len, "execution.jobId",
                     p->job_id, sizeof(p->job_id)) != 0) {
        return -1;   /* no jobId -> not an actionable job */
    }
    /* status / version are best-effort — left empty if absent. */
    ota_json_str(doc, len, "execution.status",
                 p->status, sizeof(p->status));
    ota_json_str(doc, len, "execution.jobDocument.firmware.version",
                 p->version, sizeof(p->version));
    return 0;
}

/* ==========================================================================
 * Job pipeline — download + verify + flash + report  (blocking)
 * ======================================================================= */

static void ota_handle_job(void) {
    ota_job_peek_t  peek;
    ota_job_info_t  info;
    ota_dl_result_t r;
    char     dev_ver[OTA_VERSION_MAX_LEN + 1];
    uint32_t job_ts = 0;

    if (ota_peek_job(s_job_doc, s_job_doc_len, &peek) != 0) {
        printf(" > OTA_SVC:job has no jobId — ignored\r\n");
        return;
    }

    /* AWS stamps every Jobs message with a root "timestamp" (epoch seconds);
     * the device has no RTC, so this is the only real wall-clock it sees. */
    {
        char ts_buf[24];
        if (ota_json_str(s_job_doc, s_job_doc_len, "timestamp",
                         ts_buf, sizeof(ts_buf)) == 0) {
            job_ts = (uint32_t)strtoul(ts_buf, NULL, 10);
        }
    }

    snprintf(dev_ver, sizeof(dev_ver), "%d.%d.%d",
             MAJOR_VER, MINOR_VER, MAINTENANCE_VER);
    printf(" > OTA_SVC:JOB id=%s status=%s target=%s device=%s\r\n",
           peek.job_id, peek.status, peek.version, dev_ver);

    /* --- already finalized by us? (dedup / lost-report recovery) -----------
     * AWS only re-delivers a job that is still QUEUED / IN_PROGRESS on its
     * side. If our flash record shows this exact job is already DONE, our
     * terminal report was lost in transit — just re-send it (idempotent). */
    if (s_last_ota.valid && s_last_ota.state == OTA_REC_DONE &&
        strcmp(peek.job_id, s_last_ota.job_id) == 0) {
        printf(" > OTA_SVC:job already finalized (%s) — re-reporting\r\n",
               s_last_ota.result);
        if (strcmp(s_last_ota.result, "success") == 0) {
            ota_report_job(peek.job_id, "SUCCEEDED", NULL, s_last_ota.version);
        } else {
            ota_report_job(peek.job_id, "FAILED", "PREV_FAILED", NULL);
        }
        return;
    }

    /* --- a job we flashed but have not confirmed yet -----------------------
     * Normally confirmed at connect time; this is a guard for a job that is
     * delivered before ota_confirm_pending() ran. */
    if (s_last_ota.valid && s_last_ota.state == OTA_REC_PENDING &&
        strcmp(peek.job_id, s_last_ota.job_id) == 0) {
        ota_confirm_pending();
        return;
    }

    /* --- safety net: stuck IN_PROGRESS job, device already on target -------
     * No matching flash record (e.g. wiped) but the device already runs the
     * target version — the OTA effectively succeeded; report it without
     * re-downloading (which would otherwise loop). */
    if (strcmp(peek.status, "IN_PROGRESS") == 0 &&
        peek.version[0] != '\0' &&
        strcmp(peek.version, dev_ver) == 0) {
        printf(" > OTA_SVC:already running v%s — reporting SUCCEEDED\r\n",
               dev_ver);
        ota_report_job(peek.job_id, "SUCCEEDED", NULL, dev_ver);
        ota_report_shadow("idle");
        ota_last_store(OTA_REC_DONE, peek.job_id, dev_ver, "success", job_ts);
        return;
    }

    /* --- genuinely new job ------------------------------------------------
     * Steps 1-2: download + verify + flash + arm the bootloader. NOTHING is
     * reported during this — the job stays QUEUED on AWS while the image is
     * being fetched. IN_PROGRESS is reported only once the image is in flash,
     * meaning "downloaded, now applying" (reported below).
     *
     * Blocking: parse -> HTTPS download -> SHA256 -> Bank1 -> arm bootloader.
     * The MQTT keep-alive (OTA_CFG_KEEPALIVE_SEC) is long enough to span it. */
    memset(&info, 0, sizeof(info));
    r = ota_download_apply(s_job_doc, s_job_doc_len, &info);

    if (r == OTA_DL_SUCCESS) {
        /* The image is flashed and verified and the bootloader is armed — but
         * the new firmware has NOT run yet, so SUCCEEDED is NOT reported here.
         *   Step 3: report Job IN_PROGRESS ("downloaded, applying").
         *   Step 4: shadow "installing" — the last state the server hears
         *           before the bank copy + reboot; if it sticks here the
         *           apply failed.
         *   Step 5: persist the PENDING flag for the next boot to confirm.
         *   Step 6: reboot — Phase 2 (ota_confirm_pending) reports SUCCEEDED. */
        printf(" > OTA_SVC:JOB downloaded — installing, rebooting\r\n");
        ota_report_job(info.job_id, "IN_PROGRESS", NULL, NULL);
        ota_report_shadow("installing");
        ota_last_store(OTA_REC_PENDING, info.job_id, info.version,
                       "pending", job_ts);
        vTaskDelay(pdMS_TO_TICKS(1000));   /* let the publishes flush */
        device_reboot();
        /* not reached */
    } else {
        /* Failure during download / verify: the job is still QUEUED on AWS,
         * so report FAILED directly (QUEUED -> FAILED is a valid transition). */
        const char *reason = ota_reason_str(r);
        printf(" > OTA_SVC:JOB FAILED — %s\r\n", reason);
        ota_report_job(peek.job_id, "FAILED", reason, NULL);
        ota_report_shadow("failed");
        ota_last_store(OTA_REC_DONE, peek.job_id, peek.version,
                       "failed", job_ts);
    }
}

/* ==========================================================================
 * Connection bring-up
 * ======================================================================= */

/*
 * DNS -> socket -> TLS handshake -> MQTT CONNECT -> SUBSCRIBE -> $next/get.
 * @return 0 on success, -1 on failure (caller cleans up and retries)
 */
static int ota_service_connect(void) {
    uint8_t ip[4] = {0};
    bool session_present;
    int ret;

    /* --- DNS --- */
    printf(" > OTA_SVC:Resolving %s\r\n", OTA_CFG_ENDPOINT);
    if (get_ipaddr_from_dns((uint8_t *)OTA_CFG_ENDPOINT, ip, 5000) != 1) {
        printf(" > OTA_SVC:DNS FAILED\r\n");
        return -1;
    }
    printf(" > OTA_SVC:DNS %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);

    /* --- TLS --- */
    printf(" > OTA_SVC:HEAP before TLS = %d\r\n", (int)xPortGetFreeHeapSize());
    if (ota_tls_init(&s_ota_tls, OTA_CFG_MQTT_SOCK) <= 0) {
        printf(" > OTA_SVC:TLS init FAILED\r\n");
        return -1;
    }
    if (wiz_tls_socket(&s_ota_tls, OTA_CFG_MQTT_SOCK, 0) != OTA_CFG_MQTT_SOCK) {
        printf(" > OTA_SVC:socket open FAILED\r\n");
        return -1;
    }
    if (connect(OTA_CFG_MQTT_SOCK, ip, OTA_CFG_PORT) != SOCK_OK) {
        printf(" > OTA_SVC:TCP connect FAILED\r\n");
        return -1;
    }
    if (wiz_tls_connect(&s_ota_tls, OTA_CFG_ENDPOINT, OTA_CFG_PORT) != 0) {
        printf(" > OTA_SVC:TLS handshake FAILED\r\n");
        return -1;
    }
    printf(" > OTA_SVC:HEAP after TLS  = %d\r\n", (int)xPortGetFreeHeapSize());
    printf(" > OTA_SVC:TLS CONNECTED\r\n");

    /* --- coreMQTT init --- */
    s_ota_net.socketDescriptor       = OTA_CFG_MQTT_SOCK;
    s_ota_transport.pNetworkContext  = &s_ota_net;
    s_ota_transport.send             = ota_tls_send;
    s_ota_transport.recv             = ota_tls_recv;

    s_ota_mqtt.mqtt_fixed_buf.pBuffer = s_ota_recv_buf;
    s_ota_mqtt.mqtt_fixed_buf.size    = sizeof(s_ota_recv_buf);
    s_ota_mqtt.subscribe_count        = 0;

    ret = MQTT_Init(&s_ota_mqtt.mqtt_context, &s_ota_transport,
                    (MQTTGetCurrentTimeFunc_t)xTaskGetTickCount,
                    ota_mqtt_event_cb, &s_ota_mqtt.mqtt_fixed_buf);
    if (ret != MQTTSuccess) {
        printf(" > OTA_SVC:MQTT_Init FAILED %d\r\n", ret);
        return -1;
    }
    ret = MQTT_InitStatefulQoS(&s_ota_mqtt.mqtt_context,
                               s_ota_incoming, OTA_SVC_QOS_RECORD_MAX,
                               s_ota_outgoing, OTA_SVC_QOS_RECORD_MAX);
    if (ret != MQTTSuccess) {
        printf(" > OTA_SVC:InitStatefulQoS FAILED %d\r\n", ret);
        return -1;
    }

    /* --- MQTT CONNECT --- */
    memset(&s_ota_mqtt.mqtt_connect_info, 0, sizeof(s_ota_mqtt.mqtt_connect_info));
    s_ota_mqtt.mqtt_connect_info.cleanSession          = true;
    s_ota_mqtt.mqtt_connect_info.pClientIdentifier     = OTA_CFG_CLIENT_ID;
    s_ota_mqtt.mqtt_connect_info.clientIdentifierLength = strlen(OTA_CFG_CLIENT_ID);
    s_ota_mqtt.mqtt_connect_info.keepAliveSeconds      = OTA_CFG_KEEPALIVE_SEC;

    ret = MQTT_Connect(&s_ota_mqtt.mqtt_context, &s_ota_mqtt.mqtt_connect_info,
                       NULL, MQTT_TIMEOUT, &session_present);
    if (ret != MQTTSuccess) {
        printf(" > OTA_SVC:MQTT_Connect FAILED %d\r\n", ret);
        return -1;
    }
    printf(" > OTA_SVC:MQTT CONNECTED (client=%s)\r\n", OTA_CFG_CLIENT_ID);

    /* --- SUBSCRIBE --- */
    if (ota_mqtt_subscribe(s_topic_notify) != 0)      return -1;
    if (ota_mqtt_subscribe(s_topic_nextget_acc) != 0) return -1;

    /* --- active poll: ask for the next queued job --- */
    ota_mqtt_publish(s_topic_nextget, "{}");
    printf(" > OTA_SVC:PUBLISH %s\r\n", s_topic_nextget);

    /* Phase 2 of the two-phase OTA commit: if the previous boot flashed an
     * image and left a PENDING record, the fact that this code is running
     * confirms the update worked — report SUCCEEDED + shadow "success".
     * On a normal boot (nothing pending) just settle the shadow to "idle". */
    if (ota_confirm_pending() == 0) {
        ota_report_shadow("idle");
    }

    /* One-shot device status report on the first successful connect. */
    {
        static int s_status_reported = 0;
        if (!s_status_reported) {
            s_status_reported = 1;
            ota_report_status();
        }
    }

    /* === TEMPORARY: one-shot cleanup of stale shadow fields ===============
     * Nulling a reported field deletes it from the shadow document. The
     * capital-S "Status" is a ghost from before the field was renamed to the
     * lowercase "status". Remove this block once that ghost is gone. */
    {
        static int s_shadow_cleaned = 0;
        if (!s_shadow_cleaned) {
            s_shadow_cleaned = 1;
            const char *cleanup =
                "{\"state\":{\"reported\":{\"Status\":null}}}";
            printf(" > OTA_SVC:SHADOW CLEANUP %s\r\n", cleanup);
            ota_mqtt_publish(s_topic_shadow, cleanup);
        }
    }
    /* === end TEMPORARY ==================================================== */

    return 0;
}

/* ==========================================================================
 * Task
 * ======================================================================= */

void ota_service_task(void *argument) {
    (void)argument;
    int ret;

    /* Build runtime topic strings once. */
    snprintf(s_topic_notify,      sizeof(s_topic_notify),
             OTA_CFG_TOPIC_NOTIFY_NEXT,   OTA_CFG_THING_NAME);
    snprintf(s_topic_nextget,     sizeof(s_topic_nextget),
             OTA_CFG_TOPIC_NEXT_GET,      OTA_CFG_THING_NAME);
    snprintf(s_topic_nextget_acc, sizeof(s_topic_nextget_acc),
             OTA_CFG_TOPIC_NEXT_GET_ACC,  OTA_CFG_THING_NAME);
    snprintf(s_topic_shadow,      sizeof(s_topic_shadow),
             OTA_CFG_TOPIC_SHADOW_UPDATE, OTA_CFG_THING_NAME);

    /* Load the persisted last-OTA record (reported in the status snapshot). */
    ota_last_load();

    /* Give DHCP / link a moment to settle before the first DNS query. */
    vTaskDelay(pdMS_TO_TICKS(8000));

    for (;;) {
        if (ota_service_connect() == 0) {
            /* Connected — run the MQTT yield loop until a fatal error. */
            for (;;) {
                ret = MQTT_ProcessLoop(&s_ota_mqtt.mqtt_context);
                /* 7 = MQTTNoDataAvailable, 11 = MQTTNeedMoreBytes: benign. */
                if (ret != MQTTSuccess && ret != 7 && ret != 11) {
                    printf(" > OTA_SVC:ProcessLoop error %d — reconnecting\r\n", ret);
                    break;
                }
                /* A job arrived in the callback — run the (blocking) pipeline
                 * here, outside MQTT_ProcessLoop. On success it reboots. */
                if (s_job_pending) {
                    s_job_pending = 0;
                    ota_handle_job();
                }
                vTaskDelay(pdMS_TO_TICKS(OTA_SVC_YIELD_MS));
            }
        }

        /* Failed / dropped — tear down and retry. */
        wiz_tls_deinit(&s_ota_tls);
        close(OTA_CFG_MQTT_SOCK);
        printf(" > OTA_SVC:retry in %d ms\r\n", OTA_SVC_RETRY_MS);
        vTaskDelay(pdMS_TO_TICKS(OTA_SVC_RETRY_MS));
    }
}
