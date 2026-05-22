/*
 * ============================================================================
 * OTA firmware download / flash engine
 * ----------------------------------------------------------------------------
 * Pure image engine — see otaDownload.h. No MQTT, no cloud messaging.
 * Sole caller: otaService.c.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

#include "common.h"
#include "port_common.h"
#include "ConfigData.h"
#include "deviceHandler.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "otaDownload.h"

#include "socket.h"

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
#define OTA_RET_SUCCESS         0
#define OTA_RET_FAILED          (-1)

#define OTA_URL_MAX_LEN         512
#define OTA_SHA256_HEX_LEN      64   /* 32 bytes * 2 hex chars */

#define OTA_HTTP_BUF_SIZE       DATA_BUF_SIZE
#define OTA_FLASH_SECTOR_SIZE   FLASH_SECTOR_SIZE

/* -------------------------------------------------------------------------
 * Internal state — populated by ota_parse_job_document()
 * ---------------------------------------------------------------------- */
static char     s_job_id[OTA_JOB_ID_MAX_LEN + 1];
static char     s_fw_url[OTA_URL_MAX_LEN + 1];
static char     s_fw_version[OTA_VERSION_MAX_LEN + 1];
static uint32_t s_fw_size;
static char     s_fw_sha256[OTA_SHA256_HEX_LEN + 1];
static uint8_t  s_download_sha256[32]; /* SHA256 computed while streaming */

/* TLS context for the HTTPS firmware download. */
static wiz_tls_context s_ota_tls_ctx;

/* -------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------- */
static int ota_parse_job_document(const uint8_t *payload, uint32_t len);
static int ota_parse_url(const char *url, char *host_out, size_t host_max,
                         char *path_out, size_t path_max);
static int ota_download_and_flash(void);
static int ota_verify_sha256(uint32_t fw_size, const char *expected_sha256_hex);
static int hex_char_to_nibble(char c);
static int sha256_hex_to_bytes(const char *hex, uint8_t *out, size_t out_len);

/* =========================================================================
 * Public API
 * ====================================================================== */

ota_dl_result_t ota_download_apply(const uint8_t *job_doc, uint32_t job_doc_len,
                                   ota_job_info_t *info_out) {
    /* 1. Parse the Jobs document. */
    if (ota_parse_job_document(job_doc, job_doc_len) != OTA_RET_SUCCESS) {
        printf(" > OTA:PARSE:FAILED\r\n");
        return OTA_DL_ERR_PARSE;
    }

    if (info_out != NULL) {
        strncpy(info_out->job_id,  s_job_id,      OTA_JOB_ID_MAX_LEN);
        info_out->job_id[OTA_JOB_ID_MAX_LEN] = '\0';
        strncpy(info_out->version, s_fw_version,  OTA_VERSION_MAX_LEN);
        info_out->version[OTA_VERSION_MAX_LEN] = '\0';
        info_out->fw_size = s_fw_size;
    }

    printf(" > OTA:JOB_ID=%s\r\n", s_job_id);
    printf(" > OTA:URL=%s\r\n", s_fw_url);
    printf(" > OTA:SIZE=%lu\r\n", (unsigned long)s_fw_size);
    printf(" > OTA:SHA256=%s\r\n", s_fw_sha256);

    /* 2. Validate firmware size. */
    if (s_fw_size == 0 || s_fw_size > FLASH_APP_BANK_SIZE) {
        printf(" > OTA:SIZE:INVALID\r\n");
        return OTA_DL_ERR_SIZE;
    }

    /* 3. Stream the image over HTTPS into Bank1. */
    if (ota_download_and_flash() != OTA_RET_SUCCESS) {
        printf(" > OTA:DOWNLOAD:FAILED\r\n");
        return OTA_DL_ERR_DOWNLOAD;
    }

    /* 4. Verify the SHA-256 computed during the stream. */
    if (ota_verify_sha256(s_fw_size, s_fw_sha256) != OTA_RET_SUCCESS) {
        return OTA_DL_ERR_SHA256;
    }

    /* 5. Bank1 first-word sanity check. */
    {
        uint32_t bank1_word0 = *(volatile uint32_t *)(FLASH_START_ADDR_BANK1);
        printf(" > OTA:BANK1[0]=0x%08lX\r\n", (unsigned long)bank1_word0);
        if (bank1_word0 == 0x00000000 || bank1_word0 == 0xFFFFFFFF) {
            printf(" > OTA:BANK1:INVALID\r\n");
            return OTA_DL_ERR_FLASH;
        }
    }

    /* 6. Arm the bootloader — next boot copies Bank1 -> Bank0. */
    {
        struct __firmware_update *fwupdate =
            (struct __firmware_update *)&(get_DevConfig_pointer()->firmware_update);
        fwupdate->fwup_size      = s_fw_size;
        fwupdate->fwup_copy_flag = 1;
        save_DevConfig_to_storage();
    }

    printf(" > OTA:APPLY:Bootloader armed\r\n");
    return OTA_DL_SUCCESS;
}

/* =========================================================================
 * Job document parsing
 * ====================================================================== */

/*
 * Parse an AWS IoT Jobs payload (notify-next or $next/get/accepted):
 *
 * {
 *   "execution": {
 *     "jobId": "ota-...",
 *     "jobDocument": {
 *       "firmware": {
 *         "url": "https://...", "version": "2.2.2",
 *         "size": 524288, "sha256": "abcdef..."
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

    if (payload == NULL || len == 0) {
        return OTA_RET_FAILED;
    }

    json_ret = JSON_Validate((const char *)payload, len);
    if (json_ret != JSONSuccess) {
        printf(" > OTA:JSON:Invalid (%d)\r\n", json_ret);
        return OTA_RET_FAILED;
    }

    /* jobId */
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobId", sizeof("execution.jobId") - 1,
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

    /* firmware.version — optional */
    s_fw_version[0] = '\0';
    json_ret = JSON_Search((const char *)payload, len,
                           "execution.jobDocument.firmware.version",
                           sizeof("execution.jobDocument.firmware.version") - 1,
                           &val, &val_len);
    if (json_ret == JSONSuccess && val_len > 0 && val_len <= OTA_VERSION_MAX_LEN) {
        memcpy(s_fw_version, val, val_len);
        s_fw_version[val_len] = '\0';
    }

    return OTA_RET_SUCCESS;
}

/* =========================================================================
 * HTTPS download -> Bank1
 * ====================================================================== */

/* Extract host and path from https://host/path?query */
static int ota_parse_url(const char *url, char *host_out, size_t host_max,
                         char *path_out, size_t path_max) {
    const char *p = url;
    const char *scheme_end = strstr(p, "://");
    if (scheme_end == NULL) return -1;
    p = scheme_end + 3;

    const char *host_end = strchr(p, '/');
    if (host_end == NULL) {
        if ((size_t)strlen(p) >= host_max) return -1;
        strcpy(host_out, p);
        strcpy(path_out, "/");
        return 0;
    }

    size_t host_len = (size_t)(host_end - p);
    if (host_len >= host_max) return -1;
    memcpy(host_out, p, host_len);
    host_out[host_len] = '\0';

    size_t path_len = strlen(host_end);
    if (path_len >= path_max) return -1;
    memcpy(path_out, host_end, path_len);
    path_out[path_len] = '\0';
    return 0;
}

static int ota_download_and_flash(void) {
    char host_str[128];
    char path_str[OTA_URL_MAX_LEN];

    if (ota_parse_url(s_fw_url, host_str, sizeof(host_str),
                      path_str, sizeof(path_str)) != 0) {
        printf(" > OTA:URL:Parse FAILED\r\n");
        return OTA_RET_FAILED;
    }
    printf(" > OTA:HTTPS:Host=%s\r\n", host_str);

    /* DNS resolve the S3 hostname. */
    uint8_t s3_ip[4] = {0};
    printf(" > OTA:DNS:Resolving %s\r\n", host_str);
    if (get_ipaddr_from_dns((uint8_t *)host_str, s3_ip, 5000) != 1) {
        printf(" > OTA:DNS:FAILED\r\n");
        return OTA_RET_FAILED;
    }
    printf(" > OTA:DNS:Resolved %d.%d.%d.%d\r\n",
           s3_ip[0], s3_ip[1], s3_ip[2], s3_ip[3]);

    uint8_t sock = (uint8_t)SOCK_OTA_HTTP;
    int ret = wiz_tls_init(&s_ota_tls_ctx, (int *)sock);
    if (ret <= 0) {
        printf(" > OTA:TLS:Init FAILED (%d)\r\n", ret);
        return OTA_RET_FAILED;
    }

    ret = wiz_tls_socket(&s_ota_tls_ctx, sock, 0);
    if (ret != (int)sock) {
        printf(" > OTA:TLS:Socket FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    ret = connect(sock, s3_ip, 443);
    if (ret != SOCK_OK) {
        printf(" > OTA:TCP:Connect FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    ret = wiz_tls_connect(&s_ota_tls_ctx, host_str, 443);
    if (ret != 0) {
        printf(" > OTA:TLS:Connect FAILED (%d)\r\n", ret);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }
    printf(" > OTA:TLS:Connected\r\n");

    /* HTTP GET */
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

    uint8_t *http_buf = pvPortMalloc(OTA_HTTP_BUF_SIZE);
    if (http_buf == NULL) {
        printf(" > OTA:MALLOC:FAILED\r\n");
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

    /* Read until end of HTTP headers (double CRLF). */
    uint32_t header_buf_len = 0;
    int header_done = 0;
    while (!header_done) {
        int n = (int)wiz_tls_read(&s_ota_tls_ctx, http_buf + header_buf_len, 1);
        if (n <= 0) break;
        header_buf_len += n;
        if (header_buf_len >= 4 &&
            memcmp(http_buf + header_buf_len - 4, "\r\n\r\n", 4) == 0) {
            header_done = 1;
        }
        if (header_buf_len >= OTA_HTTP_BUF_SIZE - 1) break;
    }

    if (!header_done) {
        printf(" > OTA:HTTP:Headers incomplete\r\n");
        vPortFree(http_buf);
        wiz_tls_deinit(&s_ota_tls_ctx);
        close(sock);
        return OTA_RET_FAILED;
    }

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

    /* Stream the body straight into Bank1. */
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

    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts(&sha_ctx, 0);

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

        mbedtls_sha256_update(&sha_ctx, http_buf, n);

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

    if (download_ok && buf_fill > 0) {
        printf(" > OTA:FLASH:0x%08lX (last %lu bytes)\r\n",
               (unsigned long)f_addr, (unsigned long)buf_fill);
        write_flash(f_addr, sector_buf, OTA_FLASH_SECTOR_SIZE);
    }

    mbedtls_sha256_finish(&sha_ctx, s_download_sha256);
    mbedtls_sha256_free(&sha_ctx);

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

/* =========================================================================
 * SHA-256 verification
 * ====================================================================== */

static int ota_verify_sha256(uint32_t fw_size, const char *expected_sha256_hex) {
    uint8_t expected[32];
    (void)fw_size;

    if (sha256_hex_to_bytes(expected_sha256_hex, expected, sizeof(expected)) != 0) {
        printf(" > OTA:SHA256:Invalid hex string\r\n");
        return OTA_RET_FAILED;
    }

    /* Compare against the hash computed while streaming (not a flash readback). */
    if (memcmp(s_download_sha256, expected, 32) != 0) {
        printf(" > OTA:SHA256:MISMATCH\r\n");
        printf(" > OTA:SHA256:Expected=%s\r\n", expected_sha256_hex);
        printf(" > OTA:SHA256:Computed=");
        for (int i = 0; i < 32; i++) printf("%02x", s_download_sha256[i]);
        printf("\r\n");
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
