#ifndef _OTA_DOWNLOAD_H_
#define _OTA_DOWNLOAD_H_

#include <stdint.h>

/*
 * ============================================================================
 * OTA firmware download / flash engine
 * ----------------------------------------------------------------------------
 * Pure firmware-image engine: given an AWS IoT Jobs document it parses the
 * firmware URL, streams the image over HTTPS into Bank1, verifies its SHA-256
 * and arms the bootloader copy flag.
 *
 * Performs NO MQTT and owns no cloud messaging. The standalone OTA service
 * (otaService.c) is the ONLY caller — it receives jobs over MQTT, drives this
 * engine, then reports status and reboots. Dependency direction is strictly
 * one-way: otaService.c -> otaDownload.h. This module never references it back.
 * ============================================================================
 */

#define OTA_JOB_ID_MAX_LEN   64
#define OTA_VERSION_MAX_LEN  32

/* W5500 socket dedicated to the HTTPS firmware download. */
#define SOCK_OTA_HTTP        7

/* Result of ota_download_apply(). */
typedef enum {
    OTA_DL_SUCCESS = 0,
    OTA_DL_ERR_PARSE,      /* job document malformed / fields missing      */
    OTA_DL_ERR_SIZE,       /* firmware size zero or larger than a bank     */
    OTA_DL_ERR_DOWNLOAD,   /* HTTPS download failed or returned short      */
    OTA_DL_ERR_SHA256,     /* SHA-256 of the downloaded image mismatched   */
    OTA_DL_ERR_FLASH       /* Bank1 first-word sanity check failed         */
} ota_dl_result_t;

/* Job fields the caller needs for status reporting. Filled as soon as the
 * job document is parsed, so it is valid for every result except
 * OTA_DL_ERR_PARSE. */
typedef struct {
    char     job_id[OTA_JOB_ID_MAX_LEN + 1];
    char     version[OTA_VERSION_MAX_LEN + 1];
    uint32_t fw_size;
} ota_job_info_t;

/**
 * @brief Parse a Jobs document, download + verify the firmware and arm the
 *        bootloader. Blocking; runs to completion in the caller's task.
 *
 * On OTA_DL_SUCCESS the bootloader copy flag is set — the caller should
 * report SUCCEEDED and reboot. On any error nothing is armed; the caller
 * should report FAILED with the matching reason.
 *
 * @param job_doc      raw notify-next / $next/get JSON payload
 * @param job_doc_len  length of job_doc
 * @param info_out     [out] parsed job id / version / size (may be NULL)
 * @return OTA_DL_SUCCESS or an OTA_DL_ERR_* code
 */
ota_dl_result_t ota_download_apply(const uint8_t *job_doc, uint32_t job_doc_len,
                                   ota_job_info_t *info_out);

#endif /* _OTA_DOWNLOAD_H_ */
