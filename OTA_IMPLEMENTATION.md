# OTA 변경 내역 (W55RP20-S2E)

AWS IoT Jobs 기반 OTA 펌웨어 업데이트 기능 도입과 그에 이은 안정성 개선 작업을 정리한 문서.

- **베이스 커밋**: `dc16715` — *OTA update* (OTA 기반 구축)
- **이후 추가 수정**: 다운로드 중 SHA256 스트리밍 검증, AWS Device Shadow 보고, Bank1 사전 검사, 디버그용 heap 모니터링 태스크, 빌드 환경 정비

---

## 1. 전체 동작 개요

1. MQTT 연결 직후 `ota_init()` 호출 → AWS IoT Jobs 토픽 구독.
2. 클라우드가 Job을 publish하면 `$aws/things/<thing>/jobs/notify-next` 수신.
3. coreJSON으로 Job document 파싱 (URL / size / SHA256 / version).
4. 별도 `OTA_TASK`가 세마포어로 깨어나 HTTPS(S3 presigned URL)로 펌웨어 다운로드.
5. 다운로드 청크를 그대로 **Bank1 flash**에 sector 단위로 streaming write.
6. **다운로드 중 동시에 SHA256 계산** → 완료 후 Job document의 expected SHA256과 비교.
7. Bank1 첫 워드가 valid한지 사전 확인.
8. `firmware_update.fwup_copy_flag = 1` 설정 후 저장 → reboot.
9. 부트로더가 Bank1 → Bank0 복사 후 새 펌웨어 부팅.
10. Jobs / Device Shadow에 `SUCCEEDED` / `FAILED` 상태 보고.

```
[AWS IoT Jobs] ──notify-next──▶ MQTT CB ──ota_mqtt_handle()──▶ [OTA_TASK]
                                                                   │
                                                  ┌────────────────┴────────────────┐
                                                  ▼                                 ▼
                                       HTTPS GET (S3 presigned URL)         streaming SHA256
                                                  │                                 │
                                                  ▼                                 │
                                       Bank1 flash (sector write) ◀─────────────────┘
                                                  │
                                                  ▼
                                        SHA256 verify + Bank1 sanity
                                                  │
                                                  ▼
                                        fwup_copy_flag=1 → reboot → 부트로더가 Bank1→Bank0
                                                  │
                                                  ▼
                                       Jobs UPDATE + Shadow UPDATE
```

---

## 2. 신규 파일

### 2-1. `port/app/platform_handler/inc/otaHandler.h`

OTA 모듈 공개 API와 토픽 매크로.

```c
#define OTA_BROKER_ENDPOINT "a3uz5t2azg1xdz-ats.iot.ap-northeast-2.amazonaws.com"
#define OTA_BROKER_PORT     8883
#define OTA_TOPIC_BUF_SIZE  256

#define OTA_NOTIFY_TOPIC_FMT        "$aws/things/%s/jobs/notify-next"
#define OTA_JOB_UPDATE_TOPIC_FMT    "$aws/things/%s/jobs/%s/update"
#define OTA_SHADOW_UPDATE_TOPIC_FMT "$aws/things/%s/shadow/update"   /* 추가 */
#define OTA_PING_PUB_TOPIC_FMT      "$aws/things/%s/jobs/get"
#define OTA_PING_ACC_TOPIC_FMT      "$aws/things/%s/jobs/get/accepted"
#define OTA_PING_REJ_TOPIC_FMT      "$aws/things/%s/jobs/get/rejected"

#define SOCK_OTA_HTTP   7   /* HTTPS download 전용 소켓 */
```

공개 API:
| 함수 | 역할 |
| --- | --- |
| `int ota_init(void *mqtt_config)` | OTA 모듈 초기화. 토픽 구독, OTA 태스크 생성. |
| `int ota_ping(void)` | `$aws/things/<id>/jobs/get` 발행으로 클라우드 연결 확인. |
| `int ota_is_ota_topic(const char *topic, uint16_t topicLen)` | `$aws/things/` 접두사 여부 판단. |
| `void ota_mqtt_handle(...)` | OTA 메시지 라우팅 (ping accepted/rejected, job document). |

> Fleet Provisioning 적용 시 하드코딩 endpoint 제거 필요 (TODO 명시).

---

### 2-2. `port/app/platform_handler/src/otaHandler.c`

OTA 본체 구현. 주요 구성 요소:

#### (a) 내부 상태
- `s_mqtt_config`, `s_thing_name`
- 현재 작업 정보: `s_job_id`, `s_fw_url`, `s_fw_size`, `s_fw_sha256`
- TLS context `s_ota_tls_ctx`
- FreeRTOS 동기화: `s_ota_job_sem` (binary semaphore), `s_ota_task_handle`
- **추가**: `static uint8_t s_download_sha256[32];` — 다운로드 중 계산된 SHA256

#### (b) `ota_init()`
- `client_id`에서 Thing Name 복사.
- 토픽 문자열 생성 (`notify`, `ping acc/rej`).
- 중복 구독 방지(`ota_subscribe_topic_if_needed`)로 `notify` / `ping acc/rej` 구독.
- 최초 호출 시 `OTA_TASK` 생성 (stack 2048, `tskIDLE_PRIORITY+2`).

#### (c) Job document 파싱 (`ota_parse_job_document`)
coreJSON으로 다음 필드를 추출:
```
execution.jobId
execution.jobDocument.firmware.url
execution.jobDocument.firmware.size
execution.jobDocument.firmware.sha256
```
크기/길이 검증 후 정적 버퍼에 저장.

#### (d) HTTPS 다운로드 (`ota_download_and_flash`)
- `ota_parse_url()`로 https URL에서 host/path 분리.
- DNS resolve → `wiz_tls_init / socket / connect` → TLS handshake.
- 단순 `GET ... HTTP/1.1` + `Host` + `Connection: close` 요청.
- `\r\n\r\n`까지 헤더 읽고, status가 200인지 확인.
- 본문은 `DATA_BUF_SIZE` 단위로 stream 수신.
- `FLASH_SECTOR_SIZE` 채워질 때마다 `write_flash(Bank1 offset, ...)` 호출.
- **추가**: 수신 청크마다 `mbedtls_sha256_update(...)` 수행해 다운로드와 동시에 SHA256 계산. 완료 후 `s_download_sha256`에 저장.
- 워치독 enable 시 매 루프에 `device_wdt_reset()`.

#### (e) SHA256 검증 (`ota_verify_sha256`)
- 초기 구현은 Bank1 flash(XIP)에서 256바이트씩 다시 읽어 SHA256 재계산.
- **현재 구현**: 다운로드 단계에서 누적된 `s_download_sha256`을 expected와 직접 `memcmp`.
- 효과:
  - flash readback 부담 / XIP 타이밍 이슈 회피
  - 다운로드와 검증이 한 패스로 끝나 속도 향상
- 불일치 시 expected/computed 해시를 모두 로그로 덤프.

#### (f) Job 상태 보고 (`ota_report_status`)
**Jobs UPDATE 페이로드 분기:**
- `SUCCEEDED`:
  ```json
  {"status":"SUCCEEDED","statusDetails":{"version":"2.2.2"}}
  ```
- `reason != NULL`:
  ```json
  {"status":"<status>","statusDetails":{"reason":"<reason>"}}
  ```
- 그 외:
  ```json
  {"status":"<status>"}
  ```

**추가: Device Shadow UPDATE 동시 발행**

같은 함수에서 `$aws/things/<thing>/shadow/update`로도 QoS1 publish:
- 성공 시:
  ```json
  {"state":{"reported":{"FW Version":"2.2.2","Status":"idle"}}}
  ```
- 그 외:
  ```json
  {"state":{"reported":{"Status":"<status>"}}}
  ```
- 로그: `> OTA:SHADOW:<body>`

#### (g) OTA 메인 태스크 (`ota_task`)
무한 루프로 세마포어 대기 → 다음 시퀀스 수행:

1. `ota_report_status("IN_PROGRESS", NULL)` + `set_device_status(ST_UPGRADE)`
2. `ota_download_and_flash()` → 실패 시 `FAILED / DOWNLOAD_ERROR` 보고 후 `continue`
3. `ota_verify_sha256()` → 실패 시 `FAILED / SHA256_MISMATCH`
4. **Bank1 첫 워드 검사** (현재 구현 추가):
   ```c
   uint32_t bank1_word0 = *(volatile uint32_t *)(FLASH_START_ADDR_BANK1);
   if (bank1_word0 == 0x00000000 || bank1_word0 == 0xFFFFFFFF) {
       /* FAILED / INVALID_FLASH */
   }
   ```
   → 잘못된 Bank1로 부트로더가 진입해 벽돌이 되는 상황을 방지.
5. `firmware_update.fwup_size` 설정, `fwup_copy_flag = 1`, `save_DevConfig_to_storage()`
6. `SUCCEEDED` 보고 후 500ms 대기 → `device_reboot()`

#### (h) 유틸리티
- `hex_char_to_nibble`, `sha256_hex_to_bytes`
- `ota_parse_url` (간단한 https URL 파서)
- `ota_subscribe_topic_if_needed` (중복 구독 방지)
- `ota_copy_thing_name`

---

### 2-3. `port/app/configuration/inc/fw_info.h`

빌드된 펌웨어 바이너리에 임베드되는 정보 블록 정의 (OTA 사전 검증 및 운영 식별용).

```c
#define FW_INFO_MAGIC   0x57495A4E  /* 'WIZN' */
#define FW_INFO_OFFSET  0x100       /* 2nd stage bootloader 직후 */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  major, minor, patch, reserved;
    char     version_str[16];
    char     board[16];
    char     manufacturer[16];
    char     status[12];
    uint32_t build_date;   /* BCD */
} fw_info_t;
```

Flash map:
```
0x00000000  [0x100]  2nd stage bootloader
0x00000100  [0x100]  fw_info_t      ← 여기
0x00000200  [...]    vector table + code
```

---

### 2-4. `tools/test_ota_ping.py`

OTA ping 동작 검증용 호스트 스크립트 (`$aws/things/<id>/jobs/get` 발행 및 accepted/rejected 응답 확인용).

---

## 3. 기존 파일 수정

### 3-1. `port/app/AWS-IoT-Device-SDK-Embedded-C/coreMQTT/inc/core_mqtt_config.h`

| 항목 | 변경 전 | 변경 후 | 이유 |
| --- | --- | --- | --- |
| `MQTT_BUF_MAX_SIZE` | `1024 * 2` | `1024 * 4` | Job document 페이로드가 큼 |
| `MQTT_SUBSCRIPTION_MAX_NUM` | `3` | `6` | OTA 토픽 3종 추가 구독 |

### 3-2. `port/app/AWS-IoT-Device-SDK-Embedded-C/coreMQTT/src/mqtt_transport_interface.c`

- `mqtt_event_callback`:
  - 디버그 로그 `MQTT:CB:type=0x..` 추가.
  - publish 수신 시 `ota_is_ota_topic()` 결과에 따라 **OTA 토픽은 `ota_mqtt_handle()`, 그 외는 기존 `user_sub_callback()`** 으로 라우팅.
- `mqtt_transport_subscribe`:
  - `> MQTT_SUBSCRIPTION_MAX_NUM` → `>= MQTT_SUBSCRIPTION_MAX_NUM` (off-by-one 수정).
- `mqtts_read`:
  - `wiz_tls_read()` 반환값이 음수일 때 (`WANT_READ`) 0으로 정규화해 비정상 에러 처리 방지.

### 3-3. `port/app/configuration/inc/common.h`

| 항목 | 변경 |
| --- | --- |
| `MAX_HTTPSOCK` | `4` → `3` |
| `SOCK_HTTPSERVER_4` (소켓 7) | OTA HTTPS 다운로드 전용으로 예약 (HTTP 서버에서 미사용) |
| `MQTT_BUF_SIZE` | `2048` → `4096` |

### 3-4. `libraries/CMakeLists.txt`

`AWS_SDK_FILES`에 coreJSON 추가:
- 소스: `coreJSON/source/core_json.c`
- include: `coreJSON/source/include`

### 3-5. `port/app/CMakeLists.txt`

`APP_PLATFORM_FILES`에 다음 추가:
- 소스: `platform_handler/src/otaHandler.c`
- link: `AWS_SDK_FILES`

### 3-6. `port/app/serial_to_ethernet/src/seg.c`

- `otaHandler.h` include 추가.
- `proc_SEG_mqtt_client()` / `proc_SEG_mqtts_client()` 둘 다, MQTT 연결 및 구독 성공 직후:
  ```c
  /* Subscribe to OTA notification topic */
  ota_init(&g_mqtt_config);
  ```

### 3-7. `main/App/App.c`

- `start_task()` 진입 시 빌드 식별 로그:
  ```c
  PRT_INFO(" > Lihan`s New OTA Ver\r\n");
  ```
- **추가: Heap 모니터링 태스크** (현재 단계)
  ```c
  #define HEAP_MONITOR_TASK_STACK_SIZE 512
  #define HEAP_MONITOR_TASK_PRIORITY   9

  void heap_monitor_task(void *argument) {
      while (1) {
          printf("Free heap: %d\n", xPortGetFreeHeapSize());
          printf("Min free heap: %d\n", xPortGetMinimumEverFreeHeapSize());
          vTaskDelay(pdMS_TO_TICKS(100));
      }
  }
  ```
  `start_task()`에서 `xTaskCreate(heap_monitor_task, ...)` 등록.
  → OTA 다운로드 / TLS 핸드셰이크 동안 메모리 여유 추적용 디버그 태스크. 릴리즈 전 제거 또는 컨디셔널 컴파일 권장.

### 3-8. `port/app/html_file/Web_page.h`

- OTA UI 관련 대규모 갱신 (1712줄 변경).
- 헤더 주석 `data:` 필드를 `2026-04-21`까지 두 번 갱신.

---

## 4. 빌드 설정 변경 (`CMakeLists.txt` 루트)

### 4-1. 타겟 보드 전환
- `BOARD_NAME`: `W55RP20_S2E` → `PLATYPUS_S2E`

### 4-2. Windows Python launcher로 통일
- `html_to_c_header` 타깃:
  - `python ...` → `cmd /c "py ... & exit 0"` (스크립트 실패해도 빌드 계속)
- `merge_hex` 타깃: `python` → `py`
- `hex_to_uf2_converter` 타깃: `python` → `py`

### 4-3. `restyle` 타깃 비활성화
- `style/restyle.py` 자동 실행 custom target과 `add_dependencies(restyle ...)`를 모두 주석 처리.

---

## 5. AWS IoT Jobs Job document 포맷 (참고)

```json
{
  "execution": {
    "jobId": "ota-test-xxx",
    "jobDocument": {
      "operation": "ota_update",
      "firmware": {
        "url":     "https://<bucket>.s3.<region>.amazonaws.com/...?X-Amz-...",
        "version": "2.2.2",
        "size":    524288,
        "sha256":  "abcdef..."
      }
    }
  }
}
```

---

## 6. 리포팅 토픽 요약

| 시점 | 토픽 | 페이로드 |
| --- | --- | --- |
| 다운로드 시작 | `$aws/things/<id>/jobs/<jobId>/update` | `{"status":"IN_PROGRESS"}` |
| 다운로드 실패 | 〃 | `{"status":"FAILED","statusDetails":{"reason":"DOWNLOAD_ERROR"}}` |
| SHA256 실패 | 〃 | `{"status":"FAILED","statusDetails":{"reason":"SHA256_MISMATCH"}}` |
| Bank1 invalid | 〃 | `{"status":"FAILED","statusDetails":{"reason":"INVALID_FLASH"}}` |
| 성공 | 〃 | `{"status":"SUCCEEDED","statusDetails":{"version":"2.2.2"}}` |
| 모든 상태 변화 | `$aws/things/<id>/shadow/update` | `{"state":{"reported":{"Status":"<state>"}}}` (성공 시 `FW Version` 포함) |

---

## 7. 알려진 TODO / 후속 작업

- Fleet Provisioning 적용 후 `OTA_BROKER_ENDPOINT` 하드코딩 제거.
- 성공 시 페이로드의 `"version":"2.2.2"`가 상수로 박혀 있음 → `fw_info_t.version_str` 기반으로 동적 치환 필요.
- `heap_monitor_task`는 디버그 용도 — 릴리즈 빌드에서 제거 또는 `#ifdef` 가드.
- `Web_page.h` 헤더 `data:` 필드 갱신은 자동화 여지 있음.
