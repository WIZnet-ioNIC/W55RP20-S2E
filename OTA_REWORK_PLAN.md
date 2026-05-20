# OTA Rework 계획서

목표: **Config tool 의 FW 업데이트 동작과 동등한 방식으로**, MQTT 로 받은 URL 에서 펌웨어를 다운로드해 Bank1 flash 에 기록하고 Boot 가 Bank0 로 복사하도록 보장한다.

---

## 0. 배경 — 두 흐름 비교

### Config tool 경로 (검증된 기준)

`port/app/platform_handler/src/httpHandler.c :: update_module_firmware()`

```
HTTP multipart body
  ├─ 본문을 FLASH_SECTOR_SIZE 버퍼에 누적
  ├─ 가득 차면 write_flash(f_addr, temp_buf, FLASH_SECTOR_SIZE);  f_addr += SECTOR
  ├─ 마지막 남은 buf_len 도 write_flash(f_addr, temp_buf, FLASH_SECTOR_SIZE);  (sector 단위)
  ├─ fwupdate->fwup_copy_flag = 1
  ├─ fwupdate->fwup_size      = (f_addr - FLASH_START_ADDR_BANK1_OFFSET) + buf_len
  └─ save_DevConfig_to_storage()
```

`main/Boot/Boot.c :: main()`

```
if (fwup_copy_flag == 1) {
    device_bank_check(1);            // Bank1 첫 워드 != 0x00 / 0xFF
    erase_storage(STORAGE_APPBANK);  // Bank0 erase
    device_bank_copy();              // Bank1 → Bank0, sector 단위, size+1sector
    fwup_copy_flag = 0;
    write_storage(STORAGE_CONFIG, ...)
}
jump_to_app(FLASH_START_ADDR_BANK0);
```

### 현재 OTA 경로

`port/app/platform_handler/src/otaHandler.c :: ota_task() → ota_download_and_flash()`

```
HTTPS GET (S3 presigned) body
  ├─ sector_buf 누적 → 가득 차면 write_flash(...)   ← Config tool 과 동일
  ├─ 마지막 partial sector 도 write_flash(...)       ← Config tool 과 동일
  ├─ 다운로드 도중 mbedtls_sha256_update            ← OTA 만의 강화
  ├─ ota_verify_sha256() (memcmp)                   ← OTA 만의 강화
  ├─ Bank1 첫 워드 사전 검사 (0x00/0xFF 아닌지)      ← OTA 만의 강화
  ├─ fwupdate->fwup_size      = s_fw_size  (Job 문서 값)
  ├─ fwupdate->fwup_copy_flag = 1
  ├─ save_DevConfig_to_storage()
  └─ device_reboot()
```

골격은 동일하지만 몇 가지 정합성 / 일관성 / 견고성 이슈가 있다.

---

## 1. 문제 진단 (코드 리뷰 결과)

### 1-1. `fwup_size` 의미가 다르다

- Config tool: **실제로 기록된 바이트 수** = `(f_addr - bank_base) + buf_len`
- 현재 OTA: **Job 문서가 선언한 size**

`device_bank_copy()` 는 `fwup_size + FLASH_SECTOR_SIZE` 만큼 sector 단위 복사를 수행한다. Job 문서의 size 가 실제 다운로드한 양과 어긋나면:
- 작으면 → 일부 Bank0 가 복사되지 않음 (실행 시 hard fault 가능)
- 크면 → Bank1 의 다른 영역을 함께 Bank0 로 복사 (안전하지만 시간 낭비)

OTA 는 partial download 시 FAILED 반환하니 실제 사고 가능성은 낮지만, **검증된 기준값(Config tool 방식)으로 통일하는 게 안전**.

### 1-2. Bank1 사전 erase 부재

Config tool 흐름에서도 Bank1 을 별도로 erase 하지 않는다. `write_flash()` 내부에서 `erase_flash_sector()` 를 호출하기 때문 ([flashHandler.c:44-53](port/boot/platform_handler/src/flashHandler.c#L44-L53)). 현재 OTA 도 동일 함수를 사용하므로 OK.

### 1-3. 다운로드 실패 시 잔여 데이터 처리

다운로드가 중간에 실패하면 OTA 는 `OTA_RET_FAILED` 만 반환하고 Bank1 상태는 그대로 둔다. `fwup_copy_flag` 를 건드리지 않으므로 Boot 가 적용을 시도하지는 않는다 — 안전.

다만 다음 OTA 시도 시 이미 Bank1 의 일부 sector 가 dirty 한 채로 시작될 수 있는데, sector 단위로 다시 erase+write 되므로 functionally 무해.

### 1-4. `fwup_flag` 미사용

`fwup_flag` 는 Config tool 의 다른 경로(SEGCP `SEGCP_FW`)에서만 set 됨. Boot 가 보는 것은 `fwup_copy_flag` 뿐. OTA 도 `fwup_copy_flag` 만 set 하면 충분. 변경 불필요.

### 1-5. SHA256 검증과 Bank1 sanity 의 위치

현재 순서:
```
download (+streaming SHA) → verify SHA256 → Bank1 word0 check → set flag → reboot
```

문제 없음. SHA OK 인데 Bank1 word0 가 0x00/0xFF 라면 그건 flash 자체 이슈이므로 abort 가 맞다.

### 1-6. 부분 다운로드 시 buf_fill 처리

```c
/* Flush last partial sector */
if (download_ok && buf_fill > 0) {
    write_flash(f_addr, sector_buf, OTA_FLASH_SECTOR_SIZE);
}
```

`sector_buf` 는 `0xFF` 로 초기화되어 있고, partial 데이터만 앞에 채워져 있다. sector 단위로 통째로 기록 → 동일한 Config tool 동작과 일치. OK.

### 1-7. URL 길이 / 보안 / 안정성

- `OTA_URL_MAX_LEN = 512` — S3 presigned URL 도 일반적으로 들어감.
- TLS 핸드셰이크 / DNS 타임아웃 / 비정상 응답 처리 다 있음.
- 소켓 7 (SOCK_OTA_HTTP) 을 HTTP 서버에서 빼고 OTA 전용 예약 — 이미 적용됨.

### 1-8. 누락된 점

- **fwup_size 산정 방식 미일치** (1-1 항목)
- **다운로드 시작 전 Bank1 명시적 erase 가 없음 (write_flash 가 처리하긴 함)** — 위험은 없으나 명시성 부족
- **HTTP 비-TLS URL 지원 안 됨** — 사용자 요구에 따라 추가 검토
- **다운로드 진행률 로그가 sector 단위뿐** — 큰 펌웨어일 때 침묵 구간이 김

---

## 2. 변경 범위 (제안)

본 작업은 **Config tool 동작과의 일관성 확보**와 **신뢰성 보강**이 목표이지 새 기능 추가가 아니다.

### A. 필수 수정 (Config tool 동작과 동등화)

#### A-1. `fwup_size` 산정을 실제 기록 길이로 변경

`ota_download_and_flash()` 가 끝낼 때 `total_recv` 를 반환하고, `ota_task()` 에서 그 값을 그대로 `fwup_size` 로 사용한다. SHA256 검증은 Job 문서의 size 와 무관하게 진행된다 (현재도 그렇게 동작).

```c
/* before */
fwupdate->fwup_size = s_fw_size;

/* after */
fwupdate->fwup_size = s_actual_written;  /* total bytes successfully flashed */
```

`s_actual_written` 은 새 정적 변수, `ota_download_and_flash()` 종료 직전에 `total_recv` 값을 저장.

#### A-2. fwup_size 와 Job size 불일치 시 명시 로그

```c
if (s_actual_written != s_fw_size) {
    printf(" > OTA:SIZE:MISMATCH actual=%lu job=%lu\r\n",
           s_actual_written, s_fw_size);
    /* 계속 진행 — 어차피 SHA256 으로 무결성 보장 */
}
```

### B. 권장 보강 (안정성)

#### B-1. 진행률 로그 (선택)

10% 단위 또는 64KB 단위 진행률 로그 추가. 다운로드 무응답인지 진행 중인지 외부에서 판단 가능.

```c
if ((total_recv / progress_step) > last_logged) {
    printf(" > OTA:PROGRESS %lu/%lu (%lu%%)\r\n", ...);
    last_logged = total_recv / progress_step;
}
```

#### B-2. 다운로드 시작 전 명시적 Bank1 erase (선택)

`write_flash()` 가 내부적으로 sector erase 를 하기 때문에 functionally 불필요. 다만 명시적으로 `erase_flash_bank(1)` 을 한 번 호출하면 분석/디버깅 시 의도가 명확해진다.

> 단점: 큰 bank erase 는 수백 ms 걸리고, 실패 시 전체 다운로드 무효화. 현재 sector-by-sector 방식이 더 견고하므로 **추가하지 않는 것을 권장**.

#### B-3. 다운로드 타임아웃 (선택)

`wiz_tls_read()` 가 0 을 계속 반환하는 경우(=서버 무응답) 무한 루프에 빠질 수 있다. 누적 시간 또는 연속 0 카운트로 끊는 로직 추가.

```c
if (n == 0 && ++zero_count > MAX_ZERO_READS) {
    printf(" > OTA:HTTP:Timeout\r\n");
    download_ok = 0;
    break;
}
```

### C. 의도적 미반영 (스코프 밖)

- **HTTP (non-TLS) 지원** — 현재 S3 presigned 만 가정. HTTP 가 필요하면 별도 issue.
- **MQTT 메시지 포맷 단순화** (URL-only) — 현재 AWS Jobs document JSON 유지. 변경 시 AWS 콘솔/Job 정책과 영향 큼.
- **Resume / chunked transfer-encoding** — S3 presigned 가 content-length 명시하므로 불필요.

---

## 3. 파일별 변경 명세

### 3-1. `port/app/platform_handler/src/otaHandler.c`

#### (1) 정적 변수 추가
```c
static uint32_t s_actual_written = 0;  /* bytes successfully flashed in last download */
```

#### (2) `ota_download_and_flash()` 끝부분
```c
/* 기존: return OTA_RET_SUCCESS */

s_actual_written = total_recv;          /* 실제 기록 길이 저장 */
printf(" > OTA:DOWNLOAD:Complete %lu bytes\r\n", (unsigned long)total_recv);
return OTA_RET_SUCCESS;
```

#### (3) `ota_task()` 의 flag set 부분
```c
struct __firmware_update *fwupdate = ...;
fwupdate->fwup_size      = s_actual_written;   /* ← was: s_fw_size */
fwupdate->fwup_copy_flag = 1;

if (s_actual_written != s_fw_size) {
    printf(" > OTA:SIZE:MISMATCH actual=%lu job=%lu\r\n",
           (unsigned long)s_actual_written, (unsigned long)s_fw_size);
}
save_DevConfig_to_storage();
```

#### (4) (선택) 다운로드 진행률 로그
- 매 64KB 단위로 1회 출력
- `wiz_tls_read()` 직후, `total_recv += n` 이후

#### (5) (선택) zero-read 타임아웃

### 3-2. 기타 파일

- `otaHandler.h` — 공개 API 변경 없음
- Boot 측 코드 — 변경 없음 (`fwup_size` 의미 통일로 자연스럽게 호환)

---

## 4. 검증 시나리오

### 4-1. 정상 OTA
- AWS IoT Jobs 로 정상 펌웨어 publish
- 기대 로그:
  ```
  > OTA:JOB:Signaled OTA task
  > OTA:TASK:Starting download
  > OTA:HTTPS:Host=...
  > OTA:DNS:Resolved ...
  > OTA:TLS:Connected
  > OTA:HTTP:200 OK - Starting flash write
  > OTA:FLASH:0x... (반복)
  > OTA:DOWNLOAD:Complete N bytes
  > OTA:SHA256:OK
  > OTA:BANK1[0]=0x... (≠ 0x00/0xFF)
  > OTA:REPORT:{"status":"SUCCEEDED","statusDetails":{"version":"2.2.2"}}
  > OTA:SHADOW:{...}
  > (reboot)
  > [부팅 후 Boot.c]
  > fw_data = 0x...
  > write_fw_len = N+sector, fwupdate->fwup_size = N
  > jump addr = 0x10000100
  > [새 펌웨어 시작 로그]
  ```

### 4-2. SHA256 mismatch
- 펌웨어 바이너리는 정상이지만 Job 문서의 SHA256 만 1바이트 변조
- 기대: `OTA:SHA256:MISMATCH` 출력, FAILED 보고, **`fwup_copy_flag` 안 건드림** → reboot 안 함

### 4-3. 다운로드 중단
- TLS read 중간 종료
- 기대: `OTA:RECV:EOF` → FAILED/DOWNLOAD_ERROR → Bank1 일부만 기록되었지만 `fwup_copy_flag` 안 변경됨 → 다음 부팅에 영향 없음

### 4-4. Bank1 invalid
- (인위적 재현 어려움) — 보강 로그만 확인

### 4-5. Config tool FW 업데이트와 충돌 없음 확인
- 본 변경 후 웹 UI 로 펌웨어 업데이트해도 동일하게 동작
- `update_module_firmware()` 는 건드리지 않음

---

## 5. 작업 순서 (체크리스트)

- [ ] **Step 1** — `s_actual_written` 정적 변수 추가
- [ ] **Step 2** — `ota_download_and_flash()` 가 `total_recv` 를 `s_actual_written` 에 저장하도록 수정
- [ ] **Step 3** — `ota_task()` 의 `fwup_size = s_fw_size` 를 `fwup_size = s_actual_written` 으로 변경
- [ ] **Step 4** — size mismatch 로그 추가
- [ ] **Step 5** — (선택) 64KB 단위 진행률 로그
- [ ] **Step 6** — (선택) zero-read 타임아웃
- [ ] **Step 7** — 빌드 (`build/` 디렉터리에서 cmake/ninja or 기존 빌드 스크립트)
- [ ] **Step 8** — 펌웨어 플래시 후 시리얼 로그로 정상 OTA 시나리오(4-1) 확인
- [ ] **Step 9** — SHA mismatch 시나리오(4-2) 확인
- [ ] **Step 10** — 문서 (`OTA_IMPLEMENTATION.md`) 의 fwup_size 항목 업데이트

---

## 6. 리스크 / 롤백

- 변경 범위는 **`otaHandler.c` 의 다섯 줄 내외** + (선택) 진행률 로그.
- 잘못되어도 `fwup_size` 값만 영향. SHA256 검증은 그대로 → 무결한 펌웨어만 적용됨.
- 롤백: 해당 라인만 되돌리면 끝. git revert 단위로 단순.

---

## 7. 결정이 필요한 항목 (사용자 확인)

| 번호 | 항목 | 옵션 |
| --- | --- | --- |
| Q1 | A-1 (fwup_size 산정 변경) | **반영** / 미반영 |
| Q2 | A-2 (size mismatch 로그) | **반영** / 미반영 |
| Q3 | B-1 (진행률 로그) | 반영 / **미반영** |
| Q4 | B-3 (zero-read 타임아웃) | 반영 / **미반영** |
| Q5 | C (HTTP/MQTT 단순화 등 스코프 확장) | **미반영** / 반영 |

기본값은 **A 만 반영**.
