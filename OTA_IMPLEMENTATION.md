# OTA 구현 문서 (W55RP20-S2E)

AWS IoT Jobs 기반 OTA(Over-The-Air) 펌웨어 업데이트 — 현재 구현 상태와 동작
FLOW를 정리한 문서. 실제 코드(`port/app/ota/`) 기준.

---

## 1. 설계 개요

- OTA는 **S2E 동작 모드와 완전히 독립**된 별도 백그라운드 서비스(`ota_service_task`)다.
  TCP/UDP/MQTT 등 어떤 working mode에서도 항상 동작한다.
- 자체 MQTT-over-TLS 연결, 자체 소켓, 자체 client ID를 가진다. S2E의 MQTT 연결을
  재사용하지 않는다.
- 의존성은 **단방향**: `App.c → ota → (platform_handler / config / mbedtls / AWS SDK)`.
  platform_handler·S2E 코드는 OTA를 전혀 참조하지 않는다.
- **2단계 커밋(two-phase commit)**: 다운로드 직후가 아니라, 새 펌웨어가 실제로
  부팅에 성공한 다음에만 Job을 `SUCCEEDED`로 보고한다. → 부팅 실패 시 거짓 성공이
  발생하지 않는다.

---

## 2. 모듈 구성

```
port/app/ota/
  inc/
    otaConfig.h     파라미터 1개 헤더 (엔드포인트 / 인증서 / 소켓 / 토픽)
    otaDownload.h   다운로드 엔진 API
    otaService.h    태스크 진입점 (ota_service_task)
  src/
    otaDownload.c   HTTPS 다운로드 · SHA256 검증 · Bank1 플래시 엔진 (MQTT 없음)
    otaService.c    MQTT AWS IoT Jobs 서비스 + 2단계 커밋
```

- CMake: 전용 라이브러리 `APP_OTA_FILES` (`App` / `App_linker` 양쪽이 링크).
- 의존 방향: `otaService.c → otaDownload.h` (단방향). 다운로드 엔진은 서비스를
  역참조하지 않는다.

---

## 3. 사용 리소스

| 항목 | 값 |
|---|---|
| AWS 엔드포인트 | `a3uz5t2azg1xdz-ats.iot.ap-northeast-2.amazonaws.com:8883` |
| Thing 이름 | `lihan_thing` |
| MQTT Client ID | `lihan_thing_ota` (S2E와 별개 — duplicate-client-id 충돌 방지) |
| **소켓 6** | MQTT-over-TLS 제어 채널 (AWS IoT) — 상시 유지 |
| **소켓 7** | HTTPS 펌웨어 다운로드 (S3) — 다운로드 시에만 사용 |
| MQTT keepalive | 1200초 |
| 인증서 | `otaConfig.h`에 내장 (Root CA / 디바이스 인증서 / 개인키) |

### 플래시 맵 (2MB)

| 영역 | 오프셋 | 크기 |
|---|---|---|
| Bootloader | `0x00000` | 128KB |
| Bank0 (실행 중인 앱) | `0x20000` | 512KB |
| Bank1 (OTA 스테이징) | `0xA0000` | 512KB |
| Config / RootCA / CliCA / PriKey / MAC | `0x120000`~`0x124000` | 4KB ×5 |
| **OTA 레코드** | `0x125000` | 4KB (1섹터) |

### MQTT 토픽

| 방향 | 토픽 |
|---|---|
| SUB | `$aws/things/lihan_thing/jobs/notify-next` |
| SUB | `$aws/things/lihan_thing/jobs/$next/get/accepted` |
| PUB | `$aws/things/lihan_thing/jobs/$next/get` |
| PUB | `$aws/things/lihan_thing/jobs/{jobId}/update` |
| PUB | `$aws/things/lihan_thing/shadow/update` |

---

## 4. 동작 FLOW (타임라인)

마커: 📤 W55RP20→AWS · 📥 AWS→W55RP20 · ⚙️ W55RP20 내부 작업

### Phase 0 — 부팅 & 연결

`ota_service_task()` → `ota_service_connect()`

```
⚙️ App.c가 ota_service_task 생성 (S2E 모드와 무관, 항상 생성)
⚙️ 토픽 문자열 4종 생성
⚙️ ota_last_load() — 플래시 0x125000 읽기, magic 'OTA2' 확인
       → 레코드 유효: state / job_id / version / result / timestamp 복원
       → 무효(0xFF, 미기록/지움): last_ota = N/A
⚙️ 8초 대기 (DHCP / 링크 안정화)

⚙️ DNS 조회 → 엔드포인트 IP
⚙️ TLS 핸드셰이크 (소켓 6, 포트 8883, 내장 인증서)
📤 MQTT CONNECT (client=lihan_thing_ota, keepalive=1200)
📥 CONNACK
📤 SUBSCRIBE  jobs/notify-next
📤 SUBSCRIBE  jobs/$next/get/accepted
📥 SUBACK ×2
📤 PUBLISH    jobs/$next/get   {}        ← 대기 중인 잡 폴링

   ota_confirm_pending() 분기:
     · PENDING 레코드 있음  → Phase 2 실행 (아래)
     · 없음 (정상 부팅)     → 📤 shadow/update {status:"idle"}

📤 PUBLISH shadow/update — 기기 상태 스냅샷 (부팅당 1회)
         {fw_version, chip, mac, ip, uptime_sec, last_ota{...}}
⚙️ 무한 루프 진입: MQTT_ProcessLoop() 200ms 주기 (잡 수신 대기)
```

### Phase 1 — 잡 수신 → 다운로드 → 적용  (BOOT N, 구 펌웨어)

`ota_handle_job()`

```
📥 PUBLISH 수신 — notify-next 푸시 또는 $next/get/accepted 응답
            payload = 잡 문서 (execution.jobId / status /
                       jobDocument.firmware.{url, size, sha256, version})
⚙️ ota_mqtt_event_cb: 문서를 s_job_doc로 복사, "jobDocument" 있으면 s_job_pending=1
⚙️ 루프가 s_job_pending 감지 → ota_handle_job()
⚙️ 잡 문서 파싱 (jobId, status, version, 루트 timestamp)
⚙️ 중복/복구 체크 — 신규 잡이면 통과 (자세한 분기는 §6)

──── 다운로드 구간: ota_download_apply() — 블로킹, MQTT 보고 0건 ────
⚙️ 잡 문서에서 S3 presigned URL · 크기 · SHA256 추출
⚙️ 펌웨어 크기 검증 (0 < size ≤ 512KB)
⚙️ 소켓 7로 HTTPS GET → S3
📥 (소켓7) 펌웨어 이미지 스트림 수신
⚙️ 받는 즉시 Bank1(0xA0000)에 섹터 단위 기록 + SHA256 동시 계산
⚙️ SHA256 대조 검증
⚙️ Bank1 첫 워드 sanity 체크 (유효한 벡터테이블 형태인가)
⚙️ 부트로더 arm — fwup_copy_flag=1, ConfigData 플래시 저장
──────────────────────────────────────────────────────────────

   ↓ 다운로드 + 검증 성공
📤 PUBLISH jobs/{jobId}/update   {status:"IN_PROGRESS"}
📤 PUBLISH shadow/update         {status:"installing"}   ← 서버가 듣는 마지막 상태
⚙️ ota_last_store(PENDING) — 플래시 0x125000에 {state=PENDING, job_id, version, ts}
⚙️ 1초 대기 (publish 플러시)
⚙️ device_reboot()
──────────────────────────────────────────────────────────────
⚙️ [부트로더] Bank1 → Bank0 복사   ← 실제 펌웨어 교체
⚙️ 새 펌웨어로 부팅
```

> **실패 시** (다운로드/검증 실패): `📤 jobs/{id}/update {status:"FAILED",
> statusDetails.reason:"SHA256_MISMATCH"...}` + `📤 shadow {status:"failed"}` +
> 플래시 DONE/failed 기록. **리붓하지 않음.**

### Phase 2 — 부팅 확인 → 성공 보고  (BOOT N+1, 새 펌웨어)

`ota_confirm_pending()`

```
⚙️ ota_service_task 재시작 → ota_last_load() → state == PENDING 감지
⚙️ 8초 대기 → DNS → TLS → MQTT CONNECT → SUBSCRIBE ×2 → 📤 $next/get
⚙️ ota_confirm_pending() 발동:
       "이 코드가 실행 중 = 새 FW가 정상 부팅함 = OTA 성공"
📤 PUBLISH jobs/{jobId}/update   {status:"SUCCEEDED", statusDetails.version}
📤 PUBLISH shadow/update         {status:"success"}
⚙️ ota_last_store(DONE, result="success") — 플래시 레코드 PENDING → DONE
📤 PUBLISH shadow/update — 기기 상태 스냅샷 (last_ota=success 반영)
⚙️ 무한 루프 진입 (다음 잡 대기)

📥 $next/get/accepted 응답: 잡이 이미 SUCCEEDED → 빈 응답 → 정상 종료
```

---

## 5. Job 상태 vs Shadow 상태

서로 **다른 AWS 서비스**이며 별개 채널로 보고된다.

| | Job 상태 | Shadow `status` |
|---|---|---|
| 정의 주체 | **AWS** (고정 enum) | **펌웨어** (임의 문자열) |
| 토픽 | `jobs/{id}/update` | `shadow/update` |
| 값 | `QUEUED`→`IN_PROGRESS`→`SUCCEEDED`/`FAILED`... | `idle` / `installing` / `success` / `failed` |
| AWS 해석 | ✅ 잡 큐·재시도에 사용 | ❌ 저장만 |
| 보고 함수 | `ota_report_job()` | `ota_report_shadow()` |

- `TIMED_OUT` / `CANCELED` / `REJECTED`는 **AWS가 직접** 설정 — 디바이스는 보고하지
  않는다. 디바이스가 보고하는 종료 상태는 `SUCCEEDED` / `FAILED` 둘뿐.
- Shadow `reported`는 누적 문서다. `fw_version`은 `ota_report_status()`만, `status`는
  `ota_report_shadow()`만 소유 — 필드 충돌 없음.

### Shadow 문서 예시

```json
{
  "fw_version": "1.2.1",
  "chip": "W55RP20",
  "mac": "EC:74:CD:00:01:C6",
  "ip": "192.168.11.2",
  "uptime_sec": 11,
  "status": "idle",
  "last_ota": { "job_id": "ota-...", "result": "success", "timestamp": 1779348277 }
}
```

---

## 6. 플래시 OTA 레코드 & 상태머신

`FLASH_OTA_INFO_ADDR`(0x125000)에 1섹터로 영속 저장. OTA 경로는 리붓을 동반하므로
결과를 플래시에 남겨야 다음 부팅에서 확인할 수 있다.

```c
typedef struct {
    uint32_t magic;        // 'OTA2' — 유효성 마커 (불일치 시 "레코드 없음")
    uint32_t state;        // OTA_REC_PENDING(1) / OTA_REC_DONE(2)
    uint32_t timestamp;    // AWS 잡 timestamp (epoch sec)
    char     job_id[65];
    char     version[33];
    char     result[12];   // "pending" / "success" / "failed"
} ota_last_record_t;
```

상태 전이:

```
(없음) ──다운로드 성공──▶ PENDING ──부팅 확인──▶ DONE/success
(없음) ──다운로드 실패──────────────────────▶ DONE/failed
```

`ota_handle_job()`의 잡 처리 분기:

1. **DONE 레코드 + job_id 일치** → 종료 보고가 유실됨 → 멱등 재전송 (`SUCCEEDED`/`FAILED`)
2. **PENDING 레코드 + job_id 일치** → 부팅 확인 (`ota_confirm_pending`)
3. **안전망**: 레코드 없음 + 잡 `IN_PROGRESS` + 버전이 현재 실행 버전과 일치 →
   재다운로드 없이 `SUCCEEDED` (무한 루프 방지)
4. **신규 잡** → 다운로드 → Phase 1

---

## 7. 견고성 / 실패 처리

- **부팅 실패(부트루프):** Phase 2가 실행되지 않음 → Job은 `IN_PROGRESS`로 남음 →
  거짓 `SUCCEEDED` 없음. (2단계 커밋의 핵심)
- **보고 유실:** AWS가 잡을 재전달하면 플래시 DONE 레코드를 근거로 멱등 재전송.
- **플래시 wipe:** OTA 레코드 섹터가 0xFF → magic 불일치 → "레코드 없음" → `last_ota`
  N/A. 별도 초기화 없이 정상 동작.
- **소켓 분리:** MQTT(6)와 HTTPS 다운로드(7)가 별개 소켓 → 다운로드 중에도 MQTT 유지.
- **RTC 부재:** `timestamp`는 AWS 잡 메시지 루트의 `timestamp` 필드(epoch초)를 파싱해
  사용 — 디바이스 자체 시계 없이 실제 시각 확보.

---

## 8. 변경 이력 (이번 리팩토링)

| # | 내용 |
|---|---|
| 1 | OTA를 S2E 피기백 → **독립 백그라운드 서비스**(`ota_service_task`)로 분리 |
| 2 | 다운로드 엔진(`otaDownload`)과 MQTT 서비스(`otaService`)를 단방향 의존으로 분리 |
| 3 | `last_ota` 결과를 **플래시 영속 저장** (전용 섹터 0x125000) |
| 4 | 부팅 시 **기기 상태 스냅샷** 보고 추가 (`ota_report_status`) |
| 5 | **2단계 커밋** 도입 — `SUCCEEDED`는 새 FW 부팅 확인 후에만 보고 |
| 6 | Shadow 필드 정리 — `status` 소문자 통일, `fw_version` 충돌 제거 |
| 7 | Shadow 상태 단어 `rebooting` → `installing` |
| 8 | OTA 코드를 독립 모듈 `port/app/ota/`로 이동, CMake `APP_OTA_FILES` 신설 |
