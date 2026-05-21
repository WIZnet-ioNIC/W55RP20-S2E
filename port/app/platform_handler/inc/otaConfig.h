#ifndef _OTA_CONFIG_H_
#define _OTA_CONFIG_H_

/*
 * ============================================================================
 * OTA Service Configuration
 * ----------------------------------------------------------------------------
 * Single header holding every parameter the standalone OTA background service
 * needs: AWS IoT endpoint, the dedicated MQTT client identity, the W5500
 * socket assignment, and the embedded X.509 credentials.
 *
 * The OTA service runs as its own task with its own MQTT-over-TLS connection,
 * independent of the S2E working mode. It does NOT reuse g_mqtt_config or
 * s2e_tlsContext.
 *
 * TODO: replace the hard-coded credentials with Fleet Provisioning later.
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * AWS IoT endpoint
 * ------------------------------------------------------------------------- */
#define OTA_CFG_ENDPOINT      "a3uz5t2azg1xdz-ats.iot.ap-northeast-2.amazonaws.com"
#define OTA_CFG_PORT          8883

/* ---------------------------------------------------------------------------
 * MQTT client identity
 *  - THING_NAME drives the $aws/things/<thing>/... topic paths.
 *  - CLIENT_ID MUST be unique per AWS IoT connection. It is intentionally
 *    different from the S2E client_id so the OTA connection and a possible
 *    S2E MQTT connection do not kick each other off (duplicate-client-id).
 * ------------------------------------------------------------------------- */
#define OTA_CFG_THING_NAME    "lihan_thing"
#define OTA_CFG_CLIENT_ID     "lihan_thing_ota"

/* MQTT keep-alive in seconds (AWS IoT allows 30 ~ 1200). */
#define OTA_CFG_KEEPALIVE_SEC 1200

/* Chip / hardware identifier reported in the device status shadow. */
#define OTA_CFG_CHIP_NAME     "W55RP20"

/* ---------------------------------------------------------------------------
 * W5500 socket dedicated to the OTA MQTT connection.
 *  - Socket 7 stays reserved for the HTTPS firmware download (SOCK_OTA_HTTP).
 *  - Socket 6 is HTTP-server socket #3; free while the web server is not the
 *    active path. The OTA MQTT link and the HTTPS download never need the
 *    same socket, but they DO overlap in time (status is reported over MQTT
 *    while the download runs), so they must be two distinct sockets.
 * ------------------------------------------------------------------------- */
#define OTA_CFG_MQTT_SOCK     6

/* ---------------------------------------------------------------------------
 * MQTT topics (printf format, %s = thing name)
 * ------------------------------------------------------------------------- */
#define OTA_CFG_TOPIC_NOTIFY_NEXT   "$aws/things/%s/jobs/notify-next"
#define OTA_CFG_TOPIC_NEXT_GET      "$aws/things/%s/jobs/$next/get"
#define OTA_CFG_TOPIC_NEXT_GET_ACC  "$aws/things/%s/jobs/$next/get/accepted"
#define OTA_CFG_TOPIC_JOB_UPDATE    "$aws/things/%s/jobs/%s/update"
#define OTA_CFG_TOPIC_SHADOW_UPDATE "$aws/things/%s/shadow/update"

/* ---------------------------------------------------------------------------
 * Embedded X.509 credentials (from /policy)
 *  - OTA_CFG_ROOT_CA  : Amazon Root CA 1  -> verifies the AWS IoT broker
 *  - OTA_CFG_CLIENT_CRT / OTA_CFG_CLIENT_KEY : this device's certificate and
 *    private key for mutual-TLS authentication.
 *
 * Each macro is a single NUL-terminated string literal. mbedtls_x509_crt_parse
 * / mbedtls_pk_parse_key expect the length to include the terminating NUL,
 * i.e. pass (sizeof(string)) — see otaService.c.
 *
 * Defined here (not in a .c) on purpose: this header is included by exactly
 * one translation unit (otaService.c), so there is no duplication.
 * ------------------------------------------------------------------------- */

/* Amazon Root CA 1 */
#define OTA_CFG_ROOT_CA \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----\n"

/* Device certificate */
#define OTA_CFG_CLIENT_CRT \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDWTCCAkGgAwIBAgIUSuz+4vN7WnZ7WxP/kVyVKndjAQEwDQYJKoZIhvcNAQEL\n" \
"BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g\n" \
"SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDQwMjIzMzUy\n" \
"MFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0\n" \
"ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAJW5t9xlXL0yJiF8nK2a\n" \
"qsZopzg57YsvyjH9BvZkifNMC36Si+8xLQhfThLnDHHPoQ3hdXUShWDZF8werEKk\n" \
"QbbnQabRXRpS8OG4IBNR8CdsWdYGVdDcliGfCru/MFm3k+F2Hq7j4oI6AKvZPojp\n" \
"jih9c4xwioYM0Pja4VJIl+IsyPF0TH0KkivF2Q34k99CZ/okt48gO7LSrZ3M6T9F\n" \
"AZuTK3OCCZLMPt4E76yj0W39aQbENtOIhV8eJcJCGOR2UNVquXz7YLbjt7O2qfKp\n" \
"fG1hdeBf6t0KpGq0CB3lbZxHJBZqCUIt5aKhTGmqg5fUS79fBaeYnDjJ9jXqKKTK\n" \
"A88CAwEAAaNgMF4wHwYDVR0jBBgwFoAU2vvlLsZZsPIZqOo4Cu8k3xluFSUwHQYD\n" \
"VR0OBBYEFERPuMp+wV8MIcEYh7k1leXk2185MAwGA1UdEwEB/wQCMAAwDgYDVR0P\n" \
"AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCDJS1TuCm9mt0jhGmaFuCIS2xO\n" \
"Vu5ThQy3ayfGPppHTL6QQbJGL5yQags2AYy277QiTD5sPgG6jOpNl/MI0jCgriyQ\n" \
"pIDI93ohkorhewVoeNMQAU6btrEsBLrYrFeA8Basi7RnUu9LND1Zem8HDSGumErV\n" \
"avMp8AHhZEBTN314XwbH/d0rZY+yT8LnCZuhVnVC5ZV/+J/oNKZLUD+pR7jA4Gsb\n" \
"zT5msiedri83o/NvwvhN5CN6r+WbKI2gUelJprcb4MGMOj5W0B/w1kaSCrywznVz\n" \
"mMRA/7xlu3N3c22+AlvnD5eveT2uMCvft6F0WYT4stZh2lXvamAAJhIGuzln\n" \
"-----END CERTIFICATE-----\n"

/* Device private key */
#define OTA_CFG_CLIENT_KEY \
"-----BEGIN RSA PRIVATE KEY-----\n" \
"MIIEowIBAAKCAQEAlbm33GVcvTImIXycrZqqxminODntiy/KMf0G9mSJ80wLfpKL\n" \
"7zEtCF9OEucMcc+hDeF1dRKFYNkXzB6sQqRBtudBptFdGlLw4bggE1HwJ2xZ1gZV\n" \
"0NyWIZ8Ku78wWbeT4XYeruPigjoAq9k+iOmOKH1zjHCKhgzQ+NrhUkiX4izI8XRM\n" \
"fQqSK8XZDfiT30Jn+iS3jyA7stKtnczpP0UBm5Mrc4IJksw+3gTvrKPRbf1pBsQ2\n" \
"04iFXx4lwkIY5HZQ1Wq5fPtgtuO3s7ap8ql8bWF14F/q3QqkarQIHeVtnEckFmoJ\n" \
"Qi3loqFMaaqDl9RLv18Fp5icOMn2NeoopMoDzwIDAQABAoIBAF/use6GB6i1pBba\n" \
"p+zX58nerh3ph9khkrT6wWZHvtfjho3fycdFQ+xUXEriPWgCt9eT+NU9O7Z1Arln\n" \
"gcnlQrG1dKw1AyllsS9+8RZP7sbjtp/aSUvculjqdU8cThd990ODkl9OZgg4r8ts\n" \
"E5RXyR0KXGnEhg0j0xVP8QTuvFpOir9KwmKZHrbrixGZ4i4UrhpLj8l23aOb4wNP\n" \
"ArgrKmld+uZ1+XKMddYB5hgVRSSjdvnrqUzS3ZldUjl/jojzzNN2qb5v1pT4UMQG\n" \
"fQvGKfkmIRI5axaG2WJnLI7Oe1IUomgohYpEf3PV7MRVUNBxvf/P8BZEBG848DLp\n" \
"3CraesECgYEAxfDsoGVXpqk9YS5+ALBCVKhm4dvvYw+42XAqD2SwzPMrFQF1j8Ds\n" \
"gf7eksDjDEqddfBc6a30FH/qyhnb62V2OT9YdZIVppNZEQkQBEYQlp568SdDtRCQ\n" \
"mXfzRmQK8Sml99JYtSPd73f1H62WkSJX29fbfNeSgY+pFUIocfPLky8CgYEAwaRb\n" \
"/IKJqzi6C6DAcuJiyJNo0vTUDNOk/BjlQq7M40i07Uj4kjopeq9LKT2Yzku2QS0B\n" \
"cF6i6uk9+xC0zG4OV1Lwdvzar6dpFce3vc8PS0ukJ/4T0cRYJYJldhL3ZrjsSqPg\n" \
"FSFUdNnRYNccB7zB5HFIrdLPMDRSjQDv42xo8WECgYEAjjUlC6dDba0xN9ybEguP\n" \
"3BqQr7u/JMBq4WzACwcgLl6VmSXPPFQylsJxTaXeeToyHkVtu0UKkp/EIyao5Vmj\n" \
"skVcXDtz6pT57E9Cfo+H425AjXjUIAWinV+cG/pMEi8F2iH5MUpvbFgWd4fTILo3\n" \
"vO479lk6HHMF2wwI2rV4kGkCgYALPt5QtcwE85BqaHWEtn0CJP1rcuh1rjzgnxmB\n" \
"W+FuOaS4Owqkg1gxVcjJplgfcuosss5oljZj6hO9ZuT5ElMm4xwv7NNObCyAJU8Y\n" \
"aNP8jXIFGI5SGlL8Kqx1xPg1MPwYAZhb4cmj15p/Qz7PfDjOaX3NCncZ15ALvEgy\n" \
"LvORQQKBgFA1fmsx9vwejVsNsqr9g2MuAbqKFB5bI+cJ3zJcnw+I0pmw/QSw36H2\n" \
"X6PVcym5ixSdJ7JjjQkyhulq9KdIWzE9wNgPdjhnRj+2o7Lq9V3vJbeDM95/jJjz\n" \
"Duts321kYqzNGcTeOAlESmOlFfFezggOXDXOR9g1v7nxjlY3ztGb\n" \
"-----END RSA PRIVATE KEY-----\n"

#endif /* _OTA_CONFIG_H_ */
