"""
OTA Cloud Connectivity Test
AWS IoT Jobs ping test via MQTT over TLS

Usage:
    python test_ota_ping.py

Requirements:
    pip install paho-mqtt

Certificate files needed (from AWS IoT Console):
    certs/AmazonRootCA1.pem
    certs/certificate.pem.crt
    certs/private.pem.key
"""

import ssl
import time
import json
import threading
import paho.mqtt.client as mqtt

# ─────────────────────────────────────────────
# TODO: Fill in your settings
# ─────────────────────────────────────────────
ENDPOINT   = "TODO_YOUR_ENDPOINT.iot.ap-northeast-2.amazonaws.com"
PORT       = 8883
THING_NAME = "w55rp20-benjamin-test"

CA_CERT    = "certs/AmazonRootCA1.pem"
CLIENT_CERT= "certs/certificate.pem.crt"
PRIVATE_KEY= "certs/private.pem.key"
# ─────────────────────────────────────────────

TOPIC_PUB  = f"$aws/things/{THING_NAME}/jobs/get"
TOPIC_ACC  = f"$aws/things/{THING_NAME}/jobs/get/accepted"
TOPIC_REJ  = f"$aws/things/{THING_NAME}/jobs/get/rejected"

received = threading.Event()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[OK] Connected to {ENDPOINT}")
        client.subscribe(TOPIC_ACC)
        client.subscribe(TOPIC_REJ)
        print(f"[>>] Ping → {TOPIC_PUB}")
        client.publish(TOPIC_PUB, "{}")
    else:
        print(f"[ERR] Connect failed, rc={rc}")
        received.set()

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode("utf-8", errors="replace")

    if "accepted" in topic:
        print(f"[OK] PING ACCEPTED - Cloud is reachable")
        try:
            data = json.loads(payload)
            jobs = data.get("queuedJobs", []) + data.get("inProgressJobs", [])
            if jobs:
                print(f"[INFO] Pending jobs: {len(jobs)}")
                for j in jobs:
                    print(f"       jobId={j.get('jobId')}  status={j.get('status')}")
            else:
                print("[INFO] No pending OTA jobs")
        except Exception:
            print(f"[INFO] Raw: {payload}")
    elif "rejected" in topic:
        print(f"[FAIL] PING REJECTED")
        print(f"       {payload}")

    received.set()

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"[WARN] Disconnected unexpectedly, rc={rc}")


def main():
    print("=" * 50)
    print(" AWS IoT OTA Ping Test")
    print("=" * 50)
    print(f" Endpoint  : {ENDPOINT}")
    print(f" Thing     : {THING_NAME}")
    print(f" CA        : {CA_CERT}")
    print(f" Cert      : {CLIENT_CERT}")
    print(f" Key       : {PRIVATE_KEY}")
    print("=" * 50)

    client = mqtt.Client(client_id=f"{THING_NAME}-ping-test")
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.on_disconnect = on_disconnect

    client.tls_set(
        ca_certs    = CA_CERT,
        certfile    = CLIENT_CERT,
        keyfile     = PRIVATE_KEY,
        tls_version = ssl.PROTOCOL_TLSv1_2
    )

    try:
        client.connect(ENDPOINT, PORT, keepalive=10)
    except Exception as e:
        print(f"[ERR] Connection failed: {e}")
        return

    client.loop_start()

    timeout = 10  # seconds
    if not received.wait(timeout=timeout):
        print(f"[TIMEOUT] No response within {timeout}s")
        print("  Check: endpoint URL, certificates, internet connection")

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
