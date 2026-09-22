#include <string.h>

#include "port_common.h"
#include "common.h"
#include "ConfigData.h"
#include "PPPoE.h"
#include "pppoeHandler.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "seg.h"        // seg_wizchip_api_lock(): the data path shares this chip

// PPPoE.c reaches for these by name. The example defined them beside its own
// hard-coded strings; here they are filled from the stored configuration before
// every attempt, and pppoe_ip is where the library leaves the assigned address.
uint8_t pppoe_id[PPPOE_ID_SIZE];
uint8_t pppoe_id_len;
uint8_t pppoe_pw[PPPOE_PW_SIZE];
uint8_t pppoe_pw_len;
uint8_t pppoe_ip[4];
uint16_t pppoe_retry_count;

// ppp_start() reads whole Ethernet frames into this, so it has to hold one.
// PPP_RXFRAME_SIZE is 1514.
static uint8_t pppoe_frame_buf[PPP_RXFRAME_SIZE] __attribute__((aligned(4)));

static uint8_t pppoe_connected;

uint8_t is_pppoe_connected(void) {
    return pppoe_connected;
}

void pppoe_get_assigned_ip(uint8_t *ip) {
    memcpy(ip, pppoe_ip, 4);
}

// The chip raises IR_PPPoE when the far end drops the session. Nothing else
// notices: the data sockets simply stop passing traffic.
uint8_t pppoe_link_lost(void) {
    if (!pppoe_connected) {
        return 0;
    }
    if (getIR() & IR_PPPoE) {
        setIR(IR_PPPoE);
        pppoe_connected = 0;
        return 1;
    }
    return 0;
}

void pppoe_disconnect(void) {
    if (pppoe_connected) {
        do_lcp_terminate();
        pppoe_connected = 0;
    }
    setMR(getMR() & ~MR_PPPOE);
    seg_wizchip_api_lock();
    close(SOCK_PPPOE);
    seg_wizchip_api_unlock();
}

int8_t process_pppoe(void) {
    struct __network_option *network_option =
        (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    uint8_t ret;

    pppoe_connected = 0;

    pppoe_id_len = (uint8_t)strnlen(network_option->pppoe_id, sizeof(network_option->pppoe_id));
    pppoe_pw_len = (uint8_t)strnlen(network_option->pppoe_pw, sizeof(network_option->pppoe_pw));
    if ((pppoe_id_len == 0) || (pppoe_pw_len == 0)) {
        PRT_ERR(" - PPPoE: no account set\r\n");
        return PPPOE_RET_NO_ACCOUNT;
    }
    memcpy(pppoe_id, network_option->pppoe_id, pppoe_id_len);
    memcpy(pppoe_pw, network_option->pppoe_pw, pppoe_pw_len);
    memset(pppoe_ip, 0x00, sizeof(pppoe_ip));

    PRT_INFO(" - PPPoE connecting as [%s]\r\n", network_option->pppoe_id);

    // ppp_start() runs one step of the negotiation per call and reports
    // PPP_RETRY until it reaches an answer.
    pppoe_retry_count = 0;
    while (1) {
        seg_wizchip_api_lock();
        ret = ppp_start(pppoe_frame_buf);
        seg_wizchip_api_unlock();

        if (ret == PPP_SUCCESS) {
            pppoe_connected = 1;
            PRT_INFO(" - PPPoE up, address %d.%d.%d.%d\r\n",
                     pppoe_ip[0], pppoe_ip[1], pppoe_ip[2], pppoe_ip[3]);
            return PPPOE_RET_SUCCESS;
        }
        if ((ret == PPP_FAIL) || (pppoe_retry_count > PPP_MAX_RETRY_COUNT)) {
            PRT_ERR(" - PPPoE failed\r\n");
            pppoe_disconnect();
            return PPPOE_RET_FAILED;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
