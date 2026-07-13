
#ifndef __HTTPHANDLER_H
#define __HTTPHANDLER_H

#include <stdint.h>
#include "httpParser.h"

void make_json_devinfo(uint8_t * buf, uint16_t * len);
uint8_t set_devinfo(uint8_t * uri);
uint8_t set_devreset(uint8_t * uri);
uint8_t set_devfacreset(uint8_t * uri);
uint8_t update_module_firmware(st_http_request * p_http_request, uint8_t *buf);

#ifdef __USE_S2E_OVER_TLS__
uint8_t update_ssl_rootca(st_http_request * p_http_request, uint8_t *buf);
uint8_t update_ssl_clica(st_http_request * p_http_request, uint8_t *buf);
uint8_t update_ssl_prikey(st_http_request * p_http_request, uint8_t *buf);
#endif

void http_webserver_task(void *argument);


#endif //__HTTPHANDLER_H


