
#ifndef __HTTPHANDLER_H
#define __HTTPHANDLER_H

#include <stdint.h>
#include "httpParser.h"

void make_json_devinfo(uint8_t * buf, uint16_t * len);
uint8_t set_devinfo(uint8_t * uri);
uint8_t set_devreset(uint8_t * uri);
uint8_t set_devfacreset(uint8_t * uri);
uint8_t update_module_firmware(st_http_request * p_http_request, uint8_t *buf);

void http_webserver_task(void *argument);


#endif //__HTTPHANDLER_H


