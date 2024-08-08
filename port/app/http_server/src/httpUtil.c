/**
 * @file	httpUtil.c
 * @brief	HTTP Server Utilities	
 * @version 1.0
 * @date	2014/07/15
 * @par Revision
 *			2014/07/15 - 1.0 Release
 * @author	
 * \n\n @par Copyright (C) 1998 - 2014 WIZnet. All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "httpUtil.h"
#include "WIZ5XXSR-RP_Debug.h"

uint8_t http_get_cgi_handler(uint8_t * uri_name, uint8_t * buf, uint32_t * file_len)
{
	uint8_t ret = HTTP_OK;
	uint16_t len = 0;
	
	if(predefined_get_cgi_processor(uri_name, buf, &len))
	{
		;
	}
	else
	{
		// CGI file not found
		ret = HTTP_FAILED;
	}

	if(ret)	*file_len = len;
	return ret;
}

uint8_t http_post_cgi_handler(uint8_t * uri_name, st_http_request * p_http_request, uint8_t * buf, uint32_t * file_len)
{
	uint8_t ret = HTTP_OK;
	uint16_t len = 0;
	uint8_t * device_ip;
	uint8_t val;

	if(predefined_set_cgi_processor(uri_name, p_http_request, buf, &len))
	{
    ret = HTTP_RESET;
	}
	else
	{
		// CGI file not found
		ret = HTTP_FAILED;
	}

	if(ret)	*file_len = len;
	return ret;
}


uint8_t predefined_get_cgi_processor(uint8_t * uri_name, uint8_t * buf, uint16_t * len)
{
	uint8_t ret = 1;	// ret = 1 means 'uri_name' matched
	uint8_t cgibuf[14] = {0, };
	int8_t cgi_dio = -1;
	int8_t cgi_ain = -1;

	uint8_t i;
  
	if(strcmp((const char *)uri_name, "get_devinfo.cgi") == 0)
	{
    make_json_devinfo(buf, len);
	}
	else
	{
		;
	}

	return ret;

}


uint8_t predefined_set_cgi_processor(uint8_t * uri_name, st_http_request * p_http_request, uint8_t * buf, uint16_t * len)
{
	uint8_t ret = 1;	// ret = 1 means 'uri_name' matched
	uint8_t val = 0;

	// Devinfo; devname
	PRT_INFO("uri_name = %s\r\n", uri_name);
	if(strcmp((const char *)uri_name, "set_devinfo.cgi") == 0)
	{
		val = set_devinfo(p_http_request->URI);
    *len = sprintf((char *)buf, "%d", val);
	}
  else if(strcmp((const char *)uri_name, "set_devreset.cgi") == 0)
	{
		val = set_devreset(p_http_request->URI);
	  *len = sprintf((char *)buf, "%d", val);
	  //*len = sprintf((char *)buf,"<html><head><title>WIZ750SR - Configuration</title><body>Reset. Please wait a few seconds. <span style='color:red;'>DHCP server</span></body></html>\r\n\r\n");
	}
  else if(strcmp((const char *)uri_name, "set_devfacreset.cgi") == 0)
	{
		val = set_devfacreset(p_http_request->URI);
		*len = sprintf((char *)buf, "%d", val);
    //*len = sprintf((char *)buf,"<html><head><title>WIZ750SR - Configuration</title><body>Factory Reset Complete. Please wait a few seconds. <span style='color:red;'>DHCP server</span></body></html>\r\n\r\n");
	}
  else if(strcmp((const char *)uri_name, "update_module_firmware.cgi") == 0)
	{
		if (update_module_firmware(p_http_request, buf))
      *len = sprintf((char *)buf, "<html><head><title>W55RP20-S2E</title><body>F/W Update Complete. Device Reboot Please wait a few seconds.</body></html>\r\n\r\n");
	}
	else
	{
		ret = 0;
	}

  //*len = (*len)+ 1;
	return ret;
}
