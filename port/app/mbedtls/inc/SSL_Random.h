/*
    file: SSL_Random.c
    description: random generator function
    author: peter
    company: wiznet
    data: 2015.11.26
*/


#ifndef __SSL_RANDOM_H_
#define __SSL_RANDOM_H_

#include <stdio.h>
#include <string.h>
#include "port_common.h"

int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen);

#endif
