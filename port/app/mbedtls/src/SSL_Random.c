/*
    file: SSL_Random.h
    description: random generator function
    author: peter
    company: wiznet
    data: 2015.11.26
*/

#include "SSL_Random.h"


int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen) {
    (void)data;
    for (size_t i = 0; i < len; i += 4) {
        uint32_t r = get_rand_32(); // pico_rand (ROSC-based)
        size_t copy = (len - i) < 4 ? (len - i) : 4;
        memcpy(output + i, &r, copy);
    }
    *olen = len;
    return 0;
}
