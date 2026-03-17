#ifndef _MB_H_
#define _MB_H_

#define MB_RETRY_MAX 3

void mbTCPtoRTU(uint8_t sock);
void mbRTURetransmit(void);
int mbRTUtoTCP(uint8_t sock);
int mbASCIItoTCP(uint8_t sock);
void mbTCPtoASCII(uint8_t sock);
#endif

