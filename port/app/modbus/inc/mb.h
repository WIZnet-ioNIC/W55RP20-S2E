#ifndef _MB_H_
#define _MB_H_

#define MB_RETRY_MAX 3

void mbTCPtoRTU(uint8_t sock, int channel);
void mbRTURetransmit(int channel);
int mbRTUtoTCP(uint8_t sock, int channel);
int mbASCIItoTCP(uint8_t sock, int channel);
void mbTCPtoASCII(uint8_t sock, int channel);
#endif

