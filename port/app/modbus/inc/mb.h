#ifndef _MB_H_
#define _MB_H_

int mbTCPtoRTU(uint8_t sock, int channel);
void mbRTURetransmit(int channel);
int mbRTUtoTCP(uint8_t sock, int channel);
int mbASCIItoTCP(uint8_t sock, int channel);
int mbTCPtoASCII(uint8_t sock, int channel);
#endif

