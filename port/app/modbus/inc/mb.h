#ifndef _MB_H_
#define _MB_H_

void mbTCPtoRTU(uint8_t sock, int channel);
void mbRTUtoTCP(uint8_t sock, int channel);
void mbASCIItoTCP(uint8_t sock, int channel);
void mbTCPtoASCII(uint8_t sock, int channel);
#endif

