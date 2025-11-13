#include <stdbool.h>

#ifndef _MBASCII_H
#define _MBASCII_H

#define MB_ASCII_START          ':'
#define MB_ASCII_DEFAULT_CR     '\r'    /*!< Default CR character for Modbus ASCII. */
#define MB_ASCII_DEFAULT_LF     '\n'    /*!< Default LF character for Modbus ASCII. */

extern volatile uint8_t mb_state_ascii_finish[];

void eMBAsciiInit(int channel);
bool MBtcp2asciiFrame(uint8_t sock, int channel);
bool MBascii2tcpFrame(int channel);
uint8_t prvucMBLRC(uint8_t * pucFrame, uint16_t usLen);
uint8_t prvucMBCHAR2BIN(uint8_t ucCharacter);
uint8_t prvucMBBIN2CHAR(uint8_t ucByte);
void ASCII_Uart_RX(int channel);
#endif

