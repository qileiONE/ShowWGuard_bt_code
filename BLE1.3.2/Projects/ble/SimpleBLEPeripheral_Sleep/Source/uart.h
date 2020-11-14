#ifndef UART_H_
#define UART_H_

#include <ioCC2540.h>
#include <string.h>

void InitUart(void);
void UartSendString(char *Data, int len);
unsigned int crc_calc(unsigned char ddata[],unsigned char l);


 #endif
