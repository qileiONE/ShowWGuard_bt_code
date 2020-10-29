#ifndef __MY_APPLICATION_H_
#define __MY_APPLICATION_H_

#include <ioCC2540.h>

#include "bcomdef.h"
#include "osal_snv.h"
//#include "snv_flash.h"
#include "OSAL_Memory.h"
#include "OSAL.h"

#include "gatt.h"
#include "gap.h"

#include "hal_i2c.h"
#include "hal_uart.h"

#include "OSAL_Clock.h"

#include "hal_adc.h"
#include "Npi.h"
#include "peripheral.h"

#include "hal_lcd.h"

extern float tmpCaliK;
extern float tmpCaliB;

extern gaprole_States_t state;

extern uint8 *macAddr;

//typedef void (*npiCBack_t) ( uint8 port, uint8 event );

void timerUserFunc();

void Flash_read( uint8 *pData, uint8 dataLen );
void Flash_write( uint8 *pData, uint8 dataLen );
void Flash_erase();

void Blooth_send( uint8 task_id, uint8 *sendBuff, uint8 dataLen );
void Blooth_recv( uint8 task_id, uint8 *recvBuff, uint8 dataLen );

void IIC_Init(  );
int IIC_Read( uint8 address, uint8 len, uint8 *pBuf);
int IIC_Write( uint8 address, uint8 len, uint8 *pBuf );
int IIC_ReadReg(uint8 address, uint8 regAddr, i2cLen_t len, uint8 *pBuf);

void GPIO_Init( uint8 port, uint8 pin );
void GPIO_setHigh( uint8 port, uint8 pin );
void GPIO_setLow( uint8 port, uint8 pin );

uint8 Timer_1ms_start( uint8 task_id, uint16 event_id );
uint8 Timer_1ms_reload( uint8 task_id, uint16 event_id );
uint8 Timer_1ms_stop( uint8 task_id, uint16 event_id );

uint8 Uart_Init( uint32 baund, npiCBack_t callback );
uint16 UART_Read( uint8 *buf, uint16 len );
uint16 UART_Write( uint8 *buf, uint16 len );

gaprole_States_t getCentralStatus();

void setUTCTime(   uint8 seconds, uint8 minutes, uint8 hour, uint8 day, uint8 month, uint16 year  );

void ADC_Init();
void ADC_setRefference( uint8 reference );
uint16 ADC_Read( uint8 channel );
void Motor_ON(void);
void Motor_OFF(void);

#endif