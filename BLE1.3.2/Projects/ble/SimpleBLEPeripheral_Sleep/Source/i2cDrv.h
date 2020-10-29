
#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "myApplication.h"

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE   
#define FALSE   0
#endif

#define I2C_DELAY_COUNT		800

#define SCL_H()		{I2CWC |= 0x02; I2CIO |= 0x02;}
#define SCL_L()		{I2CWC |= 0x02; I2CIO &= ~0x02;}
#define SDA_H()		{I2CWC |= 0x01; I2CIO |= 0x01;}
#define SDA_L()		{I2CWC |= 0x01; I2CIO &= ~0x01;}

uint8_t SCL_READ();
uint8_t SDA_READ();

void I2C_Init();
void I2C_SR();
void I2C_Rstart();
uint8_t I2C_Start();
void I2C_Reset();
void I2C_Stop();
void I2C_Ack();
void I2C_NoAck();
uint8_t I2C_WaitAck();	 //返回为:=1有ACK,=0无ACK
void I2C_SendByte(uint8_t SendByte); //数据从高位到低位//
uint8_t I2C_ReceiveByte();  //数据从高位到低位//
uint8_t I2C_WriteHaveAdd(uint8_t *SendByte,uint8_t len,uint8_t WriteAddress, uint8_t DeviceAddress);
uint8_t I2C_WriteNoAdd(uint8_t *SendByte,uint8_t len,uint8_t DeviceAddress);
uint8_t I2C_ReadHaveAdd(uint8_t* pBuffer,uint8_t len,uint8_t ReadAddress,uint8_t DeviceAddress);
uint8_t I2C_ReadNoAdd(uint8_t* pBuffer,uint8_t len,uint8_t DeviceAddress);


#endif
