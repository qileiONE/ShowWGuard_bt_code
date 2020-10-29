
#include "i2cDrv.h"

//#define I2C_DELAY_COUNT		2000

uint8_t SCL_READ()
{
  I2CWC &= ~0x02;  //
  if(I2CIO & 0x02)
    return 1;
  else 
    return 0;
}

uint8_t SDA_READ()
{
  I2CWC &= ~0x01; 
  if(I2CIO & 0x01)
    return 1;
  else 
    return 0;
}


void I2C_Init()
{
  I2CWC |= 0X80; //GPIO functionality
  //I2CWC &= ~0x03;// input mode
  I2CWC |= 0X03; //output mode
  I2CIO &= ~0X02; //SCL = 0
  I2CIO |= 0x01; //SDA = 1
}

void I2C_Reset()
{
  I2CWC = 0x0c; 
  I2CIO = 0x00; 
}


void I2C_delay(void)
{
 // volatile uint16_t i = I2C_DELAY_COUNT;
  unsigned int a;
  a = I2C_DELAY_COUNT;
   
   while(a--);
}

void I2C_SR()
{
  SDA_H();
  //I2C_delay();
  SCL_H();
  //I2C_delay();
  SDA_L();
 // I2C_delay();
  SCL_L();
 // I2C_delay();

} 
 void I2C_Rstart()
{
  SDA_H();
  I2C_delay();
  SCL_H();
  I2C_delay();
  SDA_L();
  I2C_delay();
  SCL_L();
  I2C_delay();
}

uint8_t I2C_Start()
{
  SDA_H();
  //I2C_delay();
  SCL_H();
  I2C_delay();
  I2C_delay();
  I2C_delay();
  if(!SCL_READ())return FALSE;	//SCL��Ϊ�͵�ƽ������æ,�˳�
  SDA_L();
  if(SDA_READ()) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
  SDA_L();
  I2C_delay();
  I2C_delay();
  
  SCL_L();
  I2C_delay();
  return TRUE;  
}

void I2C_Stop()
{
  SCL_L();
 // I2C_delay();
  SDA_L();
  I2C_delay();
  I2C_delay();
  SCL_H();
  I2C_delay();
  I2C_delay();
  SDA_H();
  I2C_delay();
  I2C_delay();
  I2C_delay();
}

void I2C_Ack()
{	
  SCL_L();
  I2C_delay();
  SDA_L();
  I2C_delay();
  SCL_H();
  I2C_delay();
  SCL_L();
  I2C_delay();
}

void I2C_NoAck()
{	
  SCL_L();
  I2C_delay();
  SDA_H();
  I2C_delay();
  SCL_H();
  I2C_delay();
  I2C_delay();
  SCL_L();
  I2C_delay();
}

uint8_t I2C_WaitAck()	 //����Ϊ:=1��ACK,=0��ACK
{
  uint8_t ucErrTime = 0; 
  SCL_L();
  I2C_delay();
  SDA_H();			 
  I2C_delay();
  SCL_H();
  I2C_delay();
  I2C_delay();
  while(SDA_READ())
  {
    ucErrTime++;
    if(ucErrTime > 250)
    {//ָ��ʱ����û���յ���IC��ACK, ���г�ʱ����
            return FALSE;
    }
    //SCL_L();
   // return FALSE;
  }
  SCL_L();
  
  SDA_L();
  I2C_delay();
  return TRUE;
}

void I2C_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//
{
  uint8_t i=8;
  SCL_L();
  while(i--)
  {
  //  SCL_L();
   // I2C_delay();
    if(SendByte & 0x80)
    {
      SDA_H();  
    }
    else 
    {
      SDA_L();  
    }
    SendByte<<=1;
    I2C_delay();
    SCL_H();
    I2C_delay();
    I2C_delay();
    SCL_L();
    I2C_delay();
  }
  SCL_L();
}

uint8_t I2C_ReceiveByte()  //���ݴӸ�λ����λ//
{ 
  uint8_t i=8;
  uint8_t ReceiveByte=0;

  SDA_H();				
  while(i--)
  {
    ReceiveByte<<=1;      
    SCL_L();
    I2C_delay();
    SCL_H();
    I2C_delay();	
    if(SDA_READ())
    {
      ReceiveByte|=0x01;//���λ��1
      I2C_delay();
    }
  }
  SCL_L();
  I2C_delay();
  return ReceiveByte;
}
//�������������ݵļĴ�����ַ��
uint8_t I2C_WriteHaveAdd(uint8_t *SendByte,uint8_t len,uint8_t WriteAddress, uint8_t DeviceAddress)
{	
  //    FM24_WP_L;	
  if(!I2C_Start())return FALSE;
  I2C_SendByte(DeviceAddress & 0xFE);//���ø���ʼ��ַ+������ַ 
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(WriteAddress);   //���õ���ʼ��ַ      
  I2C_WaitAck();
      while(len--)
      {	
          I2C_SendByte(*SendByte++);
          I2C_WaitAck();
      }   
  I2C_Stop(); 	

  //	FM24_WP_H;
  return TRUE;
}
//���������������ݵļĴ�����ַ��ֱ��д����
uint8_t I2C_WriteNoAdd(uint8_t *SendByte,uint8_t len,uint8_t DeviceAddress)
{	
  //    FM24_WP_L;	
  
  if(!I2C_Start())return FALSE;
  I2C_SendByte(DeviceAddress & 0xFE);//���ø���ʼ��ַ+������ַ 
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}	
  while(len--)
      {	
          I2C_SendByte(*SendByte++);
          I2C_WaitAck();
      }   
  I2C_Stop(); 	

  
  //	FM24_WP_H;
  return TRUE;
}
//�������������ݵļĴ�����ַ��
uint8_t I2C_ReadHaveAdd(uint8_t* pBuffer,uint8_t len,uint8_t ReadAddress,uint8_t DeviceAddress)
{
  I2C_delay();
  I2C_delay();
  
  if(!I2C_Start())return FALSE;
  I2C_SendByte(DeviceAddress & 0xFE);//���ø���ʼ��ַ+������ַ 
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(ReadAddress);   //���õ���ʼ��ַ      
  I2C_WaitAck();
  //I2C_Stop();
  //I2C_SR();
  I2C_Rstart();
  I2C_SendByte(DeviceAddress | 0x01);
  I2C_WaitAck();
  while(len--)
  {
    *pBuffer++ = I2C_ReceiveByte();
    if(len == 0) I2C_NoAck();
    else I2C_Ack(); 
  }
  I2C_Stop();

  I2C_Reset();
  I2C_delay();
  return TRUE;
}
//���������������ݵļĴ�����ַ��ֱ�Ӷ�ȡ
uint8_t I2C_ReadNoAdd(uint8_t* pBuffer,uint8_t len,uint8_t DeviceAddress)
{
  if(!I2C_Start())return FALSE;
  I2C_SendByte(DeviceAddress | 0x01);
  I2C_WaitAck();
  while(len--)
  {
    *pBuffer++ = I2C_ReceiveByte();
    if(len == 0) I2C_NoAck();
    else I2C_Ack(); 
  }
  I2C_Stop();
  return TRUE;
}
