#ifndef MAX30205_H_
#define MAX30205_H_

#include "myApplication.h"
//////////////////////////////////////////////////////////////////////////////////	 
//����������30205�����´���������							  
//////////////////////////////////////////////////////////////////////////////////
#define MAX30205_ADDRESS  0X90    //8bit address converted to 7bit
//һЩ�Ĵ�����ַ
#define MAX30205_TEMPERATURE 0X00
#define MAX30205_CONFIGURATION 0X01
#define MAX30205_THYST         0X02
#define MAX30205_TOS           0X03
#define IIC_WRITE 0   //���ݷ��� д��
#define IIC_READ   1   //���ݷ��� ��ȡ

#define    TempBase    2.35f
//extern float temperature;


extern unsigned char tempDevice ; //0 : MAX    1: NTC

extern   float tmpCaliK;
extern   float tmpCaliB;
 
void shutdown(void);
void max_init (void);
float GetTemperature(void);

float GetTemperature1(void);
 #endif

