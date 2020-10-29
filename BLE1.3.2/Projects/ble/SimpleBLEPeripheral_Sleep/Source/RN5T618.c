/*
config list
ldo1	2.8V 	(default)
ldo2	3.3V	(nfc)
ldo3	3.3V  Heart Sensor


*/

#include "RN5T618.h"



void PowerRN5T618_init(void)
{
    #if(0)
    uint8_t	RN5t618_wr_buf[1];
    #endif

   SYS_POWER();
    
    #if(0)
    RN5t618_wr_buf[0] = 0x01;
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,PWRIREN,RN5T618_Add);
    RN5t618_wr_buf[0] = 0x00;
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,PWRIRQ,RN5T618_Add);

    //电源管理 中断初始化
	//1.设置中断触发模式
	//2.清中断标志位
	RN5t618_wr_buf[0] = 0x00;
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,CHGCTRL_IRR,RN5T618_Add);
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,CHGSTAT_IRR1,RN5T618_Add);
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,CHGSTAT_IRR2,RN5T618_Add);
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,CHGERR_IRR,RN5T618_Add);

    //3.设置中断屏蔽位
	RN5t618_wr_buf[0] = 0x00;
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,CHGCTRL_IRFMASK,RN5T618_Add);
    
    //4.打开中断使能
	RN5t618_wr_buf[0] = 0x41;
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,INTEN,RN5T618_Add);

	RN5t618_wr_buf[0] = 0x05;//充电电流600ma
    I2C_WriteHaveAdd(&RN5T618_I2C,&RN5t618_wr_buf[0],1,USB_Limit_Current,RN5T618_Add);
    #endif
}


//set mcu voltage 2.8V
void SYS_POWER(void)
{

  PowerOn_MainMcu();
  PowerOn_NFC();
  PowerOn_HeartSensor();
    
    
}
//ldo1
void PowerOn_MainMcu(void)
{
  uint8_t buffer[3];
  buffer[2] = 0x4C;//0x4c-2.8V
      
  buffer[1] = 0x01;
  
  HalI2CWriteReg(RN5T618_Add, LDO1DAC, 1, &buffer[2]);
  HalI2CWriteReg(RN5T618_Add, LDOEN1, 1, &buffer[1]);
  
  HalI2CReadReg_2(RN5T618_Add, LDOEN1, 1, &buffer[0]);
  HalI2CReadReg_2(RN5T618_Add, VSYSSET, 1, &buffer[0]);
  
  buffer[0] = 0x05;//充电电流600ma
    HalI2CWriteReg(RN5T618_Add, USB_Limit_Current, 1, &buffer[0]);
    
    buffer[0] = 0x05;//充电电流600ma
    HalI2CWriteReg(RN5T618_Add, USB_Charge_Current, 1, &buffer[0]);
    
    buffer[0] = 0x87;//0xa7;//USB CHARGE
    HalI2CWriteReg(RN5T618_Add, CHGCONTROL, 1, &buffer[0]);
    
    HalI2CReadReg_2(RN5T618_Add, USB_Charge_Current, 1, &buffer[0]);
   // I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,USB_Charge_Current,RN5T618_Add);
    HalI2CReadReg_2(RN5T618_Add, CHGCONTROL, 1, &buffer[0]);
  //  I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,CHGCONTROL,RN5T618_Add);  

}
//ldo2
void PowerOn_NFC(void)
{
  uint8_t buffer[3];
  buffer[2] = 0x60;//0x4c;3.3V(CC2541)

  HalI2CReadReg_2(RN5T618_Add, LDOEN1, 1, &buffer[0]);

  buffer[1] = 0X07;//buffer[0]|0x02;
  HalI2CWriteReg(RN5T618_Add, LDO2DAC, 1, &buffer[2]);
  HalI2CWriteReg(RN5T618_Add, LDOEN1, 1, &buffer[1]);  
}
//ldo2
void PowerOff_NFC(void)
{
  uint8_t buffer[2];	
  HalI2CReadReg_2(RN5T618_Add, LDOEN1, 1, &buffer[0]);
  buffer[1] = buffer[0]&0XFD;
  HalI2CWriteReg(RN5T618_Add, LDOEN1, 1, &buffer[1]);  
}

//ldo3
void PowerOn_HeartSensor(void)
{
  uint8_t buffer[3];
  buffer[2] = 0x60;//60;
  HalI2CReadReg_2(RN5T618_Add, LDOEN1, 1, &buffer[0]);
  
  buffer[1] = 0X07;//buffer[0]|0x04;
  HalI2CWriteReg(RN5T618_Add, LDO3DAC, 1, &buffer[2]);
  HalI2CWriteReg(RN5T618_Add, LDOEN1, 1, &buffer[1]);  
}
//ldo3
void PowerOff_HeartSensor(void)
{
  uint8_t buffer[2];
  HalI2CReadReg_2(RN5T618_Add, LDOEN1, 1, &buffer[0]);
  buffer[1] = buffer[0]&0XFB;
  HalI2CWriteReg(RN5T618_Add, LDOEN1, 1, &buffer[1]);  
}


void ShoutDown_SYS(void)
{
  uint8_t buffer[1];
  buffer[0] = 0x01;
  HalI2CWriteReg(RN5T618_Add, SLPCNT, 1, &buffer[0]);  //POWER OFF	 
}
//获取电池电压
float Get_BatVoltage(void)
{
  float tmpf;

  uint8_t buffer[3];
  buffer[2] = 0x01;
  HalI2CWriteReg(RN5T618_Add, FG_CTRL, 1, &buffer[2]);
  HalI2CReadReg_2(RN5T618_Add, VOLTAGE_1, 1, &buffer[0]);
  HalI2CReadReg_2(RN5T618_Add, VOLTAGE_0, 1, &buffer[1]);
  //I2C_WriteHaveAdd(&RN5T618_I2C,&buffer[2],1,FG_CTRL,RN5T618_Add);// enable fuel gauge
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,VOLTAGE_1,RN5T618_Add);
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[1],1,VOLTAGE_0,RN5T618_Add);
  tmpf = (buffer[0]*256+buffer[1])*1.22f;

  return tmpf;
}

unsigned int Get_BatCap(void)
{
/*  unsigned int fullcapvalue;
  unsigned int avaicapvalue;
  float batcapvalue;
  uint8_t buffer[2];
  
  HalI2CReadReg_2(RN5T618_Add, FACAP_H, 1, &buffer[0]);
  HalI2CReadReg_2(RN5T618_Add, FACAP_L, 1, &buffer[1]);
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,FACAP_H,RN5T618_Add);
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[1],1,FACAP_L,RN5T618_Add);
  fullcapvalue = ((buffer[0]&0X00FF)<<8) + buffer[1];
  //fullcapvalue -= 2000;
  HalI2CReadReg_2(RN5T618_Add, RECAP_H, 1, &buffer[0]);
  HalI2CReadReg_2(RN5T618_Add, RECAP_L, 1, &buffer[1]);
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,RECAP_H,RN5T618_Add);
  //I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[1],1,RECAP_L,RN5T618_Add);
  avaicapvalue = ((buffer[0]&0X00FF)<<8) + buffer[1];
 // avaicapvalue -= 257;

  batcapvalue = (float)(avaicapvalue * 100)/fullcapvalue;*/
  //	
  uint8_t	buffer[1];
  float batcapvalue;
  HalI2CReadReg_2(RN5T618_Add, 0XE1, 1, &buffer[0]);
 // I2C_ReadHaveAdd(&RN5T618_I2C,&buffer[0],1,0XE1,RN5T618_Add);
  
  batcapvalue = buffer[0];
  return (unsigned int )batcapvalue;
}

uint8_t Get_BatChargeState(void)
{
  uint8_t RD_BUF[1];
  uint8_t sta;
  //HalI2CReadReg_2(RN5T618_Add, CHGSTATE, 1, &RD_BUF[0]);
  HalI2CReadReg_3(RN5T618_Add, CHGSTATE, 1, &RD_BUF[0]);
  if((RD_BUF[0]&0x1f) == 0x00)
    sta = 0;
  else
    sta = 1;
  return sta;
}


