//#define EEPROM_OP_C

#include "hal_flash.h"
#include "eeprom.h"

 uint8_t nfccard_flag;


uint32_t readEeprom(uint16_t add)
{
  uint32_t read;
  uint16_t add_offset;
  uint8_t read_buf_temp[4] = {0};
  if((add >= EEPROM_CALI_ADD)&&(add < EEPROM_BASE1_ADD))
  {
    add_offset = add - EEPROM_CALI_ADD;
    add_offset = add_offset * 4;
    HalFlashRead(FLASH_ADD_PAGE1,add_offset,read_buf_temp,4);
  }
  else if((add >= EEPROM_BASE1_ADD)&&(add < EEPROM_BASE2_ADD))
  {
    add_offset = add - EEPROM_BASE1_ADD;
    add_offset = add_offset * 4;
    HalFlashRead(FLASH_ADD_PAGE2,add_offset,read_buf_temp,4);
  }
  else if((add >= EEPROM_BASE2_ADD)&&(add < EEPROM_BASE3_ADD))
  {
    add_offset = add - EEPROM_BASE2_ADD;
    add_offset = add_offset * 4;
    HalFlashRead(FLASH_ADD_PAGE3,add_offset,read_buf_temp,4);
  }
  else 
  {
    read = 0xffffffff;
  }
  read = ((uint32_t )read_buf_temp[0] << 24) + ((uint32_t )read_buf_temp[1] << 16) + \
         ((uint32_t )(read_buf_temp[2] << 8)) + read_buf_temp[3];
  return read;
}
uint8_t writeEeprom(uint16_t add,uint32_t data)
{
  uint8_t err;
 // uint32_t read;
 // uint16_t add_offset;
  uint8_t write_buf_temp[4] = {0};
  
  write_buf_temp[0] = (uint8_t)((data & 0xff000000) >> 24);
  write_buf_temp[1] = (uint8_t)((data & 0x00ff0000) >> 16);
  write_buf_temp[2] = (uint8_t)((data & 0x0000ff00) >> 8);
  write_buf_temp[3] = (uint8_t)(data & 0x000000ff);
  if((add >= EEPROM_CALI_ADD)&&(add < EEPROM_BASE1_ADD))
  {
    HalFlashWrite(add,write_buf_temp,1);
    err = 0;
  }
  else if((add >= EEPROM_BASE1_ADD)&&(add < EEPROM_BASE2_ADD))
  {
    HalFlashWrite(add,write_buf_temp,1);
    err = 0;
  }
  else if((add >= EEPROM_BASE2_ADD)&&(add < EEPROM_BASE3_ADD))
  {
    HalFlashWrite(add,write_buf_temp,1);
    err = 0;
  }
  else 
  {
    err = 1;
  }  
  return err;
}
void ClearEeprom(uint16_t add)
{
  if((add >= EEPROM_CALI_ADD)&&(add < EEPROM_BASE1_ADD))
  {
    HalFlashErase(FLASH_ADD_PAGE1);
  }
  else if((add >= EEPROM_BASE1_ADD)&&(add < EEPROM_BASE2_ADD))
  {
    HalFlashErase(FLASH_ADD_PAGE2);
  }
  else if((add >= EEPROM_BASE2_ADD)&&(add < EEPROM_BASE3_ADD))
  {
    HalFlashErase(FLASH_ADD_PAGE3);
  }
  else 
  {
    
  } 
}

uint16_t readTempPoint(uint16_t year,uint16_t month,uint16_t day,uint16_t num)
{
  uint16_t data = 0xffff;
  uint32_t date;

  date = ((uint32_t )year << 16) + (month << 8) + day;
  if(readEeprom(EEPROM_BASE1_ADD) == date)
  {
    data = readEeprom(EEPROM_BASE1_ADD + num);
  }
  else if(readEeprom(EEPROM_BASE2_ADD) == date)
  {
    data = readEeprom(EEPROM_BASE2_ADD + num);
  }

  return data;
}
uint16_t saveTempPoint(uint16_t year,uint16_t month,uint16_t day,uint16_t num,uint16_t data)
{
  uint8_t err=0;
  uint8_t flag=0;
  uint32_t date;
  uint32_t date1,date2;
  uint32_t add;

  if((num == 0)||(num > T_MAX_POINT))
    return 101;

  date = ((uint32_t )year << 16) + (month << 8) + day;
  date1 = readEeprom(EEPROM_BASE1_ADD);

  if(date1 == date)
  {
    err = writeEeprom(EEPROM_BASE1_ADD + num,data);
  }
  else if(date1 == 0xffffffff)
  {
    writeEeprom(EEPROM_BASE1_ADD,date);
    err = writeEeprom(EEPROM_BASE1_ADD + num,data);
  }
  else
    flag = 1;
  if(flag == 1)//&&(err != 0))
  {
    flag = 0;
    date2 = readEeprom(EEPROM_BASE2_ADD);
    if(date2 == date)
    {
      err = writeEeprom(EEPROM_BASE2_ADD + num,data);
    }
    else if(date2 == 0xffffffff)
    {
      writeEeprom(EEPROM_BASE2_ADD,date);
      err = writeEeprom(EEPROM_BASE2_ADD + num,data);
    }
    else
      flag = 1;
  }
  if(flag == 1)//&&(err != 0))
  {
    if(date1 < date2)
      add = EEPROM_BASE1_ADD;
    else
      add = EEPROM_BASE2_ADD;
    ClearEeprom(add);
    writeEeprom(add,date);
    err = writeEeprom(add + num,data);
  }
  return err;
}

void saveTempCali(float  k,float b)
{
 // ClearEeprom(EEPROM_CALI_ADD);
  writeEeprom(EEPROM_CALI_ADD,(int32_t)(k*10000));
  writeEeprom(EEPROM_CALI_ADD+1,(int32_t)(b*10000));
}
void getTempCali(float *k,float *b)
{
  int32_t tmp1,tmp2;
  tmp1 = (int32_t)readEeprom(EEPROM_CALI_ADD);
  tmp2 = (int32_t)readEeprom(EEPROM_CALI_ADD+1);
  
//    if((tmp1 > 200)||(tmp1 < 50))
//        tmp1 = 100;
//    if((tmp2 > 200)||(tmp2 < -200))
//        tmp2 = 0;
  *k = (float)tmp1/10000;
  *b = (float)tmp2/10000;
}


void saveNFCCardFlag(void)
{//nfccard_flag
 // ClearEeprom(EEPROM_NFCCARD_ADD);
  writeEeprom(EEPROM_NFCCARD_ADD,nfccard_flag);
}

void getNFCCardFlag(void)
{
  int32_t tmp1;
  tmp1 = (int32_t)readEeprom(EEPROM_NFCCARD_ADD);

  if((tmp1 != 1) && (tmp1 != 0))
  {
    nfccard_flag = 0;
  }
  else
  {
    nfccard_flag = tmp1;
  }
}
