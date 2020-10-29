
#ifndef EEPROM_OP_H
#define EEPROM_OP_H

#include "myApplication.h"



#define FLASH_ADD_PAGE1         0X78
#define FLASH_ADD_PAGE2         0X79
#define FLASH_ADD_PAGE3         0X7a

#define EEPROM_CALI_ADD         ((uint16_t)0xf000)//0x78 30
#define	EEPROM_NFCCARD_ADD	((uint16_t)0xf01e)//0x78 2048 - 30 
#define EEPROM_BASE1_ADD        ((uint16_t)0xf200)//0x79 1024
#define	EEPROM_BASE2_ADD        ((uint16_t)0xf400)//0x7a 1024
#define	EEPROM_BASE3_ADD        ((uint16_t)0xf600)//0x7b 2048  ²»¿É²Ù×÷

#define FLASH_PAGE_SIZE    ((uint16_t)0x800)

#define	T_MAX_POINT			100


extern uint8_t nfccard_flag;

uint32_t readEeprom(uint16_t add);
uint8_t writeEeprom(uint16_t add,uint32_t data);
void ClearEeprom(uint16_t add);

uint16_t readTempPoint(uint16_t year,uint16_t month,uint16_t day,uint16_t num);
uint16_t saveTempPoint(uint16_t year,uint16_t month,uint16_t day,uint16_t num,uint16_t data);
void saveTempCali(float  k,float b);
void getTempCali(float *k,float *b);
void saveNFCCardFlag(void);
void getNFCCardFlag(void);
#endif
