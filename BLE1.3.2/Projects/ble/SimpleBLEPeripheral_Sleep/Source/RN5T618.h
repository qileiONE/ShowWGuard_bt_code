#ifndef __RN5T618_H
#define __RN5T618_H

#include "myApplication.h"
/* LDO */
#define RN5T618_Add     0x64
#define RN5T618_Add_WR  0x6e	//RN5T618AA
#define RN5T618_Add_RD  0x6f
#define LDOEN1					0x44	//LDO1-5 ON/OFF
#define LDOEN2					0x45	//LDORTC1-2  ON/OFF
#define LDODIS					0x46	//
#define LDO1DAC					0X4C	//LDO1 OUTPUT VOLTAGE
#define LDO2DAC					0X4D	//LDO2 OUTPUT VOLTAGE  STEP=25MA  0.9---3.5V
#define	LDO3DAC					0X4E	//LDO3 OUTPUT VOLTAGE  datasheet P103  0.6---3.5V
#define LDO4DAC					0X4F
#define LDO5DAC					0X50	//LDO4 5  0.9--3.5V
#define LDORTCDAC				0X56	//1.7---3.5V  default 3.3V
#define LDORTC2DAC			0X57	//0.9---3.5V
#define LDO1DAC_SLP			0X58	
#define LDO2DAC_SLP			0X59
#define LDO3DAC_SLP			0X5A
#define LDO4DAC_SLP			0X5B
#define LDO5DAC_SLP			0X5C
/* DCDC */
#define DC3CTL					0X30	//enable DCDC3
#define	DC3DAC					0X38	// DCDC3 3.5V=0xe8
#define DC3DAC_SLP			0X3D	//DCC3 OUTPUT VOLTAGE IN SLEEP MODE
/* POWER CONTROL */
#define PWRFUNC					0X0D
#define SLPCNT					0X0E
#define PWRONTIMSET			0X10
#define PWRIREN					0X12
#define PWRIRQ					0X13
#define	PWRMON					0X14
/*Fuel Gauge*/
#define FG_CTRL					0XE0
#define VOLTAGE_1				0XEB
#define VOLTAGE_0				0XEC	//BATTERY VOLTAGE UNIT IS 1.22MV
#define	BATTEMP_1				0XED
#define	BATTEMP_0				0XEE	// BATTERY PACK TEMPERATURE UNIT IS 0.0625¡æ
/* CHARGE */
#define BATSET1 				0XBA
#define BATSET2 				0XBB
#define CHGCONTROL				0XB3
#define VSYSSET 				0XB5
#define CHGSTATE				0XBD	// THE STATE IN CHARGING datasheet P138
#define CHGCTRL_IRFMASK	0XBE	//	This register masks interrupts for the charge control
#define	CHGSTAT_IRFMASK1	0XBF	//This register masks interrupts for the charge status.
#define	CHGSTAT_IRFMASK2	0XC0
#define	CHGERR_IRFMASK		0XC1
#define CHGCTRL_IRR			0XC2	// INTERRUPT FACTORS FOR THE CHARGE CONTROL
#define	CHGSTAT_IRR1		0XC3
#define CHGSTAT_IRR2		0XC4
#define CHGERR_IRR			0XC5	//	INTERRUPT FACTORS FOR THE CHARGE ERROR
#define	CHGERR_MONI			0xC9

/* INTC */
#define INTPOL					0X9C
#define INTEN						0X9D	// ENABLE EVERY INTERRUPT
#define	INTMON					0X9E	//	INTERRUPT MONITOR	
#define	USB_Limit_Current					0XB7
#define	USB_Charge_Current					0XB8

/*battery information*/
#define RECAP_H 	0XE2
#define RECAP_L		0XE3 
#define FACAP_H		0XE4
#define	FACAP_L		0XE5
#define AGE				0XE6

void PowerRN5T618_init(void);
void SYS_POWER(void);
void PowerOn_MainMcu(void);
void PowerOn_NFC(void);
void PowerOff_NFC(void);
void PowerOn_HeartSensor(void);
void PowerOff_HeartSensor(void);
void ShoutDown_SYS(void);
float Get_BatVoltage(void);
unsigned int Get_BatCap(void);
uint8_t Get_BatChargeState(void);

#endif
