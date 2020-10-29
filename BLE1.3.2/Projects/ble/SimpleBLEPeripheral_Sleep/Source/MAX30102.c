#include "MAX30102.h"



uint32_t aun_ir_buffer[HEART_SENSOR_DATA_LEN]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[HEART_SENSOR_DATA_LEN];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint32_t un_min;
uint32_t un_max;
uint32_t un_prev_data;
uint32_t un_brightness;

/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
uint8 maxim_max30102_write_reg(uint8 uch_addr, uint8 uch_data)
{
  uint8 ret;

  ret = HalI2CWriteReg(I2C_WRITE_ADDR,uch_addr,1,&uch_data);
//  ret = ret +1;
  if(!ret)
      return 1; //发送失败
  else
      return 0; //发送成功
  //return 0;
}
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
uint8 maxim_max30102_read_reg(uint8 uch_addr, uint8 *puch_data)

{

    uint8 ret;
    
    ret = HalI2CReadReg(I2C_WRITE_ADDR,uch_addr,1,puch_data);
    
    if(!ret)//如果
        return 1; //发送失败
    else
        return 0; //发送成功


}

void max30102_GPIO_Init(void)
{
  P1SEL &= ~0x02;     //设置P0.1为普通IO口  
  P1DIR &= ~0x02;     //按键接在P0.1口上，设P0.1为输入模式 
  P1INP &= ~0x02;     //打开P0.1上拉电阻
}

/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
void  maxim_max30102_init(void)
{
  max30102_GPIO_Init();//初始化INT引脚
  maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0); // INTR setting
  maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00);
  maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00);  //FIFO_WR_PTR[4:0]
  maxim_max30102_write_reg(REG_OVF_COUNTER,0x00);  //OVF_COUNTER[4:0]
  maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00);  //FIFO_RD_PTR[4:0]
  maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f);  //sample avg = 1, fifo rollover=-1, fifo almost full = 17
  maxim_max30102_write_reg(REG_MODE_CONFIG,0x03);   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
  maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
  maxim_max30102_write_reg(REG_LED1_PA,0x24);   //Choose value for ~ 7mA for LED1
  maxim_max30102_write_reg(REG_LED2_PA,0x24);   // Choose value for ~ 7mA for LED2
  maxim_max30102_write_reg(REG_PILOT_PA,0x7f);  // Choose value for ~ 25mA for Pilot LED
 // return 0;  //返回成功
}

/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
uint8 maxim_max30102_read_fifo(uint32 *pun_red_led, uint32 *pun_ir_led)
{
  uint8 ret;

  ret = HalI2CMax30102ReadFifo(I2C_WRITE_ADDR,REG_FIFO_DATA,pun_red_led,pun_ir_led);
  if(!ret)//如果
      return 1; //发送失败
  else
      return 0; //发送成功

}

/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
uint8 maxim_max30102_reset()
{
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return 1;
    else
        return 0; //返回成功
}
