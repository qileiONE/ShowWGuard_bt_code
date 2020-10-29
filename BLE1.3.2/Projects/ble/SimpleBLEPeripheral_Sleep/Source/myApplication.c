#include "myApplication.h"
#include "hal_i2c.h"


static uint16 simpleBLECharHdl = 0;

/* 全局变量 macAddr, 保存连接到主机的从机设备的mac地址 */
// 使用时要在头文件中声明
uint8 *macAddr; 

// 获得从机状态
gaprole_States_t state;


/* 15分钟定时器的用户处理函数 */
void timerUserFunc()
{
  /******************************/
  //User Coding Here
  /******************************/
}

/* 读flash的函数 */
//pData: 读出数据的缓存
// dataLen: 要读出数据的长度
void Flash_read( uint8 *pData, uint8 dataLen )
{
  osal_snv_read( BLE_NVID_CUST_START, dataLen, pData );
}

/* 写flash的函数 */
//pData: 写出数据的缓存
// dataLen: 要写出数据的长度
void Flash_write( uint8 *pData, uint8 dataLen )
{
  osal_snv_write( BLE_NVID_CUST_START, dataLen, pData );
}

/* 擦除flash */
void Flash_erase(  )
{
  uint8 buffer[252];
  osal_memset( buffer, 0, 252 );
  osal_snv_write( BLE_NVID_CUST_START, 252, buffer );
}

/* 蓝牙主机发送函数接口， 注意这个是蓝牙主机的 */
// task_id: 任务的id
// sendBuff: 发送数据缓存
// dataLen: 发送数据的长度
void Blooth_send( uint8 task_id, uint8 *sendBuff, uint8 dataLen )
{
  attWriteReq_t req;
  uint8 i;
  for( i = 0; i < dataLen; i ++ ){
    req.handle = simpleBLECharHdl;
    req.len = 1;
    req.value[0] = sendBuff[i];
    req.sig = 0;
    req.cmd = 0;
    uint8 status = GATT_WriteCharValue( GAP_CONNHANDLE_INIT, &req, task_id );   
  }
}

/* 蓝牙主机接收数据接口， 这是蓝牙主机的，从机的在 SimpleBLEPeripheral.c文件里 */
/* 蓝牙主机发送函数接口 */
// task_id: 任务的id
// recvBuff: 接收数据缓存
// dataLen: 要接收数据的长度
void Blooth_recv( uint8 task_id, uint8 *recvBuff, uint8 dataLen )
{
  attReadReq_t req;
  uint8 i;
  for( i = 0; i < dataLen; i ++ ){
    req.handle = simpleBLECharHdl;

    uint8 status = GATT_ReadCharValue( GAP_CONNHANDLE_INIT, &req, task_id );   
  }
}

/* IIC初始化函数 */
// 描述：初始化一个master
// 参数 :NULL
void IIC_Init(  )
{
  HalI2CInit( i2cClock_267KHZ );
}

/* IIC 读 */
// address: 地址
// len: 数据长度
// pBuf：数据缓存
int IIC_Read( uint8 address, uint8 len, uint8 *pBuf)
{
  return HalI2CRead( address, len, pBuf );
}

/* IIC 写 */
// address：地址
// len: 数据长度
// pBuf：数据缓存
int IIC_Write( uint8 address, uint8 len, uint8 *pBuf )
{
    return HalI2CWrite( address, len, pBuf );
}

int IIC_ReadReg(uint8 address, uint8 regAddr, i2cLen_t len, uint8 *pBuf)
{
  return HalI2CReadReg( address, regAddr, len, pBuf );
}

/* 通用GPIO的初始化 */
// 描述：21个通用io的初始化函数
// 参数 port：端口
// 参数 pin：引脚
void GPIO_Init( uint8 port, uint8 pin )
{
  switch( port ){
    case 0:{
      P0DIR &= 0xFF;
      P0DIR |= 0x01 << pin;
      break;
    }
    case 1:{
      P1DIR &= 0xFF;
      P1DIR |= 0x01 << pin;
      break;
    }
    case 2:{
      P2DIR &= 0xFF;
      P2DIR |= 0x01 << pin;
      break;
    }
    default: break;
  }
}

/* 将某一个引脚置为高电平 */
void GPIO_setHigh( uint8 port, uint8 pin )
{
  switch( port ){
    case 0:{
      switch(pin){
        case 0: P0_0 = 1; break;
        case 1: P0_1 = 1; break;
        case 2: P0_2 = 1; break;
        case 3: P0_3 = 1; break;
        case 4: P0_4 = 1; break;
        case 5: P0_5 = 1; break;
        case 6: P0_6 = 1; break;
        case 7: P0_7 = 1; break;
        default:  break;
      }
      break;
    }
    case 1:{
      switch(pin){
        case 0: P1_0 = 1; break;
        case 1: P1_1 = 1; break;
        case 2: P1_2 = 1; break;
        case 3: P1_3 = 1; break;
        case 4: P1_4 = 1; break;
        case 5: P1_5 = 1; break;
        case 6: P1_6 = 1; break;
        case 7: P1_7 = 1; break;
        default:  break;
      }
      break;
    }
    case 2:{
      switch(pin){
        case 0: P2_0 = 1; break;
        case 1: P2_1 = 1; break;
        case 2: P2_2 = 1; break;
        case 3: P2_3 = 1; break;
        case 4: P2_4 = 1; break;
        default:  break;
      }
      break;
    }
    default: break;
  }
}

/* 将某一个引脚置为低电平 */
void GPIO_setLow( uint8 port, uint8 pin )
{
  switch( port ){
    case 0:{
      switch(pin){
        case 0: P0_0 = 0; break;
        case 1: P0_1 = 0; break;
        case 2: P0_2 = 0; break;
        case 3: P0_3 = 0; break;
        case 4: P0_4 = 0; break;
        case 5: P0_5 = 0; break;
        case 6: P0_6 = 0; break;
        case 7: P0_7 = 0; break;
        default:  break;
      }
      break;
    }
    case 1:{
      switch(pin){
        case 0: P1_0 = 0; break;
        case 1: P1_1 = 0; break;
        case 2: P1_2 = 0; break;
        case 3: P1_3 = 0; break;
        case 4: P1_4 = 0; break;
        case 5: P1_5 = 0; break;
        case 6: P1_6 = 0; break;
        case 7: P1_7 = 0; break;
        default:  break;
      }
      break;
    }
    case 2:{
      switch(pin){
        case 0: P2_0 = 0; break;
        case 1: P2_1 = 0; break;
        case 2: P2_2 = 0; break;
        case 3: P2_3 = 0; break;
        case 4: P2_4 = 0; break;
        default:  break;
      }
      break;
    }
    default: break;
  }
}

void Motor_ON(void)
{
  GPIO_setHigh(0,6);
}

void Motor_OFF(void)
{
  GPIO_setLow(0,6);
}
    
/* 启动一个 1ms 定时器 */
uint8 Timer_1ms_start( uint8 task_id, uint16 event_id )
{
  return osal_start_timerEx( task_id, event_id, 1 );
}

/* 定时器重装载 */
uint8 Timer_1ms_reload( uint8 task_id, uint16 event_id )
{
  return osal_start_reload_timer( task_id, event_id, 1 );
}

/* 停止定时器 */
uint8 Timer_1ms_stop( uint8 task_id, uint16 event_id )
{
  return osal_stop_timerEx( task_id, event_id );
}

/* 初始化 串口 */
// 参数 baund：波特率
// 参数 callback：回调函数，初始化成功了会执行这个函数
uint8 Uart_Init( uint32 baund, npiCBack_t callback )
{
  halUARTCfg_t serialCfg;
  switch(baund){
    case 9600: {
      serialCfg.baudRate = HAL_UART_BR_9600;
      break;
    }
    case 115200: {
      serialCfg.baudRate = HAL_UART_BR_115200;
      break;
    } 
    case 19200: {
      serialCfg.baudRate = HAL_UART_BR_19200;
      break;
    }
    case 38400: {
      serialCfg.baudRate = HAL_UART_BR_38400;
      break;
    }   
    default:break;
  }
  
  serialCfg.flowControl = HAL_UART_FLOW_OFF;
  serialCfg.callBackFunc = callback;
  serialCfg.intEnable = TRUE;
  serialCfg.configured = TRUE;
  
  return  HalUARTOpen( HAL_UART_PORT_0, &serialCfg );
}

/* 串口读 */
uint16 UART_Read( uint8 *buf, uint16 len )
{
  return( HalUARTRead( HAL_UART_PORT_0, buf, len ) );
}

/* 串口写 */
uint16 UART_Write( uint8 *buf, uint16 len )
{
  return( HalUARTWrite( HAL_UART_PORT_0, buf, len ) );
}

/* 获取从机的状态，状态有以下几个： */
//GAPROLE_INIT = 0,                       //!< Waiting to be started
//GAPROLE_STARTED,                        //!< Started but not advertising
//GAPROLE_ADVERTISING,                    //!< Currently Advertising
//GAPROLE_WAITING:,                        //!< Device is started but not advertising, is in waiting period before advertising again
//GAPROLE_WAITING_AFTER_TIMEOUT,          //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again
//GAPROLE_CONNECTED,                      //!< In a connection
//GAPROLE_ERROR   
gaprole_States_t getCentralStatus()
{
  return state;
}

/* 设置 RTC 时间，格式为格林威治时间 */
void setUTCTime(   uint8 seconds, uint8 minutes, uint8 hour, uint8 day, uint8 month, uint16 year  )
{
  UTCTime utc;
  UTCTimeStruct utcTime;
  
  utcTime.seconds = seconds;
  utcTime.minutes = minutes;
  utcTime.hour = hour;
  utcTime.day = day;
  utcTime.month = month;
  utcTime.year = year;
  
  //osal_ConvertUTCTime( &utcTime, utc );
  utc = osal_ConvertUTCSecs( &utcTime );
  osal_setClock( utc );
}

/* 初始化 ADC */
void ADC_Init()
{
  HalAdcInit();
}

void ADC_setRefference( uint8 reference )
{
  HalAdcSetReference( reference );
}

/* ADC 读函数 */
// 参数 channel 取值如下：
// HAL_ADC_CHANNEL_0          
// HAL_ADC_CHANNEL_1          
// HAL_ADC_CHANNEL_2          
// HAL_ADC_CHANNEL_3          
// HAL_ADC_CHANNEL_4          
// HAL_ADC_CHANNEL_5          
// HAL_ADC_CHANNEL_6         
// HAL_ADC_CHANNEL_7         
uint16 ADC_Read( uint8 channel )
{
  return HalAdcRead( channel, HAL_ADC_RESOLUTION_12 );
}


