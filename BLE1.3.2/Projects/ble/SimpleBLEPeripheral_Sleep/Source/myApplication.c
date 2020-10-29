#include "myApplication.h"
#include "hal_i2c.h"


static uint16 simpleBLECharHdl = 0;

/* ȫ�ֱ��� macAddr, �������ӵ������Ĵӻ��豸��mac��ַ */
// ʹ��ʱҪ��ͷ�ļ�������
uint8 *macAddr; 

// ��ôӻ�״̬
gaprole_States_t state;


/* 15���Ӷ�ʱ�����û������� */
void timerUserFunc()
{
  /******************************/
  //User Coding Here
  /******************************/
}

/* ��flash�ĺ��� */
//pData: �������ݵĻ���
// dataLen: Ҫ�������ݵĳ���
void Flash_read( uint8 *pData, uint8 dataLen )
{
  osal_snv_read( BLE_NVID_CUST_START, dataLen, pData );
}

/* дflash�ĺ��� */
//pData: д�����ݵĻ���
// dataLen: Ҫд�����ݵĳ���
void Flash_write( uint8 *pData, uint8 dataLen )
{
  osal_snv_write( BLE_NVID_CUST_START, dataLen, pData );
}

/* ����flash */
void Flash_erase(  )
{
  uint8 buffer[252];
  osal_memset( buffer, 0, 252 );
  osal_snv_write( BLE_NVID_CUST_START, 252, buffer );
}

/* �����������ͺ����ӿڣ� ע����������������� */
// task_id: �����id
// sendBuff: �������ݻ���
// dataLen: �������ݵĳ���
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

/* ���������������ݽӿڣ� �������������ģ��ӻ����� SimpleBLEPeripheral.c�ļ��� */
/* �����������ͺ����ӿ� */
// task_id: �����id
// recvBuff: �������ݻ���
// dataLen: Ҫ�������ݵĳ���
void Blooth_recv( uint8 task_id, uint8 *recvBuff, uint8 dataLen )
{
  attReadReq_t req;
  uint8 i;
  for( i = 0; i < dataLen; i ++ ){
    req.handle = simpleBLECharHdl;

    uint8 status = GATT_ReadCharValue( GAP_CONNHANDLE_INIT, &req, task_id );   
  }
}

/* IIC��ʼ������ */
// ��������ʼ��һ��master
// ���� :NULL
void IIC_Init(  )
{
  HalI2CInit( i2cClock_267KHZ );
}

/* IIC �� */
// address: ��ַ
// len: ���ݳ���
// pBuf�����ݻ���
int IIC_Read( uint8 address, uint8 len, uint8 *pBuf)
{
  return HalI2CRead( address, len, pBuf );
}

/* IIC д */
// address����ַ
// len: ���ݳ���
// pBuf�����ݻ���
int IIC_Write( uint8 address, uint8 len, uint8 *pBuf )
{
    return HalI2CWrite( address, len, pBuf );
}

int IIC_ReadReg(uint8 address, uint8 regAddr, i2cLen_t len, uint8 *pBuf)
{
  return HalI2CReadReg( address, regAddr, len, pBuf );
}

/* ͨ��GPIO�ĳ�ʼ�� */
// ������21��ͨ��io�ĳ�ʼ������
// ���� port���˿�
// ���� pin������
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

/* ��ĳһ��������Ϊ�ߵ�ƽ */
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

/* ��ĳһ��������Ϊ�͵�ƽ */
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
    
/* ����һ�� 1ms ��ʱ�� */
uint8 Timer_1ms_start( uint8 task_id, uint16 event_id )
{
  return osal_start_timerEx( task_id, event_id, 1 );
}

/* ��ʱ����װ�� */
uint8 Timer_1ms_reload( uint8 task_id, uint16 event_id )
{
  return osal_start_reload_timer( task_id, event_id, 1 );
}

/* ֹͣ��ʱ�� */
uint8 Timer_1ms_stop( uint8 task_id, uint16 event_id )
{
  return osal_stop_timerEx( task_id, event_id );
}

/* ��ʼ�� ���� */
// ���� baund��������
// ���� callback���ص���������ʼ���ɹ��˻�ִ���������
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

/* ���ڶ� */
uint16 UART_Read( uint8 *buf, uint16 len )
{
  return( HalUARTRead( HAL_UART_PORT_0, buf, len ) );
}

/* ����д */
uint16 UART_Write( uint8 *buf, uint16 len )
{
  return( HalUARTWrite( HAL_UART_PORT_0, buf, len ) );
}

/* ��ȡ�ӻ���״̬��״̬�����¼����� */
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

/* ���� RTC ʱ�䣬��ʽΪ��������ʱ�� */
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

/* ��ʼ�� ADC */
void ADC_Init()
{
  HalAdcInit();
}

void ADC_setRefference( uint8 reference )
{
  HalAdcSetReference( reference );
}

/* ADC ������ */
// ���� channel ȡֵ���£�
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


