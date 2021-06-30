/****************************************************************************
* Copyright (C), 2009-2010, �人��˶�������޹�˾
*
* �ļ���: bsp_PCF8563.c
* ���ݼ���: PCF8563ʱ��оƬ����
*
* �ļ���ʷ:
* �汾�� ����       ����    ˵��
* 01a    2009-06-29 zhg  �������ļ�
*
*/
#include "stm32f0xx.h"
#include <stm32f0xx_i2c.h>
#include <stm32f0xx_gpio.h>
#include "bsp_PCF8563.h"

#define I2C_Speed              		100000 //400000
/* 8563 I2C ���ߴӵ�ַ������0A3H��д��0A2H�� */
#define PCF8563_SLAVE_ADDRESS    0xA2


#define EEPROM_ADDR (0x50<<1)   //0xA0

#define EEPROM_SCL_GPIO_CLK   RCC_APB2Periph_GPIOB
#define EEPROM_SCL_PIN        GPIO_Pin_6
#define EEPROM_SCL_GPIO_PORT GPIOB
#define EEPROM_SCL_AF          GPIO_AF_I2C1
#define EEPROM_SCL_SOURCE           GPIO_PinSource6

#define EEPROM_SDA_GPIO_CLK   RCC_APB2Periph_GPIOB
#define EEPROM_SDA_PIN        GPIO_Pin_7
#define EEPROM_SDA_GPIO_PORT GPIOB
#define EEPROM_SDA_AF                    GPIO_AF_I2C1
#define EEPROM_SDA_SOURCE  GPIO_PinSource7

#define EEPROM_I2C_CLK  RCC_APB1Periph_I2C1
#define EEPROM_I2C                    I2C1

// PB7 SDA  PB6 SCL 

/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : I2C Configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Configuration(void)
{
	
		
  I2C_InitTypeDef  I2C_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//??????gpio?????????????,???????????,???????????????,??????,?????
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	//
	
	GPIO_InitStructure.Pin = GPIO_Pin_6|GPIO_Pin_7; 	//??PB_6?SCL??
	GPIO_InitStructure.Speed = GPIO_Speed_50MHz;	 //?????50MHZ
	GPIO_InitStructure.Mode = GPIO_Mode_AF_OD;	 //???????????
	GPIO_Init(GPIOB,&GPIO_InitStructure);				 //???GPIOB???
	
  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = PCF8563_SLAVE_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
	/* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
	
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  
  
	
	
}

/*******************************************************************************
* Function Name  : I2C_ByteWrite
* Description    : Writes one byte to the I2C EEPROM.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_BufferWrite(uint8 _ucWriteAddr, uint8 *_ucaBuf, uint8 _ucLen)
{								
	int i;

/*                       - I2C_EVENT_SLAVE_ADDRESS_MATCHED   : EV1
*                       - I2C_EVENT_SLAVE_BYTE_RECEIVED     : EV2
*                       - I2C_EVENT_SLAVE_BYTE_TRANSMITTED  : EV3
*                       - I2C_EVENT_SLAVE_ACK_FAILURE       : EV3-2
*                       - I2C_EVENT_MASTER_MODE_SELECT      : EV5
*                       - I2C_EVENT_MASTER_MODE_SELECTED    : EV6
*                       - I2C_EVENT_MASTER_BYTE_RECEIVED    : EV7
*                       - I2C_EVENT_MASTER_BYTE_TRANSMITTED : EV8
*                       - I2C_EVENT_MASTER_MODE_ADDRESS10   : EV9
*                       - I2C_EVENT_SLAVE_STOP_DETECTED     : EV4
*/
	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
	
	/* Send address for write */
	I2C_Send7bitAddress(I2C1, PCF8563_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	  
	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2C1, _ucWriteAddr);
	
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	/* Send the byte to be written */
	
	for (i = 0; i < _ucLen; i++)
	{
		I2C_SendData(I2C1, _ucaBuf[i]); 
	
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}	
	
	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
}

void I2C_ByteWrite(uint8 _ucWriteAddr, uint8 _ucByte)
{
	I2C_BufferWrite(_ucWriteAddr, &_ucByte, 1);
}

/*******************************************************************************
* Function Name  : I2C_EE_BufferRead
* Description    : Reads a block of data from the EEPROM.
* Input          : - pBuffer : pointer to the buffer that receives the data read 
*                    from the EEPROM.
*                  - ReadAddr : EEPROM's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the EEPROM.
* Output         : None
* Return         : None
*******************************************************************************/
uint8 I2C_BufferRead(uint8 ReadAddr, uint8* pBuffer, uint16 NumByteToRead)
{  
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
   
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, PCF8563_SLAVE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, ReadAddr);  

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send 8563 address for read */
  I2C_Send7bitAddress(I2C1, PCF8563_SLAVE_ADDRESS, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;        
    }   
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  return TRUE;
}

/****************************************************************************
* ������: pcf8563_InitI2c
* ��  ��: ��ʼ��I2CӲ���豸.
* ��  ��: 2������.
* ��  ��: ��.
* ��  ��: BCD��.
*/
void pcf8563_InitI2c(void)
{
	I2C_Configuration();
}

/****************************************************************************
* ������: ByteToPbcd
* ��  ��: ����������ת����BCD��.
* ��  ��: 2������.
* ��  ��: ��.
* ��  ��: BCD��.
*/
uint8 pcf8563_ByteToPbcd
(
	uint8 _ucHex						/* �������� */
)
{
	return ((_ucHex / 10) << 4) + (_ucHex % 10);
}

/****************************************************************************
* ������: PbcdToByte
* ��  ��: ��PBCD����ת����HEX������.
* ��  ��: bPbcd ��ת����PBCD����.
* ��  ��: ��.
* ��  ��: uint8 ת�����HEX����.
*/
uint8 pcf8563_PbcdToByte
(
	uint8 _ucPbcd						/* ��ת����PBCD���� */
)
{
   	return ((10 * (_ucPbcd >> 4)) + (_ucPbcd & 0x0f));
}

/****************************************************************************
* ������: IsLeapYear
* ��  ��: �ж�ʱ��ʱ���Ƿ���Ч
* ��  ��: year ����.
* ��  ��: ��.
* ��  ��: TRUE��ʾ����.
*/
static uint8 IsLeapYear (uint16 year)
{
	if (!(year%4) && (year%100) || !(year%400))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/****************************************************************************
* ������: IsValidDateTime
* ��  ��: �ж�ʱ��ʱ���Ƿ���Ч
* ��  ��: RTC_T ����ʱ��ṹ��
* ��  ��: ��.
* ��  ��: TRUE��ʾ����.
*/
static uint8 RTC_MonthVal[]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint8 IsValidDateTime(RTC_T *_tRtc)
{
	/* ��ֵ���ܳ���LPC2220 Ӳ��RTC�ܹ�����Χ: 1901-2099 */
	if(_tRtc->year < RTC_YEAR_MIN || _tRtc->year > RTC_YEAR_MAX)
	{
		return FALSE;
	}

	/* ���ڲ��ܴ���ָ��������������� */
	if (_tRtc->year == RTC_YEAR_MIN)
	{
		if (_tRtc->month < RTC_MONTH_MIN)
		{
			return FALSE;
		}
		else if(_tRtc->day < RTC_DAY_MIN)
		{
			return FALSE;
		}
	}

	/* ����·� */
	if((_tRtc->month < 1) || (_tRtc->month > 12))
	{
		return FALSE;
	}

	/* ���day */
	if((_tRtc->day < 1) || (_tRtc->day > 31))
	{
		return FALSE;
	}

	/* �жϴ�С�� */
	if( _tRtc->day > RTC_MonthVal[_tRtc->month - 1])
	{
		/* ����2�·�Ϊ29�� */
		if(_tRtc->month == 2)
		{
			if (IsLeapYear( _tRtc->year))
			{
				if (_tRtc->day > 29 )
				{
					return FALSE;
				}
			}
			else if (_tRtc->day > 28 )
			{
				return FALSE;
			}
		}
		else
		{
			return FALSE;
		}
	}

	/* ���ʱ���� */
	if ((_tRtc->hour > 23) || (_tRtc->minute > 59) || (_tRtc->second > 59) )
	{
		return FALSE;
	}

	return TRUE;
}

/*************************************************************************
 * Function Name: GetDOY
 * Parameters: uint16 Year
 *			uint8 month
 *			uint8 day
 *
 * Return: int
 *
 * Description: Get the day of year according to the date
 *
 *************************************************************************/
static int GetDOY (uint16 year, uint8 month, uint8 day)
{
	int DOY=0, i;

	for(i=1; i<month; i++)
		DOY+=RTC_MonthVal[i-1];
	if (month>2)
		if (IsLeapYear(year))
			DOY++;

	return (DOY+day);
}

/*************************************************************************
 * Function Name: GetDOW
 * Parameters: uint16 Year
 *			uint8 month
 *			uint8 day
 *
 * Return: int -- (0~6)
 *
 * Description: Get the day of week according to the date.
 *
 * NOTE: Year is not smaller than RTC_YEARMIN (1901).
 *
 *************************************************************************/
int GetDOW (uint16 year, uint8 month, uint8 day)
{
	int i = RTC_BASEYEAR, DOW = 0;

	for (i = RTC_BASEYEAR, DOW = 0; i < year; i++)
	{
		DOW += 365;
		if  (IsLeapYear(i))
			DOW++;
	}

	DOW +=  GetDOY (year, month, day) - 1;
	DOW = (DOW + RTC_BASEDOW) % 7;

	return DOW;
}

/****************************************************************************
* ������: pcf8563_SetDateTime
* ��  ��: �趨ʱ�亯������
* ��  ��: RTC_T ����ʱ��ṹ�壬 _tRtc->week ������룬�ɱ������Զ�����
* ��  ��: ��.
* ��  ��: TRUE��ʾ���óɹ�.
*/
uint8 pcf8563_SetDateTime(RTC_T *_tRtc)
{   
	uint8 ucBuf[16];
	
	/* ���ʱ����Ч���������� */
	if (IsValidDateTime(_tRtc) == FALSE)
	{
		return FALSE;
	}
	
	/* ����������,�������� */
	_tRtc->week = GetDOW(_tRtc->year,_tRtc->month, _tRtc->day);
	
	ucBuf[0] = 0;	/* ���ƼĴ���1, TEST1=0;��ͨģʽ,STOP=0;оƬʱ������,TESTC=0;��Դ��λ����ʧЧ*/
	ucBuf[1] = 0;	/* ��ʱ�ͱ����ж���Ч */
	
	ucBuf[2] = pcf8563_ByteToPbcd(_tRtc->second);
	ucBuf[3] = pcf8563_ByteToPbcd(_tRtc->minute);
	ucBuf[4] = pcf8563_ByteToPbcd(_tRtc->hour);
	ucBuf[5] = pcf8563_ByteToPbcd(_tRtc->day);
	ucBuf[6] = pcf8563_ByteToPbcd(_tRtc->week);	/* 0 -6 */

	/*
	����λ��C=0 ָ��������Ϊ20������C=1	ָ��������Ϊ19����
	*/
	ucBuf[7] = pcf8563_ByteToPbcd(_tRtc->month);
	
	ucBuf[8] = pcf8563_ByteToPbcd((_tRtc->year - 2000) & 0xFF);	/* ���λ */
	
	ucBuf[9] = 0x00;	/* ���ӱ����ر� */
	ucBuf[10] = 0x00;	/* Сʱ�����ر� */
	ucBuf[11] = 0x01;	/* �ձ����ر� */
	ucBuf[12] = 0x00;	/* ���ڱ����ر� */
	
	ucBuf[13] = 0x00;	/* CLKOUT Ƶ�ʼĴ�����Ч */
	ucBuf[14] = 0x00;	/* ��������ʱ���Ĵ�����Ч */
	ucBuf[15] = 0x00;	/* ��ʱ����������ֵ�Ĵ��� */
	
	I2C_BufferWrite(0, ucBuf, 16);
	return TRUE;
}       

/****************************************************************************
* ������: pcf8563_GetDateTime
* ��  ��: ��ȡPCF8563��ʱ��
* ��  ��: 
* ��  ��: RTC_T ����ʱ��ṹ�壬 _tRtc->week ������룬�ɱ������Զ�����.
* ��  ��: false��ʾ��ȡʧ��(Ӳ������)��true��ʾ��ȡ�ɹ�
*/
uint8 pcf8563_GetDateTime(RTC_T *_tRtc)
{   
	uint8 ucaBuf[16];
	
	if (I2C_BufferRead(2, ucaBuf, 7) == FALSE)
	{
		return FALSE;
	}

	_tRtc->second = pcf8563_PbcdToByte(ucaBuf[0]);
	_tRtc->minute = pcf8563_PbcdToByte(ucaBuf[1] & 0x7F);
	_tRtc->hour = pcf8563_PbcdToByte(ucaBuf[2] & 0x3F);
	_tRtc->day = pcf8563_PbcdToByte(ucaBuf[3] & 0x3F);
	//_tRtc->week = pcf8563_PbcdToByte(ucBuf[4]);
	_tRtc->month = pcf8563_PbcdToByte(ucaBuf[5] & 0x1F);
	_tRtc->year = pcf8563_PbcdToByte(ucaBuf[6]) + 2000;
	
	/* ����������,�������� */
	_tRtc->week = GetDOW(_tRtc->year,_tRtc->month, _tRtc->day);
	
	return TRUE;
}  

/****************************************************************************
* ������: InitRtcStruct
* ��  ��: ��ʼ��ʱ�ӽṹ��
* ��  ��: RTC_T �ṹָ��
* ��  ��: ��.
* ��  ��: ��.
*/
void InitRtcStruct(RTC_T *_tRtc)
{
	_tRtc->year = 2009;
	_tRtc->month = 6;
	_tRtc->day = 29;
	_tRtc->hour = 16;
	_tRtc->minute = 17;
	_tRtc->second = 00;
	_tRtc->week = 0;	/* 0 - 6 */

	_tRtc->valid = 0;	/* ʱ����Ч */
}
