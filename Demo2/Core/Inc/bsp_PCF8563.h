/****************************************************************************
* Copyright (C), 2009-2010, �人��˶�������޹�˾
*
* �ļ���: bsp_PCF8563.h
* ���ݼ���: PCF8563ʱ��оƬ����
*
* �ļ���ʷ:
* �汾�� ����       ����    ˵��
* 01a    2009-06-29 zhg  �������ļ�
*
*/

#ifndef _BSP_PCF8563_H_
#define _BSP_PCF8563_H_

//#include "TypeDef.h"
#define uint8    char
#define uint16   unsigned int
#define FALSE    0
#define TRUE     1
#define RTC_YEAR_MIN	2009
#define RTC_YEAR_MAX	2099
#define RTC_MONTH_MIN   1		/*  */
#define RTC_DAY_MIN   	1		/*  */

#define RTC_HOUR_MIN   	0		/*  */
#define RTC_MINUTE_MIN 	0		/*  */
#define RTC_SECOND_MIN  0		/*  */

/* Base day (1901.1.1 DOW = 2), which is used for day calculate */
#define RTC_BASEYEAR	1901
#define RTC_BASEMONTH	1
#define RTC_BASEDAY		1
#define RTC_BASEDOW		2

/* ȫ�ֲ��� */
typedef struct
{
	uint8 valid;		/* ����ʱ����Ч */
	uint16 year;		/* �� */
	uint8 month;		/* �� */
	uint8 day;			/* �� */
	uint8 hour;			/* ʱ */
	uint8 minute;		/* �� */
	uint8 second;		/* �� */
	uint8 week;			/* �� */
}
RTC_T;

void pcf8563_InitI2c(void);
uint8 pcf8563_SetDateTime(RTC_T *_tRtc);
uint8 pcf8563_GetDateTime(RTC_T *_tRtc);
uint8 IsValidDateTime(RTC_T *_tRtc);
int GetDOW (uint16 year, uint8 month, uint8 day);
void InitRtcStruct(RTC_T *_tRtc);
void I2C_Configuration(void);
void I2C_BufferWrite(uint8 _ucWriteAddr, uint8 *_ucaBuf, uint8 _ucLen);
void I2C_ByteWrite(uint8 _ucWriteAddr, uint8 _ucByte);
uint8 I2C_BufferRead(uint8 ReadAddr, uint8* pBuffer, uint16 NumByteToRead);

#endif
