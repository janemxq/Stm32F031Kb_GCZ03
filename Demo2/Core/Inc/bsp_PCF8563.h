/****************************************************************************
* Copyright (C), 2009-2010, 武汉扬硕电子有限公司
*
* 文件名: bsp_PCF8563.h
* 内容简述: PCF8563时钟芯片驱动
*
* 文件历史:
* 版本号 日期       作者    说明
* 01a    2009-06-29 zhg  创建该文件
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

/* 全局参数 */
typedef struct
{
	uint8 valid;		/* 日期时间有效 */
	uint16 year;		/* 年 */
	uint8 month;		/* 月 */
	uint8 day;			/* 日 */
	uint8 hour;			/* 时 */
	uint8 minute;		/* 分 */
	uint8 second;		/* 秒 */
	uint8 week;			/* 周 */
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
