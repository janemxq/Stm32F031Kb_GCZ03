#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

 
#define	SPI_RCC					RCC_AHBPeriph_GPIOB
#define SPI_GPIOx				GPIOB
 
#define SPI_NSS_PIN			GPIO_Pin_12				
#define SPI_CLK_PIN			GPIO_Pin_13				
#define SPI_MISO_PIN		GPIO_Pin_14				
#define SPI_MOSI_PIN		GPIO_Pin_15				
 
#define SPI2_CS_ENABLE 	GPIO_ResetBits(SPI_GPIOx, SPI_NSS_PIN)
#define SPI2_CS_DISABLE GPIO_SetBits(SPI_GPIOx, SPI_NSS_PIN)
 

void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节


void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
		 
#endif

