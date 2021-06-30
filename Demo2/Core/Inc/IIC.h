//使用IIC1 挂载M24C02,OLED,LM75AD,HT1382    PB6,PB7
#ifndef IIC_H
#define IIC_H 

#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"

#include "stm32f0xx_ll_exti.h"

 typedef unsigned char      uint8_t;
 typedef unsigned char      u8;
typedef unsigned short     uint16_t;
typedef unsigned short     u16;
typedef unsigned long     u32;

#define CH1_10HZ 0x01
#define CH1_40HZ 0x02
#define CH2_TEMP 0x03
#define CH1_10HZ_CLK 25
#define CH1_40HZ_CLK 27
#define CH2_TEMP_CLK 26

#define TM7711_SCK_H  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
#define TM7711_SCK_L  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
#define TM7711_SCK_H2  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
#define TM7711_SCK_L2  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
//#define TM7711_SDA_H  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
//#define TM7711_SDA_L   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
#define TM7711_SDA_R  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define TM7711_SDA_R2  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)

void TM7711_Port_Init(void);
unsigned long Read_TM7711(unsigned char next_select);
unsigned long Read_TM7711_2(unsigned char next_select);


#endif /* IIC_H */
