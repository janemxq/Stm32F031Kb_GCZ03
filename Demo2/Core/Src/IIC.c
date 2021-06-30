
#include "IIC.h"
//extern void DelayUs(unsigned int us);
//extern TIM_HandleTypeDef htim3;
//void tx_delay_us(uint16_t nus)	//??0-8191us
//{
        // __HAL_TIM_SetCounter(&htim3, 0);//htim8

        // __HAL_TIM_ENABLE(&htim3);

        // while(__HAL_TIM_GetCounter(&htim3) < (8 * nus));//????8MHz,8???1us
        // /* Disable the Peripheral */
        // __HAL_TIM_DISABLE(&htim3);
//}
// void DelayUs(unsigned int us)
// {
//     u32 temp;
//     SysTick->LOAD=us;             //加载时间
//     SysTick->VAL=0x00;                   //清空计时器
//     SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
//     do
//     {
//         temp=SysTick->CTRL;
//     }
//     while(temp&0x01&&!(temp&(1<<16)));         //等待时间到达
//     SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;   //关闭计时器
//     SysTick->VAL=0x00;                         //清空计时器
// }
void TM7711_Port_Init(void)
{
        GPIO_InitTypeDef GPIO_InitStruct;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        //SCL
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        //SDA
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        TM7711_SCK_L;
        TM7711_SCK_L2;
        HAL_Delay(2);
}
unsigned long Read_TM7711(unsigned char next_select)
{
        unsigned char i = 0;
        unsigned long data_temp = 0;
        for (i = 0; i < 24; i++)
        {
                TM7711_SCK_H;
                data_temp <<= 1;
                tx_delay_us(5);
                if (TM7711_SDA_R == 1)
                        data_temp |= 1;
                TM7711_SCK_L;
        }
        switch (next_select)
        {
        case CH1_10HZ:
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                break;
        case CH1_40HZ:
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                tx_delay_us(1);
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                tx_delay_us(1);
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                break;
        case CH2_TEMP:
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                tx_delay_us(1);
                TM7711_SCK_H;
                tx_delay_us(1);
                TM7711_SCK_L;
                break;
        default:
                break;
        }
        return data_temp ; //5084 ´ú±í 3.3V»ù×¼ÏÂ   1mv
}
unsigned long Read_TM7711_2(unsigned char next_select)
{
        unsigned char i = 0;
        unsigned long data_temp = 0;
        for (i = 0; i < 24; i++)
        {
                TM7711_SCK_H2;
                data_temp <<= 1;
                tx_delay_us(5);
                if (TM7711_SDA_R2 == 1)
                        data_temp |= 1;
                TM7711_SCK_L2;
        }
        switch (next_select)
        {
        case CH1_10HZ:
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                break;
        case CH1_40HZ:
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                tx_delay_us(1);
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                tx_delay_us(1);
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                break;
        case CH2_TEMP:
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                tx_delay_us(1);
                TM7711_SCK_H2;
                tx_delay_us(1);
                TM7711_SCK_L2;
                break;
        default:
                break;
        }
        return data_temp ; //5084 ´ú±í 3.3V»ù×¼ÏÂ   1mv
}

