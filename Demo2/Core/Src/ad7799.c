#include "AD7799.h"			// AD7799 definitions.
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;
void spi_AD7799_init(void)
{
	AD_CS_0();
	AD_DI_1();
	AD_SCK_1();
	HAL_Delay(50);
}	
void tx_delay_us(uint16_t nus)	//??0-8191us
{
        __HAL_TIM_SetCounter(&htim3, 0);//htim8

        __HAL_TIM_ENABLE(&htim3);

        while(__HAL_TIM_GetCounter(&htim3) < (8 * nus));//????8MHz,8???1us
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(&htim3);
}

void AD7799_gpio_init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	GPIO_InitStructure.Pin = AD_CS_PIN;
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;       
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
//	HAL_GPIO_Init(AD_CS_GPIO, &GPIO_InitStructure);				//CS片选

//#if ( AD7799_INTERFACE_MODE == AD7799_INTERFACE_SPI1 )
//	//spi1 mode
//	SPI1_Init();
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);
//	
//#else
	//gpio模拟spi mode
//	GPIO_InitStructure.Pin = AD_DI_PIN;
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;  
//  GPIO_InitStructure.Pull = GPIO_NOPULL;	
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(AD_DI_GPIO, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.Pin = AD_SCK_PIN;
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;       
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(AD_SCK_GPIO, &GPIO_InitStructure);
//	
//	
//	GPIO_InitStructure.Pin = AD_DO_PIN;
//	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.Pull = GPIO_PULLUP;	
//	HAL_GPIO_Init(AD_DO_GPIO, &GPIO_InitStructure);
//	HAL_GPIO_WritePin(AD_DI_GPIO,AD_DI_PIN,GPIO_PIN_SET);

	// spi_AD7799_init();
	
//#endif   
	
	AD7799_Reset();
}
void SPI_Write(u8 * buf,u8 size)
{
//#if ( AD7799_INTERFACE_MODE == AD7799_INTERFACE_SPI1 )
//	//spi1 mode
//	int i=0;
//	for(i=0;i<size;i++)
//		SPI1_ReadWriteByte(buf[i]);
//	
//#else
	unsigned	char	ValueToWrite = 0;
    unsigned	char	i = 0;
	unsigned	char	j = 0;

	AD_SCK_1();
	__nop();
	AD_CS_1();
	__nop();
    AD_CS_0();
	__nop();

	for(i=0;i<size;i++)
 	{
	 	ValueToWrite = *(buf + i);
		for(j=0; j<8; j++)
		{
			AD_SCK_0();
			if(0x80 == (ValueToWrite & 0x80))
			{
				AD_DI_1();	  //Send one to SDO pin
			}
			else
			{
				AD_DI_0();	  //Send zero to SDO pin
			}
			__nop();
			AD_SCK_1();
			__nop();
			ValueToWrite <<= 1;	//Rotate data
		}
	}
	AD_CS_1();
//#endif
	
}
void SPI_Read(u8 * buf,u8 size)
{
//#if ( AD7799_INTERFACE_MODE == AD7799_INTERFACE_SPI1 )
//	//spi1 mode
//	int i=0;
//	for(i=0;i<size;i++)
//	buf[i]=SPI1_ReadWriteByte(0x00);
//	
//#else
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	int  	iTemp = 0;
	unsigned	char  	RotateData = 0;

	AD_SCK_1();
	__nop();
	AD_CS_1();
	__nop();
    AD_CS_0();
	__nop();

	for(j=0; j<size; j++)
	{
		for(i=0; i<8; i++)
		{
		    AD_SCK_0();
			RotateData <<= 1;		//Rotate data
			__nop();
			iTemp = AD_DO;			//Read SDI of AD7799
			 AD_SCK_1();
			if(iTemp)
			{
				RotateData |= 1;	
			}
			__nop();
		}
		*(buf + j )= RotateData;
	}	 
	AD_CS_1();
//#endif
	
}
 
void SPI_Write2(u8 * buf,u8 size,AD7799 *ad7799)
{
//#if ( AD7799_INTERFACE_MODE == AD7799_INTERFACE_SPI1 )
//	//spi1 mode
//	int i=0;
//	for(i=0;i<size;i++)
//		SPI1_ReadWriteByte(buf[i]);
//	
//#else
	unsigned	char	ValueToWrite = 0;
    unsigned	char	i = 0;
	unsigned	char	j = 0;

	HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_SET);
	//AD_SCK_1();
	__nop();
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);
	//AD_CS_1();
	__nop();
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_RESET);
    //AD_CS_0();
	__nop();

	for(i=0;i<size;i++)
 	{
	 	ValueToWrite = *(buf + i);
		for(j=0; j<8; j++)
		{
	        HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_RESET);
			//AD_SCK_0();
			if(0x80 == (ValueToWrite & 0x80))
			{
	            HAL_GPIO_WritePin(ad7799->DIPort, ad7799->DIPin, GPIO_PIN_SET);
				//AD_DI_1();	  //Send one to SDO pin
			}
			else
			{
	            HAL_GPIO_WritePin(ad7799->DIPort, ad7799->DIPin, GPIO_PIN_RESET);
				//AD_DI_0();	  //Send zero to SDO pin
			}
			__nop();
	        HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_SET);
			//AD_SCK_1();
			__nop();
			ValueToWrite <<= 1;	//Rotate data
		}
	}
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);
	//AD_CS_1();
//#endif
	
}
void SPI_Read2(u8 * buf,u8 size,AD7799 *ad7799)
{
//#if ( AD7799_INTERFACE_MODE == AD7799_INTERFACE_SPI1 )
//	//spi1 mode
//	int i=0;
//	for(i=0;i<size;i++)
//	buf[i]=SPI1_ReadWriteByte(0x00);
//	
//#else
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	int  	iTemp = 0;
	unsigned	char  	RotateData = 0;

	HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_SET);
	//AD_SCK_1();
	__nop();
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);
	//AD_CS_1();
	__nop();
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_RESET);
    //AD_CS_0();
	__nop();

	for(j=0; j<size; j++)
	{
		for(i=0; i<8; i++)
		{
		    HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_RESET);
		    //AD_SCK_0();
			RotateData <<= 1;		//Rotate data
			__nop();
			//iTemp = AD_DO;			//Read SDI of AD7799
			iTemp = HAL_GPIO_ReadPin(ad7799->DOPort, ad7799->DOPin);
			 //AD_SCK_1();
			HAL_GPIO_WritePin(ad7799->SCKPort, ad7799->SCKPin, GPIO_PIN_SET);	
			if(iTemp)
			{
				RotateData |= 1;	
			}
			__nop();
		}
		*(buf + j )= RotateData;
	}	 
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);
	//AD_CS_1();
//#endif
	
}
/***************************************************************************//**
 * @brief Initializes the AD7799 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char AD7799_Init(void)
{ 
	unsigned char status = 0x1;
	u32 ID=AD7799_GetRegisterValue(AD7799_REG_ID, 1);
	if( (ID& 0x0F) != AD7799_ID)
	{
		status = 0x0;
	}
	ID=AD7799_GetRegisterValue(AD7799_REG_OFFSET, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
		  HAL_UART_Transmit(&huart1,(uint8_t *)&ID,2,1000);//打印ID值
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_Reset(void)
{
	unsigned char dataToSend[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
	AD7799_CS_LOW;	    
	SPI_Write(dataToSend,4);
	AD7799_CS_HIGH;	
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long AD7799_GetRegisterValue(unsigned char regAddress, unsigned char size)
{
	unsigned char data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;	
	data[0] = AD7799_COMM_READ |  AD7799_COMM_ADDR(regAddress);
	AD7799_CS_LOW;  
	SPI_Write(data,1);
	SPI_Read(data,size);
	AD7799_CS_HIGH;
	if(size == 1)
	{
		receivedData += (data[0] << 0);
	}
	if(size == 2)
	{
		receivedData += (data[0] << 8);
		receivedData += (data[1] << 0);
	}
	if(size == 3)
	{
		receivedData += (data[0] << 16);
		receivedData += (data[1] << 8);
		receivedData += (data[2] << 0);
	}
    return receivedData;
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue, 
                             unsigned char size)
{
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};	
	data[0] = AD7799_COMM_WRITE |  AD7799_COMM_ADDR(regAddress);
    if(size == 1)
    {
        data[1] = (unsigned char)regValue;
    }
    if(size == 2)
    {
		data[2] = (unsigned char)((regValue & 0x0000FF) >> 0);
        data[1] = (unsigned char)((regValue & 0x00FF00) >> 8);
    }
    if(size == 3)
    {
		data[3] = (unsigned char)((regValue & 0x0000FF) >> 0);
		data[2] = (unsigned char)((regValue & 0x00FF00) >> 8);
        data[1] = (unsigned char)((regValue & 0xFF0000) >> 16);
    }
	AD7799_CS_LOW;	    
	SPI_Write(data,(1 + size));
	AD7799_CS_HIGH;

}
/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
unsigned char AD7799_Ready(void)
{
    unsigned char rdy = 0;
    rdy = (AD7799_GetRegisterValue( AD7799_REG_STAT,1) & 0x80);   
	
	return(!rdy);
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7799.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetMode(unsigned long mode,u8  rate)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
    command &= ~AD7799_MODE_SEL(0xFF); 
    command |= AD7799_MODE_SEL(mode);
	command &= 0XFFF0;
	command |= rate;			//评率最快
	
    AD7799_SetRegisterValue(
            AD7799_REG_MODE,
            command,
            2
    );

	
}
/***************************************************************************//**
 * @brief Selects the channel of AD7799.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetChannel(unsigned long channel)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_CHAN(0xFF);
    command |= AD7799_CONF_CHAN(channel);
	
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetGain(unsigned long gain)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
	
    command &= ~AD7799_CONF_GAIN(0xFF);
    command |= AD7799_CONF_GAIN(gain);
	
	
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

void AD7799_SetBurnoutCurren(u8 enable)//设置BO
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
	 
	command &= ~0X2000;
	
	if(enable)
		command |= 0X2000;
	
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );

}
void AD7799_SetBufMode(u8 enable)		//设置buf	
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
	
	command &= ~0X10;
	
	if(enable)
		command |= 0X10;
	
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

void AD7799_SetPolar(u8 enable)		//设置极性	1 单极性 2 双极性
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
	
	command &= ~0X1000;
	
	if(enable)
		command |= 0X1000;
	
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.    
*******************************************************************************/
void AD7799_SetReference(unsigned char state)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_REFDET(1);
    command |= AD7799_CONF_REFDET(state);
    AD7799_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}
void AD7799_Calibrate(void)//通道校准
{
	//4ms一周期
	AD7799_SetRegisterValue(AD7799_REG_CONF,0x2030,2);			//内部通道0校准
	AD7799_SetRegisterValue(AD7799_REG_MODE,0x8003,2);
	HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xA003,2);
	HAL_Delay(5);
	 AD7799_SetRegisterValue(AD7799_REG_MODE,0xC003,2);
	 HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xE003,2);
	HAL_Delay(5);
	
	AD7799_SetRegisterValue(AD7799_REG_CONF,0x2031,2);		//内部通道1校准	
	AD7799_SetRegisterValue(AD7799_REG_MODE,0x8003,2);
	HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xA003,2);
	HAL_Delay(5);
	 AD7799_SetRegisterValue(AD7799_REG_MODE,0xC003,2);
	 HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xE003,2);
	HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_CONF,0x2032,2);		
	//内部通道2校准
	AD7799_SetRegisterValue(AD7799_REG_MODE,0x8003,2);HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xA003,2);HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xC003,2);HAL_Delay(5);
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xE003,2);HAL_Delay(5);
	
	AD7799_SetRegisterValue(AD7799_REG_IO,0,1);//设置通道3 为ad输入
}

/**
 * Convert raw value to voltage
 * @param ad7799
 */
// void AD7799_RawToVolt(AD7799 *ad7799) {
// 	float gain = (float) pow(2.0, (double) ad7799->gain);

// 	if (ad7799->polarity == AD7799_UNIPOLAR) {
// 		float fullscale = (float) pow(2.0, (double) N);
// 		ad7799->voltConversion = (float) ad7799->rawConversion
// 				* (ad7799->vref / (fullscale * gain));
// 	} else {
// 		float fullscale = (float) pow(2.0, (double) (N - 1));
// 		ad7799->voltConversion = (((float) ad7799->rawConversion - fullscale)
// 				* ad7799->vref) / (fullscale * gain);
// 	}
// }

/**
 * Init function for ADC AD7799
 * @param ad7799
 * @return HAL_OK or HAL_ERROR when something wrong
 */
HAL_StatusTypeDef AD7799_Init2(AD7799 *ad7799) {
	uint32_t a=AD7799_GetRegisterValue2(ad7799, AD7799_REG_ID, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
			  HAL_UART_Transmit(&huart1,(uint8_t *)&a,4,1000);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
	if ((a & 0x0F) != AD7799_ID) {
		return HAL_ERROR;
	} else {
		return HAL_OK;
	}
}

/**
 * Reset Function. Clear all configuration
 * @param ad7799
 */
void AD7799_Reset2(AD7799 *ad7799) {
	uint8_t dataToSend[4] = { 0xff, 0xff, 0xff, 0xff };
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(ad7799->adcspi, dataToSend, 4, AD7799_TIMEOUT_SPI);
	SPI_Write2(dataToSend,4,ad7799);
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);


}

/**
 * Get register value from address
 * @param ad7799
 * @param regAddress
 * @param size
 * @return register value in 4 bytes
 */
uint32_t AD7799_GetRegisterValue2(AD7799 *ad7799, uint8_t regAddress,
		uint8_t size) {
	uint8_t data[4] = { 0x00, 0x00, 0x00, 0x00 };
	uint32_t receivedData = 0x00;
	data[0] = AD7799_COMM_READ | AD7799_COMM_ADDR(regAddress);
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(ad7799->adcspi, data, 1, AD7799_TIMEOUT_SPI);
	//HAL_SPI_Receive(ad7799->adcspi, data, size, AD7799_TIMEOUT_SPI);
	SPI_Write2(data,1,ad7799);
	SPI_Read2(data,size,ad7799);
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);
	if (size == 1) {
		receivedData += (data[0] << 0);
	}
	if (size == 2) {
		receivedData += (data[0] << 8);
		receivedData += (data[1] << 0);
	}
	if (size == 3) {
		receivedData += (data[0] << 16);
		receivedData += (data[1] << 8);
		receivedData += (data[2] << 0);
	}
	return receivedData;
}

/**
 * Set value(s) to address register
 * @param ad7799
 * @param regAddress
 * @param regValue
 * @param size
 */
void AD7799_SetRegisterValue2(AD7799 *ad7799, uint8_t regAddress,
		uint32_t regValue, uint8_t size) {
	uint8_t data[4] = { 0x00, 0x00, 0x00, 0x00 };
	data[0] = AD7799_COMM_WRITE | AD7799_COMM_ADDR(regAddress);
	if (size == 1) {
		data[1] = (uint8_t) regValue;
	}
	if (size == 2) {
		data[2] = (uint8_t) ((regValue & 0x0000FF) >> 0);
		data[1] = (uint8_t) ((regValue & 0x00FF00) >> 8);
	}
	if (size == 3) {
		data[3] = (uint8_t) ((regValue & 0x0000FF) >> 0);
		data[2] = (uint8_t) ((regValue & 0x00FF00) >> 8);
		data[1] = (uint8_t) ((regValue & 0xFF0000) >> 16);
	}
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(ad7799->adcspi, data, 1 + size, AD7799_TIMEOUT_SPI);
	SPI_Write2(data,1 + size,ad7799);
	HAL_GPIO_WritePin(ad7799->CSPort, ad7799->CSPin, GPIO_PIN_SET);

}

/**
 * Return 1 when conversion is done
 * @param ad7799
 * @return
 */
uint8_t AD7799_Ready2(AD7799 *ad7799) {
	uint8_t rdy = 0;
	rdy = (AD7799_GetRegisterValue2(ad7799, AD7799_REG_STAT, 1) & 0x80);
	return (!rdy);
}

/**
 * Set the functioning mode
 * @param ad7799
 * @param mode
 */
void AD7799_SetMode2(AD7799 *ad7799, AD7799_Mode mode) {
	uint32_t command;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_MODE, 2);
	command &= ~AD7799_MODE_SEL(0xFF);
	command |= AD7799_MODE_SEL((uint32_t ) mode);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_MODE, command, 2);
	ad7799->mode = mode;
}

/**
 * Set the sample rate
 * @param ad7799
 * @param rate
 */
void AD7799_SetRate2(AD7799 *ad7799, AD7799_Rate rate) {
	uint32_t command;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_MODE, 2);
	command &= ~AD7799_MODE_RATE(0xFF);
	command |= AD7799_MODE_RATE((uint32_t ) rate);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_MODE, command, 2);
	ad7799->rate = rate;
}

/**
 * Set the differential channel
 * @param ad7799
 * @param channel
 */
void AD7799_SetChannel2(AD7799 *ad7799, AD7799_Channel channel) {
	uint32_t command;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_CHAN(0xFF);
	command |= AD7799_CONF_CHAN((uint32_t ) channel);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->channel = channel;
}

/**
 * Set PGA Gain
 * @param ad7799
 * @param gain
 */
void AD7799_SetGain2(AD7799 *ad7799, AD7799_Gain gain) {
	uint32_t command;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_GAIN(0xFF);
	command |= AD7799_CONF_GAIN((uint32_t ) gain);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->gain = gain;
}

/**
 * Set reference detection
 * @param ad7799
 * @param state
 */
void AD7799_SetReference2(AD7799 *ad7799, uint8_t state) {
	uint32_t command = 0;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_REFDET(1);
	command |= AD7799_CONF_REFDET(state);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_CONF, command, 2);
}

/**
 * Set unipolar or bipolar conversion
 * @param ad7799
 * @param polarity
 */
void AD7799_SetPolarity2(AD7799 *ad7799, AD7799_Polarity polarity) {
	uint32_t command = 0;
	command = AD7799_GetRegisterValue2(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_POLAR(1);
	command |= AD7799_CONF_POLAR((uint32_t ) polarity);
	AD7799_SetRegisterValue2(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->polarity = polarity;
}

/**
 * Perform single conversion
 * @param ad7799
 * @return HAL_OK or HAL_TIMEOUT when conversion take too long
 */
HAL_StatusTypeDef AD7799_SingleConversion(AD7799 *ad7799) {
	// AD7799_SetMode(ad7799, AD7799_MODE_SINGLE);

	// uint32_t startTime = HAL_GetTick();
	// while (!AD7799_Ready2(ad7799)) {
	// 	if (HAL_GetTick() - startTime
	// 			> (uint32_t) 1.5 * settle_time_ms[(uint8_t) ad7799->rate]) {
	// 		return HAL_TIMEOUT;
	// 	}
	// }

	// ad7799->rawConversion = AD7799_GetRegisterValue2(ad7799, AD7799_REG_DATA, 3);

	// AD7799_RawToVolt(ad7799);

	return HAL_OK;
}

/**
 * Convert raw value to voltage
 * @param ad7799
 */
void AD7799_RawToVolt(AD7799 *ad7799) {
	// float gain = (float) pow(2.0, (double) ad7799->gain);

	// if (ad7799->polarity == AD7799_UNIPOLAR) {
	// 	float fullscale = (float) pow(2.0, (double) N);
	// 	ad7799->voltConversion = (float) ad7799->rawConversion
	// 			* (ad7799->vref / (fullscale * gain));
	// } else {
	// 	float fullscale = (float) pow(2.0, (double) (N - 1));
	// 	ad7799->voltConversion = (((float) ad7799->rawConversion - fullscale)
	// 			* ad7799->vref) / (fullscale * gain);
	// }
}

