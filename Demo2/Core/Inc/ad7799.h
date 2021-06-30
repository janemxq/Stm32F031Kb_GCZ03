#ifndef __AD7799_H__
#define __AD7799_H__
#include "stm32f0xx_hal.h"
#include "SPI.h"		// Communication definitions.
//#include "common.h"
/******************************************************************************/
/* AD7799                                                                   */
/******************************************************************************/



#define AD7799_INTERFACE_GPIO		0			//gpio模拟接口
#define AD7799_INTERFACE_SPI1		1			//spi1接口

#define AD7799_INTERFACE_MODE		AD7799_INTERFACE_SPI1	 //当前为spi1接口通信



#define  AD_CS_PIN     GPIO_PIN_3
#define  AD_CS_GPIO    GPIOB

#define AD7799_CS_LOW  AD_CS_0()
#define AD7799_CS_HIGH  AD_CS_1()


/******************通信接口为AD7799_INTERFACE_GPIO模式时,才需要设置下面**********************/
#define  AD_DI_PIN     GPIO_PIN_5
#define  AD_DI_GPIO    GPIOB

#define  AD_SCK_PIN    GPIO_PIN_15
#define  AD_SCK_GPIO   GPIOA

#define  AD_DO_PIN     GPIO_PIN_4
#define  AD_DO_GPIO    GPIOB


#define  AD_CS_0()	 HAL_GPIO_WritePin(AD_CS_GPIO,AD_CS_PIN,GPIO_PIN_RESET)			 
#define  AD_CS_1()	 HAL_GPIO_WritePin(AD_CS_GPIO,AD_CS_PIN,GPIO_PIN_SET)

#define  AD_DI_0()	 HAL_GPIO_WritePin(AD_DI_GPIO,AD_DI_PIN,GPIO_PIN_RESET)
#define  AD_DI_1()	 HAL_GPIO_WritePin(AD_DI_GPIO,AD_DI_PIN,GPIO_PIN_SET) 

#define  AD_SCK_0()	 HAL_GPIO_WritePin(AD_SCK_GPIO,AD_SCK_PIN,GPIO_PIN_RESET)		 
#define  AD_SCK_1()	 HAL_GPIO_WritePin(AD_SCK_GPIO,AD_SCK_PIN,GPIO_PIN_SET) 

#define  AD_DO     (HAL_GPIO_ReadPin(AD_DO_GPIO, AD_DO_PIN))

#define ADC_RDY_DAT (AD_DO)

/******************通信接口为AD7799_INTERFACE_GPIO模式时,才需要设置上面**********************/


/*AD7799 Registers*/
#define AD7799_REG_COMM		0 /* Communications Register(WO, 8-bit) */
#define AD7799_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
#define AD7799_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
#define AD7799_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
#define AD7799_REG_DATA	    3 /* Data Register	     	(RO, 16-/24-bit) */
#define AD7799_REG_ID	    4 /* ID Register	     	(RO, 8-bit) */
#define AD7799_REG_IO	    5 /* IO Register	     	(RO, 8-bit) */
#define AD7799_REG_OFFSET   6 /* Offset Register	    (RW, 24-bit */
#define AD7799_REG_FULLSALE	7 /* Full-Scale Register	(RW, 24-bit */

/* Communications Register Bit Designations (AD7799_REG_COMM) */
#define AD7799_COMM_WEN		(1 << 7) 			/* Write Enable */
#define AD7799_COMM_WRITE	(0 << 6) 			/* Write Operation */
#define AD7799_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7799_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7799_COMM_CREAD	(1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designations (AD7799_REG_STAT) */
#define AD7799_STAT_RDY		(1 << 7) /* Ready */
#define AD7799_STAT_ERR		(1 << 6) /* Error (Overrange, Underrange) */
#define AD7799_STAT_CH3		(1 << 2) /* Channel 3 */
#define AD7799_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7799_STAT_CH1		(1 << 0) /* Channel 1 */

/* Mode Register Bit Designations (AD7799_REG_MODE) */
#define AD7799_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7799_MODE_PSW(x)		(1 << 12)			/* Power Switch Control Bit */	
#define AD7799_MODE_RATE(x)		((x) & 0xF) 		/* Filter Update Rate Select */

/* AD7799_MODE_SEL(x) options */
// #define AD7799_MODE_CONT		 0 /* Continuous Conversion Mode */
// #define AD7799_MODE_SINGLE		 1 /* Single Conversion Mode */
// #define AD7799_MODE_IDLE		 2 /* Idle Mode */
// #define AD7799_MODE_PWRDN		 3 /* Power-Down Mode */
// #define AD7799_MODE_CAL_INT_ZERO 4 /* Internal Zero-Scale Calibration */
// #define AD7799_MODE_CAL_INT_FULL 5 /* Internal Full-Scale Calibration */
// #define AD7799_MODE_CAL_SYS_ZERO 6 /* System Zero-Scale Calibration */
// #define AD7799_MODE_CAL_SYS_FULL 7 /* System Full-Scale Calibration */

/* Configuration Register Bit Designations (AD7799_REG_CONF) */
#define AD7799_CONF_BO_EN	  (1 << 13) 			/* Burnout Current Enable */
#define AD7799_CONF_UNIPOLAR  (1 << 12) 			/* Unipolar/Bipolar Enable */
#define AD7799_CONF_GAIN(x)	  (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7799_CONF_REFDET(x) (((x) & 0x1) << 5) 	/* Reference detect function */
#define AD7799_CONF_BUF		  (1 << 4) 				/* Buffered Mode Enable */
#define AD7799_CONF_CHAN(x)	  ((x) & 0x7) 			/* Channel select */
#define AD7799_CONF_POLAR(x)  (((x) & 0x1) << 12)
/* AD7799_CONF_GAIN(x) options */
// #define AD7799_GAIN_1       0
// #define AD7799_GAIN_2       1
// #define AD7799_GAIN_4       2
// #define AD7799_GAIN_8       3
// #define AD7799_GAIN_16      4
// #define AD7799_GAIN_32      5
// #define AD7799_GAIN_64      6
// #define AD7799_GAIN_128     7

/* AD7799_CONF_REFDET(x) options */
#define AD7799_REFDET_ENA   1	
#define AD7799_REFDET_DIS   0

/* AD7799_CONF_CHAN(x) options */
// #define AD7799_CH_AIN1P_AIN1M	0 /* AIN1(+) - AIN1(-) */
// #define AD7799_CH_AIN2P_AIN2M	1 /* AIN2(+) - AIN2(-) */
// #define AD7799_CH_AIN3P_AIN3M	2 /* AIN3(+) - AIN3(-) */
// #define AD7799_CH_AIN1M_AIN1M	3 /* AIN1(-) - AIN1(-) */
// #define AD7799_CH_AVDD_MONITOR	7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7799_REG_ID) */
#define AD7799_ID			0x9
#define AD7799_ID_MASK		0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7799_REG_IO) */
#define AD7799_IOEN			(1 << 6)
#define AD7799_IO1(x)		(((x) & 0x1) << 4)
#define AD7799_IO2(x)		(((x) & 0x1) << 5)
/**
 * Enum to set sample rate
 */
typedef enum {
	AD7799_RATE_470HZ = 1, /**< AD7799_RATE_470HZ */
	AD7799_RATE_242HZ, /**< AD7799_RATE_242HZ */
	AD7799_RATE_123HZ, /**< AD7799_RATE_123HZ */
	AD7799_RATE_62HZ, /**< AD7799_RATE_62HZ */
	AD7799_RATE_50HZ, /**< AD7799_RATE_50HZ */
	AD7799_RATE_39HZ, /**< AD7799_RATE_39HZ */
	AD7799_RATE_33_2HZ, /**< AD7799_RATE_33_2HZ */
	AD7799_RATE_19_6HZ_90DB,/**< AD7799_RATE_19_6HZ_90DB */
	AD7799_RATE_16_7HZ_80DB,/**< AD7799_RATE_16_7HZ_80DB */
	AD7799_RATE_16_7HZ_65DB,/**< AD7799_RATE_16_7HZ_65DB */
	AD7799_RATE_12_5HZ_66DB,/**< AD7799_RATE_12_5HZ_66DB */
	AD7799_RATE_10HZ_69DB, /**< AD7799_RATE_10HZ_69DB */
	AD7799_RATE_8_33HZ_70DB,/**< AD7799_RATE_8_33HZ_70DB */
	AD7799_RATE_6_25HZ_72DB,/**< AD7799_RATE_6_25HZ_72DB */
	AD7799_RATE_4_17HZ_74DB /**< AD7799_RATE_4_17HZ_74DB */
} AD7799_Rate;

static const uint16_t settle_time_ms[] = { 0, 4, 8, 16, 32, 40, 48, 60, 101,
		120, 120, 160, 200, 240, 320, 480 };

/**
 * Enum to set functioning mode
 */
typedef enum {
	AD7799_MODE_CONT = 0, /**< AD7799_MODE_CONT */
	AD7799_MODE_SINGLE, /**< AD7799_MODE_SINGLE */
	AD7799_MODE_IDLE, /**< AD7799_MODE_IDLE */
	AD7799_MODE_PWRDN, /**< AD7799_MODE_PWRDN */
	AD7799_MODE_CAL_INT_ZERO,/**< AD7799_MODE_CAL_INT_ZERO */
	AD7799_MODE_CAL_INT_FULL,/**< AD7799_MODE_CAL_INT_FULL */
	AD7799_MODE_CAL_SYS_ZERO,/**< AD7799_MODE_CAL_SYS_ZERO */
	AD7799_MODE_CAL_SYS_FULL /**< AD7799_MODE_CAL_SYS_FULL */
} AD7799_Mode;

/**
 * Enum to set channel
 */
typedef enum {
	AD7799_CH_AIN1P_AIN1M = 0,/**< AD7799_CH_AIN1P_AIN1M */
	AD7799_CH_AIN2P_AIN2M, /**< AD7799_CH_AIN2P_AIN2M */
	AD7799_CH_AIN3P_AIN3M, /**< AD7799_CH_AIN3P_AIN3M */
	AD7799_CH_AIN1M_AIN1M, /**< AD7799_CH_AIN1M_AIN1M */
	AD7799_CH_AVDD_MONITOR = 7/**< AD7799_CH_AVDD_MONITOR */
} AD7799_Channel;

/**
 * Enum to set PGA Gain
 */
typedef enum {
	AD7799_GAIN_1 = 0,/**< AD7799_GAIN_1 */
	AD7799_GAIN_2, /**< AD7799_GAIN_2 */
	AD7799_GAIN_4, /**< AD7799_GAIN_4 */
	AD7799_GAIN_8, /**< AD7799_GAIN_8 */
	AD7799_GAIN_16, /**< AD7799_GAIN_16 */
	AD7799_GAIN_32, /**< AD7799_GAIN_32 */
	AD7799_GAIN_64, /**< AD7799_GAIN_64 */
	AD7799_GAIN_128 /**< AD7799_GAIN_128 */
} AD7799_Gain;

/**
 * enum to set polarity
 */
typedef enum {
	AD7799_BIPOLAR = 0,/**< AD7799_BIPOLAR */
	AD7799_UNIPOLAR /**< AD7799_UNIPOLAR */
} AD7799_Polarity;

/**
 * Struct to easily use configuration and values from conversions
 */
typedef struct {
	//SPI_HandleTypeDef *adcspi; /**< SPI handler */
	GPIO_TypeDef *CSPort; /**< CS GPIO Port */
	uint16_t CSPin; /**< CS GPIO Pin */

    GPIO_TypeDef *DOPort; /**< DO GPIO Port */
	uint16_t DOPin; /**< DO GPIO Pin */

	GPIO_TypeDef *DIPort; /**< DI GPIO Port */
	uint16_t DIPin; /**< DI GPIO Pin */

    GPIO_TypeDef *SCKPort; /**< SCK GPIO Port */
	uint16_t SCKPin; /**< SCK GPIO Pin */

	float vref; /**< Voltage reference */

	AD7799_Rate rate; /**< Selected sample rate */
	AD7799_Mode mode; /**< Selected mode */
	AD7799_Channel channel; /**< Selected channel */
	AD7799_Gain gain; /**< Selected gain */
	AD7799_Polarity polarity; /**< Selected polarity */

	uint32_t rawConversion; /**< Value of raw conversion */
	float voltConversion; /**< Value of voltage */
} AD7799;
HAL_StatusTypeDef AD7799_Init2(AD7799 *ad7799);

void AD7799_Reset2(AD7799 *ad7799);

uint32_t AD7799_GetRegisterValue2(AD7799 *ad7799, uint8_t regAddress,
		uint8_t size);

void AD7799_SetRegisterValue2(AD7799 *ad7799, uint8_t regAddress,
		uint32_t regValue, uint8_t size);

void AD7799_SetRate2(AD7799 *ad7799, AD7799_Rate rate);

void AD7799_SetMode2(AD7799 *ad7799, AD7799_Mode mode);

uint8_t AD7799_Ready2(AD7799 *ad7799);

void AD7799_SetChannel2(AD7799 *ad7799, AD7799_Channel channel);

void AD7799_SetGain2(AD7799 *ad7799, AD7799_Gain gain);

void AD7799_SetReference2(AD7799 *ad7799, uint8_t state);

void AD7799_SetPolarity2(AD7799 *ad7799, AD7799_Polarity polarity);

HAL_StatusTypeDef AD7799_SingleConversion(AD7799 *ad7799);

void AD7799_RawToVolt(AD7799 *ad7799);

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
void AD7799_gpio_init(void);
void spi_AD7799_init(void);
/* Initialize AD7799 and check if the device is present*/
unsigned char AD7799_Init(void);
/* Sends 32 consecutive 1's on SPI in order to reset the part. */
void AD7799_Reset(void);
/* Reads the value of the selected register. */
unsigned long AD7799_GetRegisterValue(unsigned char regAddress, 
									  unsigned char size);
/* Writes a value to the register. */
void AD7799_SetRegisterValue(unsigned char regAddress,
							 unsigned long regValue, 
							 unsigned char size);
/* Sets the operating mode of AD7799. */
void AD7799_SetMode(unsigned long mode,u8  rate);
/* Reads /RDY bit of Status register. */
unsigned char AD7799_Ready(void);              
/* Selects the channel of AD7799. */
void AD7799_SetChannel(unsigned long channel);
/* Sets the gain of the In-Amp. */
void AD7799_SetGain(unsigned long gain);
/* Enables or disables the reference detect function. */
void AD7799_SetReference(unsigned char state);

void AD7799_Calibrate(void);//通道校准

void AD7799_SetBurnoutCurren(u8 enable); //设置BO
							 
void AD7799_SetBufMode(u8 enable);		//设置buf							 
							 
							 
//void SPI_Read(u8 * buffer,u8 size);
//void SPI_Write(u8 * buffer,u8 size);							 
							 
#endif	// _AD7799_H_
