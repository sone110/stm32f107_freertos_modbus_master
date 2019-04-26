#ifndef  __SHT30_H_
#define  __SHT30_H_
#include "stm32f10x.h"
//#include "delay.h"
//#include "iic.h"
#define  i2cAddWrite_8bit           0x88       //SHT30  ADDR脚接低电平，地址为0x44，左移一位得到写地址
#define  i2cAddRead_8bit            0x89        // 写地址+1得到读地址

/*CRC*/
#define POLYNOMIAL             0x131           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001



#define I2C_SCL_Write_1         GPIOB->BSRR = GPIO_Pin_8 
#define I2C_SCL_Write_0         GPIOB->BRR  = GPIO_Pin_8
    
#define I2C_SDA_Write_1          GPIOB->BSRR = GPIO_Pin_9
#define I2C_SDA_Write_0          GPIOB->BRR  = GPIO_Pin_9

//#define SCL_read      GPIOB->IDR  & GPIO_Pin_6 
#define I2C_SDA_Read()        GPIOB->IDR  & GPIO_Pin_9

//#define SCL_read      GPIOB->IDR  & GPIO_Pin_6 
//  #define I2C_SDA_Read()        GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA)
// #define I2C_SCL GPIO_Pin_8	  //PB10
// #define I2C_SDA GPIO_Pin_9	  //PB11
// #define GPIO_I2C GPIOB

// #define I2C_SCL_H GPIO_SetBits(GPIO_I2C,I2C_SCL)
// #define I2C_SCL_L GPIO_ResetBits(GPIO_I2C,I2C_SCL)

// #define I2C_SDA_H GPIO_SetBits(GPIO_I2C,I2C_SDA)
// #define I2C_SDA_L GPIO_ResetBits(GPIO_I2C,I2C_SDA)
					


//sht30  命令结构体
					typedef enum{
    CMD_READ_SERIALNBR  = 0x3780, // read serial number       
    CMD_READ_STATUS     = 0xF32D, // read status register     
	CMD_CLEAR_STATUS    = 0x3041, // clear status register    ??
	CMD_HEATER_ENABLE   = 0x306D, // enabled heater           
	CMD_HEATER_DISABLE  = 0x3066, // disable heater           
    CMD_SOFT_RESET      = 0x30A2, // soft reset               ??
	CMD_MEAS_CLOCKSTR_H = 0x2C06, // meas. clock stretching, high rep.
	CMD_MEAS_CLOCKSTR_M = 0x2C0D, // meas. clock stretching, medium rep.
	CMD_MEAS_CLOCKSTR_L = 0x2C10, // meas. clock stretching, low rep.
	CMD_MEAS_POLLING_H  = 0x2400, // meas. no clock stretching, high rep.
	CMD_MEAS_POLLING_M  = 0x240B, // meas. no clock stretching, medium rep.
	CMD_MEAS_POLLING_L  = 0x2416, // meas. no clock stretching, low rep.
	CMD_MEAS_PERI_05_H  = 0x2032, // meas. periodic 0.5 mps, high rep.
	CMD_MEAS_PERI_05_M  = 0x2024, // meas. periodic 0.5 mps, medium rep.
	CMD_MEAS_PERI_05_L  = 0x202F, // meas. periodic 0.5 mps, low rep.
	CMD_MEAS_PERI_1_H   = 0x2130, // meas. periodic 1 mps, high rep.
	CMD_MEAS_PERI_1_M   = 0x2126, // meas. periodic 1 mps, medium rep.
	CMD_MEAS_PERI_1_L   = 0x212D, // meas. periodic 1 mps, low rep.
	CMD_MEAS_PERI_2_H   = 0x2236, // meas. periodic 2 mps, high rep.    //
	CMD_MEAS_PERI_2_M   = 0x2220, // meas. periodic 2 mps, medium rep.
	CMD_MEAS_PERI_2_L   = 0x222B, // meas. periodic 2 mps, low rep.
	CMD_MEAS_PERI_4_H   = 0x2334, // meas. periodic 4 mps, high rep.
	CMD_MEAS_PERI_4_M   = 0x2322, // meas. periodic 4 mps, medium rep.
	CMD_MEAS_PERI_4_L   = 0x2329, // meas. periodic 4 mps, low rep.
	CMD_MEAS_PERI_10_H  = 0x2737, // meas. periodic 10 mps, high rep.
	CMD_MEAS_PERI_10_M  = 0x2721, // meas. periodic 10 mps, medium rep.
	CMD_MEAS_PERI_10_L  = 0x272A, // meas. periodic 10 mps, low rep.
	CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
	CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
	CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
	CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
	CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
	CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
	CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
	CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
	CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
    CMD_NO_SLEEP        = 0x303E,
}etCommands;
					
extern float  Tem_Value_1;
extern float  RH_Value_1;
/********************************************************************************/
void i2c_delay(void);
void SHT_Init(void);
void SHT_GetValue(void);
void SHT3X_SetPeriodicMeasurement(void);//20171215
void SHT3X_WriteCMD(uint16_t cmd);
void SHT3X_ReadState(u8 *temp);
void SHX3X_ReadResults(uint16_t cmd,  uint8_t *p);
void GPIO_Config(void);
void SHT_gpio_Init(void) ;

#endif
