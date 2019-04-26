#include "sht30.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "bsp_timer.h"
#define  i2cAddWrite_8bit           0x88       //SHT30  ADDR脚接低电平，地址为0x44，左移一位得到写地址
#define  i2cAddRead_8bit            0x89        // 写地址+1得到读地址


/*CRC*/
#define POLYNOMIAL             0x131           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001



/****************************************************************/

extern float  Tem_Value_1;
extern float  RH_Value_1;
uint8_t buffer[6];

void SDA_H(void)
{
    I2C_SDA_Write_1;
}
void SDA_L(void)
{
    I2C_SDA_Write_0;
}
uint16_t SDA_Read(void)   //注意要定义为u16类型，因为Read返回类型是uint16类型的
{
    return I2C_SDA_Read();
}
void SCL_H(void)
{
    I2C_SCL_Write_1;
}
void SCL_L(void)
{
    I2C_SCL_Write_0;
}



void i2c_delay(void)
{
	//vTaskDelay(20);
   //bsp_DelayUS(20);
	 //Delay_us(20);
	delay_us(20);
}
uint8_t i2c_star(void)
{
    SDA_H();
    SCL_H();
    i2c_delay();
//     if (!SDA_Read())  //读取SDA线状态，如果已经被拉低，则起始信号发送 ，否则手动置低
//         return 1;
    SDA_L();
    i2c_delay();
//     if (SDA_Read())
//         return 1;
    SDA_L();
    SCL_L();
    i2c_delay();
    return 0;
}

void i2c_stop(void)
{
    SCL_L();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_H();
    i2c_delay();
}
void i2c_ack(void)
{
    SCL_L();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    i2c_delay();
}

void i2c_noAck(void)
{
    SCL_L();
    i2c_delay();
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    i2c_delay();
}
uint8_t i2c_waitAck(void)
{
    uint8_t t = 100;
    
    SCL_L(); 
    i2c_delay();
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();    
    while( SDA_Read() )
    {
        t --;
        if(t==0)
        {
           SCL_L();
            return 1;
        }
        i2c_delay();
    }

    SCL_L();
    i2c_delay();
    return 0;
	
}


// 发送一个字节
void i2c_sendByte( uint8_t byte )
{
    uint8_t i = 8;
    while (i--) {
        SCL_L();
        i2c_delay();
        if (byte & 0x80)
            SDA_H();
        else
            SDA_L();
        byte <<= 1;
        i2c_delay();
        SCL_H();
        i2c_delay();
    }
    SCL_L();
    i2c_delay();
}

//读取一个字节
uint8_t i2c_readByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H();
    while (i--) 
    {
        byte <<= 1;
        SCL_L();
        i2c_delay();
        SCL_H();
        i2c_delay();
        if (SDA_Read()) 
        {
            byte |= 0x01;
        }
    }
    SCL_L();
    i2c_delay();
    return byte;
}

// sht30 写命令模式

void SHT3X_WriteCMD(uint16_t cmd)
{
	i2c_star();
	i2c_sendByte(i2cAddWrite_8bit);
	i2c_waitAck();     //等待应答信号
	i2c_sendByte(cmd>>8);//先发高8位
	i2c_waitAck();
	i2c_sendByte(cmd); //发低八位
	i2c_waitAck();
	i2c_stop();
}

/************************ meas. periodic 2 mps, high rep.********************************/
void SHT3X_SetPeriodicMeasurement(void)
{
    
	SHT3X_WriteCMD(CMD_MEAS_PERI_2_H);
}
void SHT3X_ReadState(uint8_t *temp)/******??????****/
{
    i2c_star();
	i2c_sendByte(i2cAddWrite_8bit);
	i2c_waitAck();
	i2c_sendByte(0xf3);
	i2c_waitAck();
	i2c_sendByte(0X2d);
	i2c_waitAck();
    
    i2c_star();
	i2c_sendByte(i2cAddRead_8bit);
	i2c_waitAck();

	temp[0] = i2c_readByte();
	i2c_ack();
	temp[1] = i2c_readByte();
	i2c_ack();
	temp[2] = i2c_readByte();
	i2c_noAck();
    
    i2c_stop(); 
}
//读取结果
void SHX3X_ReadResults(uint16_t cmd,  uint8_t *p)
{
	i2c_star();
	i2c_sendByte(i2cAddWrite_8bit);
	i2c_waitAck();
	i2c_sendByte(cmd>>8);
	i2c_waitAck();
	i2c_sendByte(cmd);
	i2c_waitAck();

    i2c_star();
	i2c_sendByte(i2cAddRead_8bit);
	i2c_waitAck();

	p[0] = i2c_readByte();
	i2c_ack();
	p[1] = i2c_readByte();
	i2c_ack();
	p[2] = i2c_readByte();
	i2c_ack();
	p[3] = i2c_readByte();
	i2c_ack();
	p[4] = i2c_readByte();
	i2c_ack();
	p[5] = i2c_readByte();
	i2c_noAck();
	i2c_stop();
}

/********************************************************************************/
/********************************************************************************/
void SHT_Init(void)
{   
	  
	  SHT_gpio_Init();
    //Delay_ms(105);      /* Must add delay */
	 
	  delay_ms(15);
    // 设置模式
    SHT3X_SetPeriodicMeasurement();
    //Delay_ms(1800); /* Must add delay */
	  
	  delay_ms(180);
}
 //生成校验码
uint8_t SHT3X_CalcCrc(uint8_t *data, uint8_t nbrOfBytes)
{
	  uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}
//检查校验码
uint8_t SHT3X_CheckCrc(uint8_t *pdata, uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc;
	crc = SHT3X_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}
//温度计算
float SHT3X_CalcTemperature(uint16_t rawValue)
{
    // calculate temperature 
    float temp1;
    temp1 = (175 *(float)rawValue / 65535 - 45) ; // T = -45 + 175 * rawValue / (2^16-1)
    return temp1;
}
//湿度
float SHT3X_CalcRH(uint16_t rawValue)
{
    // calculate relative humidity [%RH]
    float temp2 = (100 * (float)rawValue / 65535) ;  // RH = rawValue / (2^16-1) * 10

    return temp2;
}
void SHT_GetValue(void)
{
    uint8_t temp = 0;
    uint16_t dat;
    uint8_t p[3];
    // 读取结果
	
    SHX3X_ReadResults(CMD_FETCH_DATA, buffer);
    /* check tem */
    p[0] = buffer[0];
    p[1] = buffer[1];
    p[2] = buffer[2];
	//检验校验值
    temp = SHT3X_CheckCrc(p,2,p[2]);
   if( !temp ) /* value is ture */ 
    {
        dat = ((uint16_t)buffer[0] << 8) | buffer[1];
        Tem_Value_1 = SHT3X_CalcTemperature( dat );    
    }
    /* check humidity */
    p[0] = buffer[3];
    p[1] = buffer[4];
    p[2] = buffer[5];
    temp = SHT3X_CheckCrc(p,2,p[2]);
    if( !temp )
    {
        dat = ((uint16_t)p[0] << 8) | p[1];
        RH_Value_1 = SHT3X_CalcRH( dat ); 
    }
}


void SHT_gpio_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStrcture ;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
/*	
	GPIO_InitStrcture.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStrcture.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStrcture.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOB , &GPIO_InitStrcture);
*/	
	GPIO_InitStrcture.GPIO_Pin = GPIO_Pin_8 ;	
	GPIO_InitStrcture.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStrcture.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStrcture);		  
	
  GPIO_InitStrcture.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStrcture.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStrcture.GPIO_Mode=GPIO_Mode_Out_OD;  
  GPIO_Init(GPIOB, &GPIO_InitStrcture); 
	
	GPIO_ResetBits(GPIOB , GPIO_Pin_8 | GPIO_Pin_9);


}
