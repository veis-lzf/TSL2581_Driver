#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//IIC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/28
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
#if defined(USE_HARDWARE_I2C)
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
#endif
//IIC初始化
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOH_CLK_ENABLE();   //使能GPIOH时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOB时钟
    

#if defined(USE_HARDWARE_I2C)
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_I2C3_CLK_ENABLE();
	
	/**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
	GPIO_Initure.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_Initure.Mode = GPIO_MODE_AF_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
	
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20404768;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{

	}
	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{

	}
	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{

	}
  
	/**I2C2 GPIO Configuration
    PH4     ------> I2C2_SCL
    PH5     ------> I2C2_SDA
    */
    GPIO_Initure.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_Initure.Mode = GPIO_MODE_AF_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOH, &GPIO_Initure);
	
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x20404768;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{

	}
	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{

	}
	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{

	}
	
    /**I2C3 GPIO Configuration
    PH7     ------> I2C3_SCL
    PH8     ------> I2C3_SDA
    */
	GPIO_Initure.Pin = GPIO_PIN_7|GPIO_PIN_8;
	GPIO_Initure.Mode = GPIO_MODE_AF_OD;
	GPIO_Initure.Pull = GPIO_PULLUP;
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Initure.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(GPIOH, &GPIO_Initure);
	
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x20404768;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{

	}
	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{

	}
	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{

	}
#else
    //PH4,5初始化设置
    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //快速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
    IIC_SDA(1);
    IIC_SCL(1);
#endif
}

#ifndef USE_HARDWARE_I2C
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(5);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	IIC_SCL(1); 
	delay_us(5);			
	IIC_SDA(1);//发送I2C总线结束信号				   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA(1);delay_us(1);	   
	IIC_SCL(1);delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(4);
	IIC_SCL(1);
	delay_us(4);
	IIC_SCL(0);
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(4);
	IIC_SCL(1);
	delay_us(4);
	IIC_SCL(0);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(4);   //对TEA5767这三个延时都是必须的
		IIC_SCL(1);
		delay_us(4); 
		IIC_SCL(0);	
		delay_us(4);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        delay_us(4);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(4); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

u8 IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data)
{
	IIC_Start();
	IIC_Send_Byte(daddr);
	if(IIC_Wait_Ack())
		return 0;

	IIC_Send_Byte(addr);
	if(IIC_Wait_Ack())
		return 0;
	
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack())
		return 0;
	
	IIC_Stop();
	
	return 1;

}
u8 IIC_Read_One_Byte(u8 daddr,u8 addr)
{
	u8 data = 0;
	
	IIC_Start();
	IIC_Send_Byte(daddr);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(daddr | 0x01); // write read address
	IIC_Wait_Ack();
	
	data = IIC_Read_Byte(1); // read+ack
	IIC_Stop();

	return data;
}
#endif
