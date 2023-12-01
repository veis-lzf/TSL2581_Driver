#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
//#include "key.h"
//#include "lcd.h"
//#include "sdram.h"
//#include "24cxx.h"
#include "myiic.h"
#include "TSL2581.h"
/************************************************
 ALIENTEK 阿波罗STM32F7开发板 实验25
 IIC实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
 

 
int main(void)
{
	u32 lux ;
	u8 ReadBuffer;
	
    Cache_Enable();                 //打开L1-Cache
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
	uart_init(115200);		        //串口初始化
    LED_Init();                     //初始化LED
	IIC_Init();                     //IIC初始化

	I2C_DEV_init();
	ReadBuffer = I2C_DEV_Read(COMMAND_CMD | TRANSACTION | ID);
	printf("Light SensorID:0x%02X\n\r",ReadBuffer & 0xf0);//because The lower four bits are the silicon version number

	while(1)
	{
		lux  =  calculateLux(GAIN_16X, NOM_INTEG_CYCLE);
		printf("lux = %d\n\r",lux);
		delay_ms(500);   
	} 
}
