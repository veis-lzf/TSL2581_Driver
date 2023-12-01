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
 ALIENTEK ������STM32F7������ ʵ��25
 IICʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
 

 
int main(void)
{
	u32 lux ;
	u8 ReadBuffer;
	
    Cache_Enable();                 //��L1-Cache
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
	uart_init(115200);		        //���ڳ�ʼ��
    LED_Init();                     //��ʼ��LED
	IIC_Init();                     //IIC��ʼ��

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
