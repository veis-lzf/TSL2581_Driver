#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "myiic.h"
#include "TSL2581.h"
#include "exti.h"

#define is_POWER_ON		1
#define is_POWER_OFF	0


void isr0(void);

// 中断回调函数，顺序对应到传感器的序号，见g_TSL2581_mux
light_sensor_cb g_interrupt_callback_func_arry[MAX_LIGHT_SENSOR] = 
{
    isr0, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

int main(void)
{
    u8 ReadBuffer, i, count = 0;
    u8 key, bflag;
    
    Cache_Enable();                 //打开L1-Cache
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
    uart_init(115200);		        //串口初始化
    LED_Init();                     //初始化LED
    KEY_Init();                     //初始化按键
    IIC_Init();                     //IIC初始化
    ExInterruptConfig(g_interrupt_callback_func_arry);            // 初始化外部中断
    
    printf("----------------light sensor init----------------\r\n");
    I2C_DEV_PowerOn();              //初始化总线上的TSL2581设备
    for(i = 0; i < MAX_LIGHT_SENSOR; i++)
    {
        ReadBuffer = I2C_DEV_Read(&g_TSL2581_mux[i], COMMAND_CMD | TRANSACTION | ID);
        printf("Light Sensor[%d].ID:0x%02X\r\n",i, ReadBuffer & 0xf0); //打印传感器ID(HIGH 4BIT)
    }
    // 设置中断触发阈值
    for(i = 0; i < MAX_LIGHT_SENSOR; i++)
    {
        SET_Interrupt_Threshold(&g_TSL2581_mux[i], 2000, 50000);
    }
    
    while(1)
    {
        // 按键扫描
        key = KEY_Scan(0);
        
        if(key==KEY0_PRES) // 打开传感器采集
        {
            if(is_POWER_OFF == bflag) // 如果被关电了，那么就重新初始化，否则直接重新读取数据
            {
                printf("key0 press! wait power on.....\r\n");
                bflag = is_POWER_ON;
                I2C_DEV_PowerOn();
                
                // 延时1s，等待上电稳定，由于只有一个业务，故直接使用delay_ms阻塞当前进程
                for(i = 0; i < 100; i++)
                {
                    putchar('#');
                    delay_ms(10);
                }
                printf("100%%\r\n");
            }
            // 读取传感器数据
            for(i = 0; i < MAX_LIGHT_SENSOR; i++)
            {
                calculateLux(&g_TSL2581_mux[i], GAIN_16X, NOM_INTEG_CYCLE);
            }
            printf( "lux[0-8] = %d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
                        g_TSL2581_mux[0].m_value, g_TSL2581_mux[1].m_value, g_TSL2581_mux[2].m_value, 
                        g_TSL2581_mux[3].m_value, g_TSL2581_mux[4].m_value, g_TSL2581_mux[5].m_value, 
                        g_TSL2581_mux[6].m_value, g_TSL2581_mux[7].m_value, g_TSL2581_mux[8].m_value);
        }
        else if(key==KEY1_PRES) // 关闭传感器采集
        {
            printf("key1 press! power off\r\n");
            bflag = is_POWER_OFF;
            I2C_DEV_PowerOff();
        }
        
        delay_ms(10);
        // 心跳运行指示灯
        count++;
        if(count % 50 == 0)
        {
            count = 0;
            LED0_Toggle;
        }
    } 
}

void isr0(void)
{
    calculateLux(&g_TSL2581_mux[0], GAIN_16X, NOM_INTEG_CYCLE);
    printf("light sensor0 value: %d\r\n", g_TSL2581_mux[0].m_value);
    Reload_register(&g_TSL2581_mux[0]);
}
