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

// �жϻص�������˳���Ӧ������������ţ���g_TSL2581_mux
light_sensor_cb g_interrupt_callback_func_arry[MAX_LIGHT_SENSOR] = 
{
    isr0, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

int main(void)
{
    u8 ReadBuffer, i, count = 0;
    u8 key, bflag;
    
    Cache_Enable();                 //��L1-Cache
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
    uart_init(115200);		        //���ڳ�ʼ��
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������
    IIC_Init();                     //IIC��ʼ��
    ExInterruptConfig(g_interrupt_callback_func_arry);            // ��ʼ���ⲿ�ж�
    
    printf("----------------light sensor init----------------\r\n");
    I2C_DEV_PowerOn();              //��ʼ�������ϵ�TSL2581�豸
    for(i = 0; i < MAX_LIGHT_SENSOR; i++)
    {
        ReadBuffer = I2C_DEV_Read(&g_TSL2581_mux[i], COMMAND_CMD | TRANSACTION | ID);
        printf("Light Sensor[%d].ID:0x%02X\r\n",i, ReadBuffer & 0xf0); //��ӡ������ID(HIGH 4BIT)
    }
    // �����жϴ�����ֵ
    for(i = 0; i < MAX_LIGHT_SENSOR; i++)
    {
        SET_Interrupt_Threshold(&g_TSL2581_mux[i], 2000, 50000);
    }
    
    while(1)
    {
        // ����ɨ��
        key = KEY_Scan(0);
        
        if(key==KEY0_PRES) // �򿪴������ɼ�
        {
            if(is_POWER_OFF == bflag) // ������ص��ˣ���ô�����³�ʼ��������ֱ�����¶�ȡ����
            {
                printf("key0 press! wait power on.....\r\n");
                bflag = is_POWER_ON;
                I2C_DEV_PowerOn();
                
                // ��ʱ1s���ȴ��ϵ��ȶ�������ֻ��һ��ҵ�񣬹�ֱ��ʹ��delay_ms������ǰ����
                for(i = 0; i < 100; i++)
                {
                    putchar('#');
                    delay_ms(10);
                }
                printf("100%%\r\n");
            }
            // ��ȡ����������
            for(i = 0; i < MAX_LIGHT_SENSOR; i++)
            {
                calculateLux(&g_TSL2581_mux[i], GAIN_16X, NOM_INTEG_CYCLE);
            }
            printf( "lux[0-8] = %d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
                        g_TSL2581_mux[0].m_value, g_TSL2581_mux[1].m_value, g_TSL2581_mux[2].m_value, 
                        g_TSL2581_mux[3].m_value, g_TSL2581_mux[4].m_value, g_TSL2581_mux[5].m_value, 
                        g_TSL2581_mux[6].m_value, g_TSL2581_mux[7].m_value, g_TSL2581_mux[8].m_value);
        }
        else if(key==KEY1_PRES) // �رմ������ɼ�
        {
            printf("key1 press! power off\r\n");
            bflag = is_POWER_OFF;
            I2C_DEV_PowerOff();
        }
        
        delay_ms(10);
        // ��������ָʾ��
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
