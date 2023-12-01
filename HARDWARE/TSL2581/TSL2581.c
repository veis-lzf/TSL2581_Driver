#include "TSL2581.h"
#include "myiic.h"
#include "usart.h"
#include "delay.h"
#include "stm32f7xx_hal.h"

uint16_t Channel_0, Channel_1;

#ifdef USE_HARDWARE_I2C
sTSL2581_t g_TSL2581_mux[MAX_LIGHT_SENSOR] = 
{
	// 第一组
	{
		.m_i2c_handle = &hi2c1,
		.m_address = ADDR_LOW, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第二组
	{
		.m_i2c_handle = &hi2c1, 
		.m_address = ADDR_FLOAT, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第三组
	{
		.m_i2c_handle = &hi2c1, 
		.m_address = ADDR_HIGH, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第四组
	{
		.m_i2c_handle = &hi2c2, 
		.m_address = ADDR_LOW, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第五组
	{
		.m_i2c_handle = &hi2c2, 
		.m_address = ADDR_FLOAT, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第六组
	{
		.m_i2c_handle = &hi2c2, 
		.m_address = ADDR_HIGH, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第七组
	{
		.m_i2c_handle = &hi2c3, 
		.m_address = ADDR_LOW, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第八组
	{
		.m_i2c_handle = &hi2c3, 
		.m_address = ADDR_FLOAT, 
		.m_value = 0,
		.interrupt_cb = NULL
	},
	// 第九组
	{
		.m_i2c_handle = &hi2c3, 
		.m_address = ADDR_HIGH,
		.m_value = 0,
		.interrupt_cb = NULL
	},
};

#endif

/**********************************************************************************************
* @brief  I2C_DEV_Write(uint16_t Register_Addr,uint8_t Register_Data)
* @param  Register_Addr : Register Address	
* @param  Register_Data : Register data
* @param  return Write state,Write success is 1,Write fail is 0
**********************************************************************************************/
#ifdef USE_HARDWARE_I2C
uint8_t I2C_DEV_Write(sTSL2581_t *obj, uint16_t Register_Addr,uint8_t Register_Data)
#else
uint8_t I2C_DEV_Write(uint16_t I2C_Addr, uint16_t Register_Addr,uint8_t Register_Data)
#endif
{
#ifdef USE_HARDWARE_I2C
	
	if(obj == NULL)
		return Write_fail;
		
	if(HAL_I2C_Mem_Write(obj->m_i2c_handle, obj->m_address, Register_Addr, I2C_MEMADD_SIZE_8BIT, &Register_Data, 1, 1000) == HAL_OK)
	{
		return Write_success;
	} else{
		return Write_fail;
	}
#else
	uint8_t ret = IIC_Write_One_Byte(I2C_Addr, Register_Addr, Register_Data);
	if(ret)
		return Write_success;
	else
		return Write_fail;
#endif
}

/**********************************************************************************************
* @brief  I2C_DEV_Read(uint16_t Register_Addr)
* @param  Register_Addr : Register Address	
* @param  
* @param  return read data (uchar)
**********************************************************************************************/
#ifdef USE_HARDWARE_I2C
uint8_t I2C_DEV_Read(sTSL2581_t *obj, uint16_t Register_Addr)
#else
uint8_t I2C_DEV_Read(uint16_t Register_Addr)
#endif
{
#ifdef USE_HARDWARE_I2C
	uint8_t ReadBuffer[1];
	
	if(obj == NULL)
		return 0xff;
	
	HAL_I2C_Mem_Read(obj->m_i2c_handle, (obj->m_address | 0x01), Register_Addr, I2C_MEMADD_SIZE_8BIT,ReadBuffer,1, 1000);
	return ReadBuffer[0];
#else
	uint8_t data = IIC_Read_One_Byte(ADDR_DEVICE, Register_Addr);
	return data;
#endif
}
 
/**********************************************************************************************
* @brief  	I2C_DEV_PowerOn()
* @param   	power on ,set gain is 16,interrupt is 402ms
* @param   
* @param    This field selects the integration time for each conversion.
**********************************************************************************************/
void I2C_DEV_PowerOn(void)
{ 
#ifdef USE_HARDWARE_I2C
	uint8_t i;
	
	for(i = 0; i < sizeof(g_TSL2581_mux)/sizeof(sTSL2581_t); i++)
	{
		/* write date from tsl2561 */
		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | CONTROL,CONTROL_POWERON);//power on
		//delay_ms(2000);//Wait 2 seconds for power on

		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | TIMING, INTEGRATIONTIME_400MS);  //400MS
		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | CONTROL, ADC_EN | CONTROL_POWERON); //Every ADC cycle generates interrupt
		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | INTERRUPT, INTR_INTER_MODE);	//TEST MODE
		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | ANALOG, GAIN_16X);				//GAIN = 16
	}
#else
	/* write date from tsl2561 */
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | CONTROL,CONTROL_POWERON);//power on
	delay_ms(2000);//Wait 2 seconds for power on
	
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | TIMING, INTEGRATIONTIME_400MS);  //400MS
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | CONTROL, ADC_EN | CONTROL_POWERON); //Every ADC cycle generates interrupt
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | INTERRUPT, INTR_INTER_MODE);	//TEST MODE
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | ANALOG, GAIN_16X);				//GAIN = 16
#endif
}
/**********************************************************************************************
* @brief  	I2C_DEV_PowerOff()
* @param   	power on ,set gain is 16,interrupt is 402ms
* @param   
* @param    This field selects the integration time for each conversion.
**********************************************************************************************/
void I2C_DEV_PowerOff(void)
{ 
#ifdef USE_HARDWARE_I2C
	uint8_t i;
	
	for(i = 0; i < sizeof(g_TSL2581_mux)/sizeof(sTSL2581_t); i++)
	{
		I2C_DEV_Write(&g_TSL2581_mux[i], COMMAND_CMD | CONTROL, CONTROL_POWEROFF);//power off
	}
#else
	/* write date from tsl2561 */
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | CONTROL, CONTROL_POWEROFF);//power on
	delay_ms(2000);//Wait 2 seconds for power on
#endif
}

/**********************************************************************************************
* @brief  	Reload_register()
* @param   Interrupts need to be maintained for several cycles
* @param   When the interrupt bit is 0, reload the register
* @param   Configure the special registers, clear the interrupt bits, and then re-enable the ADC
***********************************************************************************************/
#ifdef USE_HARDWARE_I2C
void Reload_register(sTSL2581_t *obj)
#else
void Reload_register(void)
#endif
{
#ifdef USE_HARDWARE_I2C
	I2C_DEV_Write(obj, COMMAND_CMD | TRANSACTION_SPECIAL | SPECIAL_FUN_INTCLEAR, INTR_INTER_MODE);
	I2C_DEV_Write(obj, COMMAND_CMD | TRANSACTION | CONTROL, ADC_EN | CONTROL_POWERON); //Every ADC cycle generates interrupt
#else
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | TRANSACTION_SPECIAL | SPECIAL_FUN_INTCLEAR, INTR_INTER_MODE);
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | TRANSACTION | CONTROL, ADC_EN | CONTROL_POWERON); //Every ADC cycle generates interrupt
#endif
}

/**********************************************************************************************
* @brief  	SET_Interrupt_Threshold(uint32_t low,uint32_t high)
* @param   	low and high max 2^16 = 65536
* @param   
* @param    This field selects the integration time for each conversion.
**********************************************************************************************/
#ifdef USE_HARDWARE_I2C
void SET_Interrupt_Threshold(sTSL2581_t *obj, uint16_t min,uint16_t max)
#else
void SET_Interrupt_Threshold(uint16_t min,uint16_t max)
#endif
{
	uint8_t DataLLow,DataLHigh,DataHLow,DataHHigh;
#ifdef USE_HARDWARE_I2C
	DataLLow = min % 256;
	DataLHigh = min / 256;
	I2C_DEV_Write(obj, COMMAND_CMD | THLLOW, DataLLow);  
	I2C_DEV_Write(obj, COMMAND_CMD | THLHIGH, DataLHigh);  

	DataHLow = max % 256;
	DataHHigh = max / 256;
	I2C_DEV_Write(obj, COMMAND_CMD | THHLOW, DataHLow);  
	I2C_DEV_Write(obj, COMMAND_CMD | THHHIGH, DataHHigh);
#else
	DataLLow = min % 256;
	DataLHigh = min / 256;
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | THLLOW, DataLLow);  
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | THLHIGH, DataLHigh);  

	DataHLow = max % 256;
	DataHHigh = max / 256;
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | THHLOW, DataHLow);  
	I2C_DEV_Write(ADDR_DEVICE,COMMAND_CMD | THHHIGH, DataHHigh);
#endif
}	

/**********************************************************************************************
* @brief  	Read_Channel()
* @param    
* @param   	read two ADC data
* @param     
**********************************************************************************************/
#ifdef USE_HARDWARE_I2C
void Read_Channel(sTSL2581_t *obj)
#else
void Read_Channel(void)
#endif
{	
	uint8_t DataLow,DataHigh;
#ifdef USE_HARDWARE_I2C
	DataLow = I2C_DEV_Read(obj, COMMAND_CMD | TRANSACTION | DATA0LOW);
	DataHigh = I2C_DEV_Read(obj, COMMAND_CMD | TRANSACTION | DATA0HIGH);
	Channel_0 = 256 * DataHigh + DataLow ;
	
	DataLow = I2C_DEV_Read(obj, COMMAND_CMD | TRANSACTION | DATA1LOW);
	DataHigh = I2C_DEV_Read(obj, COMMAND_CMD | TRANSACTION | DATA1HIGH);
	Channel_1 = 256 * DataHigh + DataLow ;
#else
	DataLow = I2C_DEV_Read(COMMAND_CMD | TRANSACTION | DATA0LOW);
	DataHigh = I2C_DEV_Read(COMMAND_CMD | TRANSACTION | DATA0HIGH);
	Channel_0 = 256 * DataHigh + DataLow ;
	
	DataLow = I2C_DEV_Read(COMMAND_CMD | TRANSACTION | DATA1LOW);
	DataHigh = I2C_DEV_Read(COMMAND_CMD | TRANSACTION | DATA1HIGH);
	Channel_1 = 256 * DataHigh + DataLow ;
#endif
	
#if defined(DEBUG_TRACE)
	printf("Channel_0 = %d\n",Channel_0);
	printf("Channel_1 = %d\n",Channel_1);
#endif
}

/**********************************************************************************************
* @brief  	calculateLux()
* @param    Channel_0 and Channel_1 is for TSL2561_Read_Channel();
* @param   	// Arguments: unsigned int iGain - gain, where 0:1X, 1:8X, 2:16X, 3:128X
* @param   	// unsigned int tIntCycles - INTEG_CYCLES defined in Timing Register
**********************************************************************************************/
#ifdef USE_HARDWARE_I2C
uint32_t calculateLux(sTSL2581_t *obj, uint16_t iGain,uint16_t tIntCycles)
#else
uint32_t calculateLux(uint16_t iGain, uint16_t tIntCycles)
#endif
{
	unsigned long chScale0;
	unsigned long chScale1;
	unsigned long channel1;
	unsigned long channel0;
	unsigned long temp;
	unsigned long ratio1 = 0;
	unsigned long ratio;
	unsigned long lux_temp;
	unsigned int b, m;

	// No scaling if nominal integration (148 cycles or 400 ms) is used
	if (tIntCycles == NOM_INTEG_CYCLE)
	{
	//     chScale0 = 65536;
		chScale0 = (1 << (CH_SCALE));
	}
	else
		chScale0 = (NOM_INTEG_CYCLE << CH_SCALE) / tIntCycles;
	
	switch (iGain)
	{
	case 0: // 1x gain
		chScale1 = chScale0; // No scale. Nominal setting
		break;
	case 1: // 8x gain
		chScale0 = chScale0 >> 3; // Scale/multiply value by 1/8
		chScale1 = chScale0;
		break;
	case 2: // 16x gain
		chScale0 = chScale0 >> 4; // Scale/multiply value by 1/16
		chScale1 = chScale0;
		break;
	case 3: // 128x gain
		chScale1 = chScale0 / CH1GAIN128X;
		chScale0 = chScale0 / CH0GAIN128X;
		break;
	}
  // Read Channel for ADC
#ifdef USE_HARDWARE_I2C
  Read_Channel(obj);
#else
  Read_Channel();
#endif
	// scale the channel values
	channel0 = (Channel_0 * chScale0) >>  CH_SCALE;
	channel1 = (Channel_1 * chScale1) >>  CH_SCALE;

	// find the ratio of the channel values (Channel1/Channel0)
	if (channel0 != 0)
	ratio1 = (channel1 << (RATIO_SCALE + 1)) / channel0;
	ratio = (ratio1 + 1) >> 1;	  									 // round the ratio value

	if ((ratio >= 0X00) && (ratio <= K1C))
	{   
		b = B1C;
		m = M1C;  
	}
	else if (ratio <= K2C)
	{    
		b = B2C;    
		m = M2C;  
	}
	else if (ratio <= K3C)
	{    
		b = B3C;    
		m = M3C;  
	}
	else if (ratio <= K4C)//276
	{    
		b = B4C;    
		m = M4C;  
	}
	else if (ratio > K5C)//276
	{    
		b = B5C;    
		m = M5C;  
	}

	temp = ((channel0 * b) - (channel1 * m));
	temp += (1 << (LUX_SCALE - 1));			// round lsb (2^(LUX_SCALE-1))
	//  temp = temp + 32768;
	lux_temp = temp >> LUX_SCALE;			// strip off fractional portion
	
#ifdef USE_HARDWARE_I2C
		obj->m_value = lux_temp;
#endif
	return (lux_temp);		  			    // Signal I2C had no errors
}

