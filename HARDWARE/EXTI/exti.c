#include "delay.h"
#include "exti.h"
#include "TSL2581.h"

// 初始化外部中断函数
void ExInterruptConfig(light_sensor_cb *p_cb)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : PCPin PCPin PCPin PCPin
						   PCPin PCPin PCPin PCPin
						   PCPin */
	GPIO_InitStruct.Pin = SENSOR_INT0_Pin|SENSOR_INT1_Pin|SENSOR_INT2_Pin|SENSOR_INT3_Pin
						  |SENSOR_INT4_Pin|SENSOR_INT5_Pin|SENSOR_INT6_Pin|SENSOR_INT7_Pin
						  |SENSOR_INT8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// 绑定中断回调函数
	for(u8 i = 0; i < MAX_LIGHT_SENSOR; i++)
	{
		g_TSL2581_mux[i].interrupt_cb = p_cb[i];
	}
	
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* NOTE: This function Should not be modified, when the callback is needed,
		   the HAL_GPIO_EXTI_Callback could be implemented in the user file
	*/
	printf("%s,%d\r\n", __func__, __LINE__);
	switch(GPIO_Pin)
	{
		case SENSOR_INT0_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[0].interrupt_cb != NULL)
					g_TSL2581_mux[0].interrupt_cb();
			}
			break;
			
		case SENSOR_INT1_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT1_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[1].interrupt_cb != NULL)
					g_TSL2581_mux[1].interrupt_cb();
			}
			break;
		case SENSOR_INT2_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT2_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[2].interrupt_cb != NULL)
					g_TSL2581_mux[2].interrupt_cb();
			}
			break;
		case SENSOR_INT3_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[3].interrupt_cb != NULL)
					g_TSL2581_mux[3].interrupt_cb();
			}
			break;
		case SENSOR_INT4_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[4].interrupt_cb != NULL)
					g_TSL2581_mux[4].interrupt_cb();
			}
			break;
		case SENSOR_INT5_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[5].interrupt_cb != NULL)
					g_TSL2581_mux[5].interrupt_cb();
			}
			break;
		case SENSOR_INT6_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[6].interrupt_cb != NULL)
					g_TSL2581_mux[6].interrupt_cb();
			}
			break;
		case SENSOR_INT7_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[7].interrupt_cb != NULL)
					g_TSL2581_mux[7].interrupt_cb();
			}
			break;
		case SENSOR_INT8_Pin:
			delay_ms(10); // 消抖
			if(HAL_GPIO_ReadPin(GPIOC, SENSOR_INT0_Pin) == 0) // 判断是否产生中断
			{
				if(g_TSL2581_mux[8].interrupt_cb != NULL)
					g_TSL2581_mux[8].interrupt_cb();
			}
			break;
	}
}
