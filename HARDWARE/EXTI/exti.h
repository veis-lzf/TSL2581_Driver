#ifndef __EXTI_H__
#define __EXTI_H__

#include "TSL2581.h"

// ���Ŷ���
#define SENSOR_INT0_Pin GPIO_PIN_4
#define SENSOR_INT0_GPIO_Port GPIOC
#define SENSOR_INT0_EXTI_IRQn EXTI4_IRQn
#define SENSOR_INT1_Pin GPIO_PIN_5
#define SENSOR_INT1_GPIO_Port GPIOC
#define SENSOR_INT1_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_INT2_Pin GPIO_PIN_6
#define SENSOR_INT2_GPIO_Port GPIOC
#define SENSOR_INT2_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_INT3_Pin GPIO_PIN_7
#define SENSOR_INT3_GPIO_Port GPIOC
#define SENSOR_INT3_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_INT4_Pin GPIO_PIN_8
#define SENSOR_INT4_GPIO_Port GPIOC
#define SENSOR_INT4_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_INT5_Pin GPIO_PIN_9
#define SENSOR_INT5_GPIO_Port GPIOC
#define SENSOR_INT5_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_INT6_Pin GPIO_PIN_10
#define SENSOR_INT6_GPIO_Port GPIOC
#define SENSOR_INT7_Pin GPIO_PIN_11
#define SENSOR_INT7_GPIO_Port GPIOC
#define SENSOR_INT8_Pin GPIO_PIN_12
#define SENSOR_INT8_GPIO_Port GPIOC

// ��ʼ���ⲿ�ж�
void ExInterruptConfig(light_sensor_cb *p_cb);


#endif /* __EXTI_H__ */