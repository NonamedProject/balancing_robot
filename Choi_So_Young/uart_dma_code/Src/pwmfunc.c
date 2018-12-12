#include "main.h"
#include "stm32l0xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pwmfunc.h"

void Dir_control(uint8_t str[])
{
	switch(str[0])
		{
			case 'w':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_RESET);
				break;
			case 's':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_SET);
				break;
			case 'd':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
				break;
			case 'a':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
				break;
			default:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET); 
				break;
		}
}

uint16_t Conversion(uint8_t str[])
{
	int g=1;
	uint16_t val=0;
	int temp;
	
	for(i=3;i>0;i--){
		temp = str[i] - 0x30;
		val+=temp*g;
		g*=10;
	}
	return val;
}

void speed(uint16_t val)
{
	TIM2->CCR1 = val;
	TIM3->CCR1 = val;
}

