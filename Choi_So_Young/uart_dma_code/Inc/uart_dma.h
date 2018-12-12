#ifndef MPU6050_H
#define MPU6050_H

#include "stm32l0xx_hal.h"

#define DMA_RX_SIZE 1024

typedef union _data
{
	uint8_t buff [4];
	float value;
}DATA;

extern uint8_t dma_rx_buff[MPU_DMA_RX_SIZE];

void init_uart_dma();
int check (uint8_t arr[]);


#endif MPU6050_H