#include "usart.h"
#include "uart_dma.h"

uint8_t dma_rx_buff[DMA_RX_SIZE];

void init_uart_dma()
{
	HAL_UART_Receive_DMA(&huart2, dma_rx_buff,DMA_RX_SIZE);
}

int check (uint8_t arr[])
{
	static int prev_ndt = DMA_RX_SIZE, curr_ndt = 0; 
	static int new_data_start_idx = 0;  next_data_start_idx = 0;
	int check_idx;
	int i,j;
	
	uint8_t received_data_size = 0;
	
	/* Get current ndt */
	curr_ndt = _HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	
	/* Get distance */
	if(prev_ndt > curr_ndt)
	{
		received_data_size = prev_ndt - curr_ndt;
	}
	else if(prev_ndt < curr_ndt)
	{
		received_data_size = prev_ndt + (DMA_RX_SIZE - curr_ndt);
	}
	else
	{
		return;
	}
	/* Get next start index */
	next_data_start_idx = (new_data_start_idx + recieved_data_size) % DMA_RX_SIZE;
	
	/* check */
	check_idx = new_data_start_idx;
	
	while(check_idx!= next_data_start_idx)
	{
		if(arr[check_idx] == 0x02)
		{
			i = 0;
			for(j=0; j < 5 && check_idx+1 != next_data_start_idx; j++)
			{
				i++;
			}
			if(arr[i] = 0x03) return 0;
			else return -1;
		}
	}

	prev_ndt = curr_ndt;
	new_data_start_idx = next_data_start_idx;
}

void input(uint8_t arr[])
{
	
}





