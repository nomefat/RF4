#include "stm32f4xx_hal.h"
#include "string.h"
#include "gprs_hal.h"
#include "main.h"



#define GPRS_RCV_DMA_BUFF_LENGTH (1024+512)  // gprs���ڽ���


extern UART_HandleTypeDef huart2;

uint8_t gprs_receive_dma_buff[GPRS_RCV_DMA_BUFF_LENGTH];



extern void gprs_str_copy_to_queue(unsigned short len,char* p_data);


/**********************************************************************************************
***func: ���ڵ�Ƭ��һ�ο���DMA����
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_gprs_dma_receive()
{
	SET_BIT((&huart2)->Instance->CR1, USART_CR1_IDLEIE);  //�򿪴��ڿ����ж�
	HAL_UART_Receive_DMA(&huart2,gprs_receive_dma_buff,GPRS_RCV_DMA_BUFF_LENGTH);	 //��DMA����
}




/**********************************************************************************************
***func:���ڿ����жϻص�����
***     �����ж������ж�һ�����ݵĽ���
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_gprs_idle_callback()
{
	HAL_DMA_Abort((&huart2)->hdmarx);
	huart2.RxState = HAL_UART_STATE_READY;
	huart2.hdmarx->State = HAL_DMA_STATE_READY;
	gprs_str_copy_to_queue(GPRS_RCV_DMA_BUFF_LENGTH-DMA1_Stream5->NDTR,(char*)gprs_receive_dma_buff);

	HAL_UART_Receive_DMA(&huart2,gprs_receive_dma_buff,GPRS_RCV_DMA_BUFF_LENGTH);	 //��DMA����


	
}



void grps_power_off()
{
	HAL_GPIO_WritePin(GPRS_pwoer_onoff_GPIO_Port,GPRS_pwoer_onoff_Pin,GPIO_PIN_RESET);
}


void grps_power_on()
{
	HAL_GPIO_WritePin(GPRS_pwoer_onoff_GPIO_Port,GPRS_pwoer_onoff_Pin,GPIO_PIN_SET);

}








void led_1_close()
{
	HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_SET);
}









