#include "stm32f4xx_hal.h"
#include "string.h"
#include "ap_param.h"
#include "to_n1.h"
#include "rf_hal.h"






#pragma pack(1)

typedef struct _ap_n1_protocol
{
	unsigned short head;
	unsigned int packet_syn;
	unsigned char lengh;
	unsigned char cmd;
	unsigned char data[1];
	
}struct_ap_n1_protocol;


uint8_t from_n1_data[2][256]={0};
uint32_t from_n1_data_index = 0;

extern uint8_t send_to_n1_data[256];
extern UART_HandleTypeDef huart3;
extern struct_sensor_rp_param sensor_rp_param ;
extern uint8_t send_to_n1_ack[];
extern struct_ap_param ap_param;
extern struct struct_gprs_stat gprs_stat;
	
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

extern void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel );
extern	void close_tcp_conn(int index);
extern void send_gprs_data(void *pdata,int len);

void n1_get_ap_pram(struct_ap_n1_protocol *ptr);
void n1_set_ap_pram(struct_ap_n1_protocol *ptr);
void n1_send_firmware(struct_ap_n1_protocol *ptr,int who);
void n1_send_2g_data(struct_ap_n1_protocol *ptr);
void n1_set_s_param(struct_ap_n1_protocol *ptr);
void n1_set_rp_param(struct_ap_n1_protocol *ptr);
void n1_get_2g_stat(struct_ap_n1_protocol *ptr);



/**********************************************************************************************
***func:处理N1接收到的数据
***   
***date: 2017/6/9
*** nome
***********************************************************************************************/
void from_n1_data_handle(void)
{
	uint8_t reback[20];
	
	struct_ap_n1_protocol *ptr = (struct_ap_n1_protocol *)(&from_n1_data[(from_n1_data_index-1) & 0x01]);
	
//	sprintf(reback,"write buff %d\r\n",from_n1_data_index & 0x01);
//	memcpy(send_to_n1_ack,from_n1_data[(from_n1_data_index-1) & 0x01],10);
	//HAL_UART_Transmit_DMA(&huart3,from_n1_data[(from_n1_data_index-1) & 0x01],50);
	
	if(ptr->head != 0x55aa)
		return;
	
	switch(ptr->cmd)
	{
		case N1_ACK:send_to_n1_data[0] = 0;break;
		case N1_GET_AP_PARAM: n1_get_ap_pram(ptr);break;
		case N1_SET_AP_PARAM: n1_set_ap_pram(ptr);break;
		case N1_SEND_AP_FIRMWARE: n1_send_firmware(ptr,N1_SEND_AP_FIRMWARE);break;
		case N1_SEND_RP_FIRMWARE: n1_send_firmware(ptr,N1_SEND_RP_FIRMWARE);break;		
		case N1_SEND_S_FIRMWARE: n1_send_firmware(ptr,N1_SEND_S_FIRMWARE);break;
		case N1_SEND_2G_DATA: n1_send_2g_data(ptr);break;
		case N1_SET_S_PARAM: n1_set_s_param(ptr);break;
		case N1_SET_RP_PARAM: n1_set_rp_param(ptr);break;		
		case N1_GET_2G_STAT: n1_get_2g_stat(ptr);break;
		default : break;
	}
	
	
}


/**********************************************************************************************
***func: 用于单片第一次开启DMA接收
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_n1_dma_receive()
{
	SET_BIT((&huart3)->Instance->CR1, USART_CR1_IDLEIE);  //打开串口空闲中断
	HAL_UART_Receive_DMA(&huart3,from_n1_data[from_n1_data_index & 0x01],256);	 //打开DMA接收
}




/**********************************************************************************************
***func:串口空闲中断回调函数
***     空闲中断用来判断一包数据的结束
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_form_n1_idle_callback()
{
	HAL_DMA_Abort((&huart3)->hdmarx);
	huart3.RxState = HAL_UART_STATE_READY;
	huart3.hdmarx->State = HAL_DMA_STATE_READY;
	from_n1_data_index++;
	HAL_UART_Receive_DMA(&huart3,from_n1_data[from_n1_data_index & 0x01],256);   //双缓冲切换

	from_n1_data_handle();                                                       //数据处理

	
	
}



void n1_get_ap_pram(struct_ap_n1_protocol *ptr)
{
	
	send_to_n1_ack[0] = 0xaa;
	insert_to_n1_buff((uint8_t*)&ap_param,sizeof(ap_param),AP_AP_PARAM);
}

void n1_set_ap_pram(struct_ap_n1_protocol *ptr)
{
	struct_ap_param* ptr_ap_param = (struct_ap_param*)(&(ptr->data[0]));	
	
	if(ptr_ap_param->ap_channel != ap_param.ap_channel)      //通道有变化 执行切通道指令
	{
		rf_set_channel(RF1,(ptr_ap_param->ap_channel>>24)&0XFF);
		rf_set_channel(RF2,(ptr_ap_param->ap_channel>>16)&0XFF);
		rf_set_channel(RF3,(ptr_ap_param->ap_channel>>8)&0XFF);
		rf_set_channel(RF4,(ptr_ap_param->ap_channel)&0XFF);		
	}
	if(ptr_ap_param->gprs_server_ip != ap_param.gprs_server_ip || ptr_ap_param->gprs_server_port != ap_param.gprs_server_port)  //服务器ip或者端口有变化 断网
	{
		close_tcp_conn(0);
	}	
	
	memcpy(&(ap_param.band_id),&(ptr->data[8]),sizeof(ap_param)-8);
	send_to_n1_ack[0] = 0xaa;
}

void n1_send_firmware(struct_ap_n1_protocol *ptr,int who)
{
	
}

void n1_send_2g_data(struct_ap_n1_protocol *ptr)
{
	send_gprs_data(ptr->data,ptr->lengh-1);
	send_to_n1_ack[0] = 0xaa;
}

void n1_set_s_param(struct_ap_n1_protocol *ptr)
{
	memcpy(&sensor_rp_param,&(ptr->data[0]),sizeof(sensor_rp_param));
	send_to_n1_ack[0] = 0xaa;
}
	
void n1_set_rp_param(struct_ap_n1_protocol *ptr)
{
	memcpy(&sensor_rp_param.ParaFram,&(ptr->data[0]),sizeof(sensor_rp_param)-1);
	send_to_n1_ack[0] = 0xaa;	
}

void n1_get_2g_stat(struct_ap_n1_protocol *ptr)
{
	insert_to_n1_buff((uint8_t*)&gprs_stat,5,AP_GPRS_STAT);	
	send_to_n1_ack[0] = 0xaa;	
}















