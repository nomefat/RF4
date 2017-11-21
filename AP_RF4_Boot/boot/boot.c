#include "main.h"
#include "stm32f4xx_hal.h"

#pragma pack(1)

typedef struct _ap_n1_protocol
{
	unsigned short head;
	unsigned int packet_syn;
	unsigned char lengh;    //后面的长度 不包含本身
	unsigned char cmd;
	unsigned short packet_all;
	unsigned short packet_syn_;
	unsigned char data[1];
	
}struct_ap_n1_protocol;



#define FLASH_ADDR 0x08004000

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern TIM_HandleTypeDef htim1;


void from_n1_data_handle(void);
int write_flash(uint32_t Address, uint8_t *pData,uint8_t len);
void get_firmware_handle(struct_ap_n1_protocol *ptr);	
void send_to_n1_ack(void);
void send_to_n1(void);
void send_to_n1_enable(void);



uint8_t from_n1_data[2][256]={0};
uint32_t from_n1_data_index = 0;

int ms_30;

int ms_500;
int sec;
uint8_t send_n1_data[256];

int flash_syn = 0;

int packet_all = 0;
int resend_flag = 0;

int re_send_30ms = 0;

extern int start_app ;

/*** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
unsigned short const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

unsigned short  crc16_byte(unsigned short crc, const unsigned char data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}
/***
 * crc16 - compute the CRC-16 for the data buffer
 * @crc:	previous CRC value
 * @buffer:	data pointer
 * @len:	number of bytes in the buffer
 *
 * Returns the updated CRC value.
 */
unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
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





/**********************************************************************************************
***func:处理N1接收到的数据
***   
***date: 2017/6/9
*** nome
***********************************************************************************************/
void from_n1_data_handle(void)
{
	uint8_t reback[20];
	uint16_t uiCrcValue;
	uint16_t *p_r_crc;
	struct_ap_n1_protocol *ptr = (struct_ap_n1_protocol *)(&from_n1_data[(from_n1_data_index-1) & 0x01]);
	
//	sprintf(reback,"write buff %d\r\n",from_n1_data_index & 0x01);
//	memcpy(send_to_n1_ack,from_n1_data[(from_n1_data_index-1) & 0x01],10);
	//HAL_UART_Transmit_DMA(&huart3,from_n1_data[(from_n1_data_index-1) & 0x01],50);
	
	if(ptr->head != 0x55aa)
		return;
	
	uiCrcValue = crc16(0, &ptr->cmd, ptr->lengh);	
	p_r_crc = (uint16_t *)(&ptr->cmd + ptr->lengh);
	if(*p_r_crc != uiCrcValue)
		return;

	switch(ptr->cmd)
	{
		case 1: resend_flag = 0;break;
		case 6: send_to_n1_ack();
						get_firmware_handle(ptr);
						HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
						break;

		default : break;
	}
	
	
}
int p = 0;
/**********************************************************************************************
***func:处理N1接收到的固件
***   
***date: 2017/6/9
*** nome
***********************************************************************************************/
void get_firmware_handle(struct_ap_n1_protocol *ptr)
{
	static uint32_t Address = 0;
	static uint32_t time_ms_30 = 0;
	int i;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;
	
	void( *pfunc)() = (void( *))(*((uint32_t *)0x08004004));
	
	HAL_FLASH_Unlock();
	p = ptr->packet_syn_;
	if(ptr->packet_syn_ == flash_syn)  //正确的包 或者是第0包 代表开始
	{
		packet_all = ptr->packet_all; //记下总包数
		
		if( ptr->packet_syn_ == 0) // 收到第0包后需要擦除Flash
		{
			start_app = 1;
			flash_syn = 0;
			HAL_FLASH_Unlock();
			Address = FLASH_ADDR;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			EraseInitStruct.Sector = 1;
			EraseInitStruct.NbSectors = 4;
			if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
			{ 
				HAL_FLASH_Lock();
				Error_Handler();
			}
			HAL_FLASH_Lock();
		}		
		if(write_flash(Address,ptr->data,ptr->lengh-5) == -1)  //写失败之后需要重新给N1发送要第0包的指令
		{
			flash_syn = 0;
			Address = FLASH_ADDR;
		}
		else  //写成功
		{   
			Address += ptr->lengh-5;
			flash_syn++;
			
			if(flash_syn>= packet_all)
			{
				send_to_n1_enable();
				start_app = 0;
				sec = 2;
//				__set_MSP(*((uint32_t *)0x08004000));
//				HAL_TIM_Base_Stop(&htim1);
//				pfunc();
			}
			send_to_n1_enable();
		}
	}
	else  //包不对 要正确的包
	{
		if(ms_30 -time_ms_30 > 5)
		{
			time_ms_30 = ms_30;
			send_to_n1_enable();
		}
	}
	HAL_FLASH_Lock();
}

uint8_t send_to_n1_ack_d[10]={0xaa,0x55,0,0,0,0,1,1,0xc1,0xc0};
void send_to_n1_ack(void)
{
	struct_ap_n1_protocol *ptr = (struct_ap_n1_protocol *)send_n1_data;	
	
	resend_flag = 1;
	
	ptr->head = 0x55aa;
	ptr->packet_syn +=1;
	ptr->cmd = 1;
	ptr->lengh = 1;
	
	HAL_UART_Transmit(&huart3,send_to_n1_ack_d,10,1);
}




void send_to_n1(void)
{
	struct_ap_n1_protocol *ptr = (struct_ap_n1_protocol *)send_n1_data;	
	int16_t uiCrcValue=0;
	resend_flag = 1;
	
	ptr->head = 0x55aa;
	ptr->packet_syn +=1;
	ptr->packet_all = packet_all;
	ptr->packet_syn_ = flash_syn;
	ptr->cmd = 56;
	ptr->lengh = 5;

  uiCrcValue = crc16(0, &send_n1_data[7], ptr->lengh+1);	
	
	HAL_UART_Transmit(&huart3,(uint8_t *)ptr,14,1);
}





int write_flash(uint32_t Address, uint8_t *pData,uint8_t len)
{
	int i = 0;
	uint8_t error_times;
	
	HAL_FLASH_Unlock();
	for(i=0;i<len;i=i)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,Address,pData[i]);
		if(*((uint8_t *)Address) != pData[i])
		{
			error_times++;
			if(error_times  >5)
			{
				HAL_FLASH_Lock();
				return -1;
			}
			continue;
		}
		else
		{
			error_times = 0;
			Address++;
			i++;
		}
	}
	HAL_FLASH_Lock();
	return 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	ms_30++;
	if(ms_30%33 == 0)
	{
		sec++;
	}
	if(ms_30%16 == 0)
	{
		ms_500++;
	}
	
	if(resend_flag)
		re_send_30ms = 1;
}

void send_to_n1_enable(void)
{
//	re_send_30ms = 1 ;
	resend_flag = 1;
}

void re_send_to_n1(void)
{
	if(re_send_30ms == 1 && resend_flag == 1)
	{
		re_send_30ms = 0;
		send_to_n1();
	}
}
