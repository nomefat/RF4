#include "stm32f4xx_hal.h"
#include "typedef_struct.h"
#include "rf_hal.h"
#include "hal_cc2520.h"
#include "debug_uart.h"
#include "ap_param.h"



#define INCLUDE_PA

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

void rf_io_tx(SPI_HandleTypeDef* hspi);
void rf_cmd_tx(SPI_HandleTypeDef* hspi);
void rf_rx(SPI_HandleTypeDef* hspi);
void rf_cs_off(SPI_HandleTypeDef* hspi);



extern void rf_rx_data_handle(int index);
extern struct_systerm_info systerm_info;

struct_rf_stat rf_stat[4];

SPI_HandleTypeDef* rf_index[] = {RF4,RF3,RF2,RF1};

uint8_t rf_sec_flag = 0;         //秒点标志 由同步包中断置1

uint8_t rf_scan_channel_enable = 0;  //单载波扫描通道使能

uint8_t rf_send_1000_p_enable = 0;   //发送1000包使能

unsigned char rf_send_data[15] = {0x0f,0x00,0x00,0x00,0x00,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e};

uint8_t rf_rx_buff[4][256];

uint32_t syn_send_error = 0;


struct_test_1000p_data test_1000p_data=
{
	sizeof(struct_test_1000p_data),
	0xaaaa5555,
	1000,
	0,
};



extern char gprs_debug_buff[256];

extern struct_ap_param ap_param;


int spi_to_index(SPI_HandleTypeDef* hspi)
{
	if( hspi == &hspi3) 
		return 3;
	else if(hspi == &hspi5) 
		return 2;	
	else if(hspi == &hspi1)  
		return 1;
	else if(hspi == &hspi4) 
		return 0;	
	else
		return 0;
}




HAL_StatusTypeDef CC2520_set_reg(SPI_HandleTypeDef* hspi,uint16_t addr,uint8_t data)
{

	HAL_StatusTypeDef ret;
	uint8_t cmd[3];
	cmd[0] = (addr>>8) | CC2520_INS_MEMWR;
	cmd[1] = addr &0x00ff;
	cmd[2] = data;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_Transmit(hspi,cmd,3,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return ret;

}

HAL_StatusTypeDef CC2520_send_cmd(SPI_HandleTypeDef* hspi,uint8_t data)
{

	HAL_StatusTypeDef ret;


	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_Transmit(hspi,&data,1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return ret;

}

uint8_t CC2520_get_data(SPI_HandleTypeDef* hspi,uint8_t cmd)
{

	HAL_StatusTypeDef ret;
	uint8_t data;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_TransmitReceive(hspi,&cmd,&data,1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return data;

}




HAL_StatusTypeDef CC2520_get_reg(SPI_HandleTypeDef* hspi,uint16_t addr,uint8_t * pdata)
{

	HAL_StatusTypeDef ret;
	uint8_t cmd[3];
	
	cmd[0] = (addr>>8) | CC2520_INS_MEMRD;
	cmd[1] = addr &0x00ff;
	cmd[2] = 0xff;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_TransmitReceive(hspi,cmd,pdata,3,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);		
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
		

	return ret;	


}


void CC2520_Reg_Init(SPI_HandleTypeDef* hspi)
{
	HAL_StatusTypeDef stat;
	
  #ifdef INCLUDE_PA

  stat = CC2520_set_reg(hspi,CC2520_TXPOWER,     0x88);
  stat = CC2520_set_reg(hspi,CC2520_TXCTRL,      0xC1);
  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x16);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL4,   0x46);                                                                        
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL5,   0x47);
	//CC2520_MEMWR8(CC2520_GPIOCTRL5,   CC2520_GPIO_LOW);
  stat = CC2520_set_reg(hspi,CC2520_GPIOPOLARITY,0x0F);
    
  #else
  stat = CC2520_set_reg(hspi,CC2520_TXPOWER,     0x32);
  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x11);
  #endif

		  
  stat = CC2520_set_reg(hspi,CC2520_CCACTRL0,    0xF8);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL0,    0x83);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL1,    0x14);
  stat = CC2520_set_reg(hspi,CC2520_RXCTRL,      0x3F);
  stat = CC2520_set_reg(hspi,CC2520_FSCTRL,      0x5A);
  stat = CC2520_set_reg(hspi,CC2520_FSCAL1,      0x03);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST0,    0x10);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST1,    0x0E);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST2,    0x03);  
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0,    0x40);
  stat = CC2520_set_reg(hspi,CC2520_EXTCLOCK,    0x00); 
//  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL3,   CC2520_GPIO_SFD); 
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL0,   CC2520_GPIO_FIFO);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL3,   0x88);     //SFD CCA 改为 TX RFOFF 管脚
//	stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL2,   0x8e);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL1,   CC2520_GPIO_FIFOP);  
//  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL2,   CC2520_GPIO_SAMPLED_CCA);
  stat = CC2520_set_reg(hspi,CC2520_FRMFILT1,    0x18);   
  stat = CC2520_set_reg(hspi,CC2520_FRMFILT0,    0x00); 
  

	if(rf_stat[spi_to_index(hspi)].mode == RF_WORK)
		return;

//ENABLE_NOMODULATE_CARRIER	
	if(rf_stat[spi_to_index(hspi)].mode == ENABLE_NOMODULATE_CARRIER)	{

  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0 , 0x43); 
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL1 , 0x00); 
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST0 , 0x65);
 
	}
   
//  ENABLE_MODULATE_CARRIER
	if(rf_stat[spi_to_index(hspi)].mode == ENABLE_MODULATE_CARRIER)	{

  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0 ,   0x43); 
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL1 ,   0x00); 
//  stat = CC2520_set_reg(hspi,CC2520_GPIOPOLARITY,0x00);
  stat = CC2520_set_reg(hspi,CC2520_CCACTRL0,    0xF8);
  stat = CC2520_set_reg(hspi,CC2520_EXTCLOCK,    0x00);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL0,    0x85);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL1,    0x14);
  stat = CC2520_set_reg(hspi,CC2520_RXCTRL,      0x3F);
  stat = CC2520_set_reg(hspi,CC2520_FSCTRL,      0x5A);
  stat = CC2520_set_reg(hspi,CC2520_FSCAL1,      0x2B);
//  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x11);  
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST0,    0x10);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST1,    0x0E);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST2,    0x03);  

 
//  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL2,  0x00);
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST0 , 0x05);
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST1 , 0x08);
//  
	}

	
}


RF_init_stat_typedef rf_init(SPI_HandleTypeDef* hspi)
{
	uint32_t tickstart = 0U;
	uint32_t Timeout ;
	uint8_t data_[3];
	HAL_StatusTypeDef stat;
	
	
	if( hspi == &hspi3) 
	{
						
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);		
		
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms

		HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == GPIO_PIN_RESET )          //判断spi管脚是否被初始化
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //启动失败
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;

	}
	else if( hspi == &hspi1) 
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);
		
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms

		HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET )          //判断spi管脚是否被初始化
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //启动失败
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		


	}
	else if(hspi == &hspi4)  
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);		
		
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms

		HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();

		while( HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == GPIO_PIN_RESET )          //判断spi管脚是否被初始化
		{
			if((HAL_GetTick()-tickstart) > 250)
			{				
				return RF_SPI_ERROR;                              //启动失败				
			}
		
		}
		HAL_GPIO_WritePin(general_led_5_GPIO_Port,general_led_5_Pin,GPIO_PIN_SET);
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		
	}
	else if(hspi == &hspi5)  
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);			
		
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms

		HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_8) == GPIO_PIN_RESET )          //判断spi管脚是否被初始化
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //启动失败
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //延时1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		
	}


	
	CC2520_Reg_Init(hspi);
	
	CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
	
	CC2520_send_cmd(hspi,CC2520_INS_SRXON);	
	
	return RF_REG_INIT_OK;
}





void rf_write_buff(SPI_HandleTypeDef* hspi,void *ptr,int len)
{
	HAL_StatusTypeDef ret;

	uint8_t d = CC2520_INS_TXBUF;
	
//	CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
//	CC2520_send_cmd(hspi,CC2520_INS_STXON);	
	
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	ret = HAL_SPI_Transmit(hspi,&d,1,1);
	
	ret = HAL_SPI_Transmit(hspi,ptr,len+1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	


//	CC2520_send_cmd(hspi,CC2520_INS_STXON);	
	

	
//	rf_io_tx(hspi);	
//	rf_cmd_tx(hspi);
}

void rf_start_send()
{


}

void rf_power_off(SPI_HandleTypeDef* hspi)
{

	if( hspi == &hspi3)
	{		
		HAL_GPIO_WritePin(SPI3_rf_power_onoff_GPIO_Port,SPI3_rf_power_onoff_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	

	}
	else if(hspi == &hspi5) 
	{
		HAL_GPIO_WritePin(SPI5_rf_power_onoff_GPIO_Port,SPI5_rf_power_onoff_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	else if(hspi == &hspi1)  
	{
		HAL_GPIO_WritePin(SPI1_rf_power_onoff_GPIO_Port,SPI1_rf_power_onoff_Pin,GPIO_PIN_RESET);
	}
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_power_onoff_GPIO_Port,SPI4_rf_power_onoff_Pin,GPIO_PIN_RESET);
	}


}

void rf_power_on(SPI_HandleTypeDef* hspi)
{

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_power_onoff_GPIO_Port,SPI3_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_power_onoff_GPIO_Port,SPI5_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_power_onoff_GPIO_Port,SPI1_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_power_onoff_GPIO_Port,SPI4_rf_power_onoff_Pin,GPIO_PIN_SET);


}


void rf_io_tx(SPI_HandleTypeDef* hspi)
{
	uint8_t data[3];
	int32_t time = 0;
	uint32_t tickstart = 0U;	
	
	if( hspi == &hspi3) 
	{
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi5) 
	{
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi1)  
	{
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_SET);
	}
	time = SysTick->VAL;
	
	tickstart = HAL_GetTick();
	while( CC2520_get_data(hspi,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV ))
	{
		if(HAL_GetTick()-tickstart>0)
			break;
	}
	
	time = time - SysTick->VAL;
	time = time;
		
	CC2520_get_reg(hspi,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHTX ); 
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHRX ); 
	}	
}


void rf_io_tx_4()
{
	uint8_t data[3];
	int32_t time = 0;
	uint32_t tickstart = 0U;	
	uint8_t wait_flag = 0;
	
	uint32_t tme[4] = {0,0,0,0};
	
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)	
	{
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_SET);
	}

	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_SET);
	}
	
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_SET);
	}
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_SET);
	}
	
	time = SysTick->VAL;
	
	tickstart = HAL_GetTick();
	
	wait_flag = 0x0f;
	while(1)
	{
		if(wait_flag & 1)
		{
			tme[0]++;
			if(!(CC2520_get_data(RF1,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				wait_flag &= 0xfe;
					HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_SET);	
			}
		}
		if(wait_flag & (1<<1))
		{
			tme[1]++;
			if(!(CC2520_get_data(RF2,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xfd;
			}
		}
		if(wait_flag & (1<<2))
		{
			tme[2]++;
			if(!(CC2520_get_data(RF3,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xfb;
			}
		}
		if(wait_flag & (1<<3))
		{
			tme[3]++;
			if(!(CC2520_get_data(RF4,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xf7;
			}
		}
			
		if(wait_flag == 0)
			break;
		
		if(HAL_GetTick()-tickstart>0)
		{
			syn_send_error++;
			break;
		}
		
		if(SysTick->VAL < 1000)
		{
			syn_send_error++;
			break;
		}
	}
	
	time = time - SysTick->VAL;
	time = time;
	
	CC2520_get_reg(RF1,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX ); 
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX ); 	
	}	
	
	CC2520_get_reg(RF2,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX ); 
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX ); 	
	}

	CC2520_get_reg(RF3,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX ); 
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX ); 	
	}

	CC2520_get_reg(RF4,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX ); 
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX ); 	
	}	
}



void rf_cmd_tx(SPI_HandleTypeDef* hspi)
{
		CC2520_send_cmd(hspi,CC2520_INS_STXON);
}


void rf_power_reset(SPI_HandleTypeDef hspi1)
{



}

void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel )
{
    uint16_t   uiReg;
	
		CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);	
		CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
	
    if(uiChannel <16)  
			uiReg       =   uiChannel + ( uiChannel << 2u ) + 0x0Bu;		// uiChannel * 5u + 0x4165.
		else
		{
			uiChannel -= 16;
			uiReg       =   uiChannel + ( uiChannel << 2u ) + 0x0Bu + 2;		// uiChannel * 5u + 0x4165.
		}
    CC2520_set_reg(hspi,CC2520_FREQCTRL,uiReg);

		CC2520_send_cmd(hspi,CC2520_INS_STXON);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState bitstatus;
	
	if(GPIO_PIN_4 == GPIO_Pin)
	{
		if((GPIOF->IDR & (1<<4)) && (GPIOF->IDR & (1<<5)))
		{
			HAL_GPIO_WritePin(SPI5_led_rf_send_GPIO_Port,SPI5_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi5);
		}
	}
	else if(GPIO_PIN_2 == GPIO_Pin)
	{
		if((GPIOI->IDR & (1<<2)) && (GPIOI->IDR & (1<<3)))
		{
			HAL_GPIO_WritePin(SPI3_led_rf_send_GPIO_Port,SPI3_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi3);		
		}
	}
	else if(GPIO_PIN_10 == GPIO_Pin)
	{
		if((GPIOB->IDR & (1<<10)) && (GPIOB->IDR & (1<<11)))
		{
			HAL_GPIO_WritePin(SPI4_led_rf_send_GPIO_Port,SPI4_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi4);
		}
	}
	else if(GPIO_PIN_12 == GPIO_Pin)
	{
		if((GPIOF->IDR & (1<<12)) && (GPIOF->IDR & (1<<11)))
		{
			HAL_GPIO_WritePin(SPI1_led_rf_send_GPIO_Port,SPI1_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi1);		
		}
	}	
}



void rf_rx(SPI_HandleTypeDef* hspi)
{
	HAL_StatusTypeDef ret;
	
	uint8_t data = CC2520_INS_RXBUF;

	//spi片选
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	ret = HAL_SPI_Transmit(hspi,&data,1,1); //发送读buf 命令
	ret = HAL_SPI_Receive(hspi,&data,1,1); //读一个字节，buf长度
	

	if(hspi == RF1)	
	{
		rf_rx_buff[0][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rf_rx_buff[0][1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[0][1],data,1);
		//从中断到这里(接收别的AP的同步包)  58US 
		rf_cs_off(hspi);		
		rf_rx_data_handle(0);
		//从中断到这里(接收别的AP的同步包)   65US
		HAL_GPIO_WritePin(SPI4_led_rf_send_GPIO_Port,SPI4_led_rf_send_Pin,GPIO_PIN_SET);	
	}
	else if(hspi == RF2)
	{
		rf_rx_buff[1][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rdata2[1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[1][1],data,1);	
		HAL_GPIO_WritePin(SPI1_led_rf_send_GPIO_Port,SPI1_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);		
		rf_rx_data_handle(1);
	
	}
	else if(hspi == RF3)	
	{
		rf_rx_buff[2][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rf_rx_buff[2][1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[2][1],data,1);
		HAL_GPIO_WritePin(SPI5_led_rf_send_GPIO_Port,SPI5_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);	
		rf_rx_data_handle(2);		

	}
	else if(hspi == RF4)
	{
		rf_rx_buff[3][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rdata2[1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[3][1],data,1);	
		HAL_GPIO_WritePin(SPI3_led_rf_send_GPIO_Port,SPI3_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);
		rf_rx_data_handle(3);		
	
	}
	
		
}

void rf_cs_off(SPI_HandleTypeDef* hspi)
{
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	

}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
//		sprintf(debug_send_buff,"d1 %d %x %d %d %d %d %d %d\r\n",rdata1[0],rdata1[1],rdata1[2],rdata1[3],rdata1[4],rdata1[5],rdata1[6],rdata1[7]);
//		debug_uart_send_string(debug_send_buff);		
	}
}







void rf_rx_voerflow_check()
{
	static unsigned char timeout[4] = {0,0,0,0};
	
	if((GPIOF->IDR & (1<<4)) ) //&& !(GPIOF->IDR & (1<<5)))
		{
			timeout[0]++;
			if(timeout[0]>3){
				timeout[0] = 0;
				CC2520_send_cmd(rf_index[1],CC2520_INS_SFLUSHRX );
				CC2520_send_cmd(rf_index[1],CC2520_INS_SFLUSHRX ); 
			}
		}	
		else
			timeout[0] = 0;


	if((GPIOI->IDR & (1<<2))) //&& !(GPIOI->IDR & (1<<3)))
		{
			timeout[1]++;
			if(timeout[1]>3){
				timeout[1] = 0;			
			CC2520_send_cmd(rf_index[2],CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(rf_index[2],CC2520_INS_SFLUSHRX ); 
			}
		}		
		else
			timeout[1] = 0;
		
	if((GPIOB->IDR & (1<<10)) )//&& !(GPIOB->IDR & (1<<11)))
		{
			timeout[2]++;
			if(timeout[2]>3){
				timeout[2] = 0;			
			CC2520_send_cmd(rf_index[3],CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(rf_index[3],CC2520_INS_SFLUSHRX ); 
			}
		}	
		else
			timeout[2] = 0;
		
	if((GPIOF->IDR & (1<<12))) //&& !(GPIOF->IDR & (1<<11)))
		{
			timeout[3]++;
			if(timeout[3]>3){
				timeout[3] = 0;				
			CC2520_send_cmd(rf_index[0],CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(rf_index[0],CC2520_INS_SFLUSHRX ); 
			}
		}	
		else
			timeout[3] = 0;
}

void rf_satt_init()
{
	int i;
	for(i=0;i<4;i++)	
	{
		rf_stat[i].mode = RF_DEFAULT_MODE;		
	}
	
}
	 int channel = 0;
void rf_scan_channel()
{

	int i = 0;
	
	struct _led
	{
		GPIO_TypeDef * port;
		uint32_t pin;
	}struct_led[4] = {
	general_led_1_GPIO_Port,general_led_1_Pin,
	general_led_2_GPIO_Port,general_led_2_Pin,
	general_led_3_GPIO_Port,general_led_3_Pin,
	general_led_4_GPIO_Port,general_led_4_Pin};
	
  if(rf_scan_channel_enable != 1)
		return;
	
	if(systerm_info.slot%2048 == 1)
	{
		for(i=0;i<4;i++)
		{
			if(rf_stat[i].mode == ENABLE_NOMODULATE_CARRIER || rf_stat[i].mode == ENABLE_MODULATE_CARRIER)
			{
				rf_set_channel(rf_index[i],channel);
				HAL_GPIO_TogglePin(struct_led[i].port,struct_led[i].pin);
				sprintf(gprs_debug_buff,"rf scan ch=%d\r\n",channel);
				debug_uart_send_string(gprs_debug_buff);					
			}
		}
			
		channel++;
		if(channel >15)
			channel = 0;

		while(systerm_info.slot%2048 == 1);
	}

}


void rf_send_1000p()
{
	uint32_t *packet_seq = (uint32_t *)&rf_send_data[1];
	
	if(systerm_info.slot%32 == 1)
	{
		if(rf_send_1000_p_enable & EN_RF1 && rf_stat[0].mode == RF_IDLE && rf_stat[0].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF1,rf_send_data,rf_send_data[0]);
				
				CC2520_send_cmd(RF1,CC2520_INS_STXON);
				HAL_GPIO_TogglePin(general_led_1_GPIO_Port,general_led_1_Pin);
				sprintf(gprs_debug_buff,"rf1_send seq=%d\r\n",*packet_seq);
				debug_uart_send_string(gprs_debug_buff);	
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF2 && rf_stat[1].mode == RF_IDLE && rf_stat[1].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF2,rf_send_data,rf_send_data[0]);
				
				CC2520_send_cmd(RF2,CC2520_INS_STXON);
				HAL_GPIO_TogglePin(general_led_2_GPIO_Port,general_led_2_Pin);
				sprintf(gprs_debug_buff,"rf2_send seq=%d\r\n",*packet_seq);
				debug_uart_send_string(gprs_debug_buff);	
				(*packet_seq)++;
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF3 && rf_stat[2].mode == RF_IDLE && rf_stat[2].rf_power_stat == RF_POWER_WORK)
		{

			if((*packet_seq)< 1000)
			{
			
				rf_write_buff(RF3,rf_send_data,rf_send_data[0] );

				CC2520_send_cmd(RF3,CC2520_INS_STXON);
				HAL_GPIO_TogglePin(general_led_3_GPIO_Port,general_led_3_Pin);
				sprintf(gprs_debug_buff,"rf3_send seq=%d\r\n",*packet_seq);
				debug_uart_send_string(gprs_debug_buff);		
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF4 && rf_stat[3].mode == RF_IDLE && rf_stat[3].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF4,rf_send_data,rf_send_data[0] );

				CC2520_send_cmd(RF4,CC2520_INS_STXON);
				HAL_GPIO_TogglePin(general_led_4_GPIO_Port,general_led_4_Pin);
				sprintf(gprs_debug_buff,"rf4_send seq=%d\r\n",*packet_seq);
				debug_uart_send_string(gprs_debug_buff);				
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}		
	}
}


void rf_manage()
{
	int i;
	
	uint8_t *p_chanel = (uint8_t *)&ap_param.ap_channel;

	rf_scan_channel();
	rf_send_1000p();
	
	if(rf_sec_flag < 1) //1sec进入下面一次
		return;


	
	rf_sec_flag = 0;
	
	
	for(i=0;i<4;i++)
	{
		if(rf_stat[i].rf_power_stat != RF_POWER_WORK)    //电源没在工作状态
		{
			if(rf_stat[i].rf_power_stat == RF_POWER_ON)    //电源为ON状态
			{
				rf_power_on(rf_index[i]);
				rf_stat[i].reg_init_stat = rf_init(rf_index[i]);
				if(rf_stat[i].reg_init_stat != RF_REG_INIT_OK)
					rf_stat[i].rf_power_stat = RF_POWER_OFF;
				else
				{
					rf_stat[i].rf_power_stat = RF_POWER_WORK;
					rf_set_channel(rf_index[i],p_chanel[i]);
					if(rf_stat[i].mode == ENABLE_NOMODULATE_CARRIER || rf_stat[i].mode == ENABLE_MODULATE_CARRIER )
						rf_cmd_tx(rf_index[i]);

				}
					

			}
			else                                           //电源为OFF或者其他异常状态
			{
				rf_power_off(rf_index[i]);
				rf_stat[i].rf_power_stat = RF_POWER_ON;
			}
			
		}
		
	}
	
	rf_rx_voerflow_check();
	
	
}







extern char gprs_debug_buff[256];

char *str_power[] = {"off","on","ok"};
char *str_init[] = {"reg_error","spi_error","ok"};
char *str_mode[] = {"work","on_MC","MC"};


char* make_rf_stat()
{
	
	
	sprintf(gprs_debug_buff,"RF1:power=%s init=%s mode=%s \r\nRF2:power=%s init=%s mode=%s \r\nRF3:power=%s init=%s mode=%s \r\nRF4:power=%s init=%s mode=%s \r\n",str_power[rf_stat[0].rf_power_stat],str_init[rf_stat[0].reg_init_stat+2],str_mode[rf_stat[0].mode],
																			str_power[rf_stat[1].rf_power_stat],str_init[rf_stat[1].reg_init_stat+2],str_mode[rf_stat[1].mode],
																			str_power[rf_stat[2].rf_power_stat],str_init[rf_stat[2].reg_init_stat+2],str_mode[rf_stat[2].mode],
																			str_power[rf_stat[3].rf_power_stat],str_init[rf_stat[3].reg_init_stat+2],str_mode[rf_stat[3].mode]	);
	return gprs_debug_buff;
}





