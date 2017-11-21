#include "stm32f4xx_hal.h"
#include "snp.h"
#include "typedef_struct.h"
#include "ap_param.h"
#include "rf_hal.h"
#include "string.h"
#include "to_n1.h"
#include "eeprom.h"
#include "debug.h"
#include "flash.h"
#include "update_s_rp.h"



#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02

SNP_SYNC_PACKET_t syn_packet;
SNP_AP_ACK_PACKET_t ack_packet;
SNP_UF_DATA_PACKET_t upadate_packet;


struct_systerm_info systerm_info;
SNP_AP_ACK_PACKET_t     s_sApAckPacket[4]; 
uint8_t		        uiIndex[4]={0};      //收到事件包的索引


struct _sensor_rp_data_recode{

	uint16_t id;
	uint8_t syn;
}sensor_rp_data_recode[256];

SNNDATA_MESSAGE   g_sSnnDataMessage=
{
  {{SNNMAGIC,
   SNN_DATA_MSG_NO,//2013 lhj  SNN_DATA_MSG_NO原  0x05
   0x2Bu,
  },
  0u,SOURCEID,0u,VERSION,0u,},
  0u,0u,0u,0u,0u
  
};


extern uint8_t gprs_sec_flag ;
extern uint8_t rf_sec_flag ;


extern uint8_t rf_rx_buff[4][256];
extern struct_rf_stat rf_stat[4];
extern char gprs_debug_buff[256];
extern struct_sensor_rp_param sensor_rp_param ;


extern void rf_write_buff(SPI_HandleTypeDef* hspi,void *ptr,int len);
extern void debug_uart_send_string(char *pstr);
extern void get_s_rp_input_update_stat(uint16_t dev_id, uint16_t flash_times);
extern void rf_send_update_packet(void);




extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

extern struct_update_s_rp_manage update_s_rp_manage ;

void ApPacketsetting(u16_t uiCurSlotNr,u8_t ucCurPacketerNr);



int test;


void clear_recode();
uint8_t get_slot_num();

uint8_t add(uint8_t *pdata,uint32_t size)
{
	int i=0;
	uint8_t ret = 0;
	
	for(i=0;i<size;i++)
	{
		ret += pdata[i];
	}
	return ret;
}

void rf_send_syn_packet(void)
{
  memcpy( (uint8_t *)&syn_packet.sensor_param[0], (uint8_t *)ap_param.ap_syn_param, 6 );
	syn_packet.sPhr.ucSize = sizeof( SNP_SYNC_PACKET_t)+1;
	syn_packet.sPhr.uiFcf = 0x0080;
	syn_packet.sPhr.ucSerNr++;
	syn_packet.sPhr.ucType = SNP_PACKET_TYPE_WORK;
	syn_packet.sPhr.uiDevId = ap_param.band_id;
	syn_packet.ucCtrlBm = 0x07;
	syn_packet.uiRemainSlot = 512-(systerm_info.slot%512);

//	syn_packet.uiCrc = 0x88;
	
				
	if(syn_packet.sPhr.ucSerNr%8 == 0)  //秒点
	{
		syn_packet.ucCurSecNr++;
		if(syn_packet.ucCurSecNr>29)
			syn_packet.ucCurSecNr = 0;
		
		syn_packet.ucCtrlBm = 0x06;
		
		if(sensor_rp_param.ParaFram.uiCmd !=0)  //有新的snesor 或者 rp参数
		{
			memcpy(&syn_packet.uiCmd,&sensor_rp_param.ParaFram.uiCmd,14);
			syn_packet.uiBindId = ap_param.band_id;
			syn_packet.sPhr.ucSensorMode = sensor_rp_param.ucSensorMode;
			sensor_rp_param.ParaFram.uiCmd = 0;
			syn_packet.sPhr.ucType |= 1<<4;
			
		}
		else
		{
			memset(&syn_packet.uiCmd,0,14);
			syn_packet.uiBindId = ap_param.band_id;
			syn_packet.sPhr.ucSensorMode = 0;
			syn_packet.sPhr.ucType &= ~(1<<4);
		}
	}
	
	
	
	

	
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)  //点灯
	{
		HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
	}


	


	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));
		rf_write_buff(RF1, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF2, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<6);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF3, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5) |(1<<6);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF4, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	

	
	rf_io_tx_4();
	
	clear_recode();
	
	if(syn_packet.sPhr.ucSerNr%8 == 0) 
	{
		g_sSnnDataMessage.basePhr.sPhr.ucLength=0x11+sizeof(SNP_SYNC_PACKET_t);
		g_sSnnDataMessage.lTimestamp=(u32_t)0u;
		g_sSnnDataMessage.sTime=0u;
		g_sSnnDataMessage.ucLqi=0u;
		g_sSnnDataMessage.ucRssi=0u;
		g_sSnnDataMessage.ucOffset=0x11u;
		memcpy(&g_sSnnDataMessage.ucLoadData[0],&syn_packet,syn_packet.sPhr.ucSize);	
		insert_to_n1_buff((unsigned char *)&g_sSnnDataMessage,syn_packet.sPhr.ucSize+0X11,AP_RF_DATA);		
	}
	
	
		if(syn_packet.uiCmd !=0 && syn_packet.sPhr.ucSerNr%8 == 0)  //有新的snesor 或者 rp参数
		{			
			sprintf(gprs_debug_buff,"syn:set s_rp pram %04X\r\n",sensor_rp_param.ParaFram.uiPoll);
			debug_uart_send_string(gprs_debug_buff);		
		}
}


void rf_send_updata_packet(uint8_t s_or_rp,uint16_t flash_seq)
{
	
	uint32_t flash_base_addr = 0;
	
	uint8_t *p_crc = 0;
	uint8_t i =0;
	
	//更新包 初始化
	upadate_packet.sPhr.ucType = SNP_PACKET_TYPE_UF;
	upadate_packet.sPhr.ucSize = sizeof( SNP_UF_DATA_PACKET_t)+1;
	upadate_packet.sPhr.uiDevId = ap_param.band_id;
	upadate_packet.sPhr.ucSerNr++;
	upadate_packet.sPhr.uiFcf = 0x0080;
	
	//选择固件基地址
	if(FIRM_RP == s_or_rp)
	{
		flash_base_addr = FLASH_RP_FIRMWARE_BEGIN;

	}
	else if(FIRM_SENSOR == s_or_rp)
	{
		flash_base_addr = FLASH_SENSOR_FIRMWARE_BEGIN;

	}
	else
		return;

//按照FLASH序号到指定的地址取得数据	
	upadate_packet.uiAddress = *((uint32_t *)(flash_base_addr + flash_seq*37));
	memcpy(upadate_packet.auiBuffer,(uint8_t *)(flash_base_addr + flash_seq*37 + 5),32);

	if(upadate_packet.uiAddress == 0xffffffff)
	{
		upadate_packet.uiAddress = 0xffffffff;
		update_s_rp_manage.upadate_s_rp_enable = 0;
		update_s_rp_manage.now_send_times = 0;
		update_s_rp_manage.now_upadate_packet_seq = 0;
		update_s_rp_manage.dev_num = 0;
	}
	//计算CRC
	p_crc = (uint8_t *)(flash_base_addr + flash_seq*37);
	upadate_packet.CrcSum = 0;
	for(i=0;i<37;i++)
	{
		if(i == 4)
			continue;
		upadate_packet.CrcSum += p_crc[i];
	}

	

	
	
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
	}


	


	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF1, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF2, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF3, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF4, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	

	
	rf_io_tx_4();		
}

int ack_num = 0;

void rf_send_ack_packet(int slot)
{
	int32_t i = 72*40;
	
	while(i--);
	
	if(s_sApAckPacket[slot].ulSlotBm >0) //有slot事件处理
	{
		
		ack_num++;
	//需要确定包序列号	     
		s_sApAckPacket[slot].sPhr.ucSerNr++;

		if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
		}
		if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
		}
		if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
		}
		if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
		}


		


		if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF1, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);
		}
		if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF2, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF3, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF4, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		

		
		rf_io_tx_4();		
 
		sprintf(gprs_debug_buff,"send_ack:s=%d slot=%d %X ack_num=%d\r\n",systerm_info.slot/get_slot_num(),slot,s_sApAckPacket[slot].ulSlotBm,ack_num);
		debug_uart_send_string(gprs_debug_buff);
		
		s_sApAckPacket[slot].ulSlotBm=0;
		uiIndex[slot]=0;		
		memset(s_sApAckPacket[slot].aucSerNr,0,sizeof(s_sApAckPacket[slot].aucSerNr));  //2013 7/25 lhj
			 

	}
}


void HAL_SYSTICK_Callback(void)
{

	systerm_info.slot++;

	if(ee_task.read_write > 0)   //有读写ee的任务
	{
		ee_task.timeout++;
	}

	if((systerm_info.slot%512) == 0) //gprs使用秒标志
	{
		gprs_sec_flag += 1;
		rf_sec_flag +=1;
	}
	
	if(systerm_info.enable_rf == 0)
		return;
	
	if((systerm_info.slot%64) == 0)  //发送同步包
		rf_send_syn_packet();

	if((systerm_info.slot%get_slot_num()) == 33)  //发送升级包
		rf_send_update_packet();	
		
	if((systerm_info.slot%get_slot_num()) == 2)  //发送ack包
		rf_send_ack_packet(0);
	if((systerm_info.slot%get_slot_num()) == 32)  //发送ack包
		rf_send_ack_packet(1);	
	if((systerm_info.slot%get_slot_num()) == 66)  //发送ack包
		rf_send_ack_packet(2);	
	if((systerm_info.slot%get_slot_num()) == 96)  //发送ack包
		rf_send_ack_packet(3);	
	
}







void clear_recode()
{
	int i = 0;
	for(i=0;i<256;i++)
	{
		sensor_rp_data_recode[i].id = 0;
		sensor_rp_data_recode[i].syn = 0;
	}
	
}


int check_recode_data_if_repeat(uint16_t id,uint8_t syn)
{
	int i = 0;
	for(i=0;i<256;i++)
	{
		if(sensor_rp_data_recode[i].id == id)
		{
			if(sensor_rp_data_recode[i].syn == syn)
				return -1;
		}
		if(sensor_rp_data_recode[i].id == 0)
		{
			sensor_rp_data_recode[i].id = id;
			sensor_rp_data_recode[i].syn = syn;
			return 0;
		}
	}
}



void make_data_to_n1(int index)
{
	
	g_sSnnDataMessage.basePhr.sPhr.ucLength=0x11+(rf_rx_buff[index][0]);
	
	g_sSnnDataMessage.lTimestamp=(u32_t)0u;
	
	g_sSnnDataMessage.sTime = systerm_info.slot%get_slot_num();			


	g_sSnnDataMessage.ucLqi= (rf_rx_buff[index][rf_rx_buff[index][0]] & 0x7F);//链路质量
			 
	g_sSnnDataMessage.ucRssi= rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76;      //信号强度 
			 
	g_sSnnDataMessage.ucOffset=0x11u;       
	
	memcpy(&g_sSnnDataMessage.ucLoadData[0],(char *)&rf_rx_buff[index][0],((rf_rx_buff[index][0]) & 0x3F));	

	insert_to_n1_buff((unsigned char *)&g_sSnnDataMessage,g_sSnnDataMessage.basePhr.sPhr.ucLength-1,AP_RF_DATA);  //发给SM 2013 0824 mod lhj 应答机制	
	

}

void make_ack(uint8_t packet_syn)
{
	ApPacketsetting(systerm_info.slot%get_slot_num(),packet_syn);
}


void rf_rx_data_handle(int index)
{
	SNP_PHR_t *ptr = (SNP_PHR_t *)&rf_rx_buff[index];

	SNP_SYNC_PACKET_tt	*ptr_syn = (SNP_SYNC_PACKET_tt *)&rf_rx_buff[index];
	SNP_STATE_PACKET_RP_t	*ptr_rp_stat = (SNP_STATE_PACKET_RP_t *)&rf_rx_buff[index];
	SNP_STATE_PACKET_SENSOR_t *ptr_s_stat = (SNP_STATE_PACKET_SENSOR_t *)&rf_rx_buff[index];
	SNP_SEN_MODE_B_PACKET_t *ptr_s_event = (SNP_SEN_MODE_B_PACKET_t *)&rf_rx_buff[index];
	
	uint8_t slot = systerm_info.slot%get_slot_num();
	
	if((rf_rx_buff[index][rf_rx_buff[index][0]] & 0x80 )!= 0x80)
		return;
	
	if(ptr->uiFcf != 0x4180)
		return;
	 
	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE)
	{
		if((ptr_s_stat->uiSlot & 0x00ff)!= slot && ptr_s_stat->ucVolt !=0)
			return;
	}
	if(ptr->ucType == SNP_PACKET_TYPE_EVENT)
	{
		if((ptr_s_event->slot & 0x00ff) != slot)
			return;
	}	
	if(ptr->ucType == SNP_PACKET_TYPE_RP_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE)
	{
		if((ptr_rp_stat->uiSlot & 0x00ff) != slot && ptr_rp_stat->reserve != 0)
			return;
	}	
	
	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_STATE ||ptr->ucType == SNP_PACKET_TYPE_EVENT
		|| ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE|| ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE) //sensor状态包 事件包 rp 状态包
	{

		if(check_recode_data_if_repeat(ptr->uiDevId,ptr->ucSerNr)==0)
		{
			sprintf(gprs_debug_buff,"receive_rf_data:id=%04X type=%d syn=%d s=%d slot=%d\r\n",ptr->uiDevId,ptr->ucType,ptr->ucSerNr,systerm_info.slot/get_slot_num(),systerm_info.slot%get_slot_num());
			debug_uart_send_string(gprs_debug_buff);			
			make_data_to_n1(index);
			make_ack(ptr->ucSerNr);
			
			if(ptr->ucType == SNP_PACKET_TYPE_EVENT) //事件包
				debug_insert_sensor_event(ptr->uiDevId,ptr->ucSerNr,rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76,slot,(ptr_s_event->slot)>>8);
			
			if(ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE) 
				get_s_rp_input_update_stat(ptr->uiDevId,ptr_s_stat->uiSubData);
			if(ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE )
				get_s_rp_input_update_stat(ptr->uiDevId,ptr_rp_stat->uiSubData);
			
/*			if(ptr->uiDevId == 0x0273)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 0x17 || ptr_rp_stat->uiGrade != 1){
				sensor_rp_param.ParaFram.uiPoll = 0x0273;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x14;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 0x17;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}
			if(ptr->uiDevId == 0x0373)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 0xb || ptr_rp_stat->uiGrade != 2){
				sensor_rp_param.ParaFram.uiPoll = 0x0373;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x5;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 2;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 0xb;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}*/
			if(ptr->uiDevId == 0x3412)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 15 || ptr_rp_stat->uiGrade != 1){
				sensor_rp_param.ParaFram.uiPoll = 0x3412;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x1550;
				sensor_rp_param.ParaFram.uiSlotStateM = 0X02AA;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 15;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			/*if(ptr->uiDevId == 0x3422)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 15 || ptr_rp_stat->uiGrade != 1){
				sensor_rp_param.ParaFram.uiPoll = 0x3422;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x1550;
				sensor_rp_param.ParaFram.uiSlotStateM = 0X02AA;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 5;
				sensor_rp_param.ParaFram.paraB.uimySlot = 15;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}				
			if(ptr->uiDevId == 0x2143)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 15 || ptr_rp_stat->uiGrade != 2){
				sensor_rp_param.ParaFram.uiPoll = 0x2143;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x8554;
				sensor_rp_param.ParaFram.uiSlotStateM = 0X00AA;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 2;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 15;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x7856)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff)  != 15 || ptr_rp_stat->uiGrade != 3){
				sensor_rp_param.ParaFram.uiPoll = 0x7856;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0xA155;
				sensor_rp_param.ParaFram.uiSlotStateM = 0X002A;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 3;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 15;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}				
			if(ptr->uiDevId == 0x0173)
			{
				if((ptr_rp_stat->uiSlot & 0x00ff) != 27 || ptr_rp_stat->uiGrade != 1){
				sensor_rp_param.ParaFram.uiPoll = 0x0173;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0xFFFF;
				sensor_rp_param.ParaFram.uiSlotStateM = 0xFFFF;
				sensor_rp_param.ParaFram.uiSlotStateH = 0xFFFF;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 5;
				sensor_rp_param.ParaFram.paraB.uimySlot = 27;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0X0F;
				}
			}						
			if(ptr->uiDevId == 0x01a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x01a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 18;
				sensor_rp_param.ParaFram.paraB.uimySlot = 63;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}
			if(ptr->uiDevId == 0x02a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x02a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 59;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x03a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x03a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 55;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}			
			if(ptr->uiDevId == 0x04a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x04a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 51;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}		
			if(ptr->uiDevId == 0x05a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x05a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 47;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}		
			if(ptr->uiDevId == 0x06a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x06a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 31;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x07a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x07a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 27;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x08a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x08a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 23;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x09a0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x09a0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 19;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x0aa0 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x0aa0;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 4;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 15;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	
			if(ptr->uiDevId == 0x5e03 && ptr_s_stat->sPhr.ucType == 6)
			{
				if((ptr_s_stat->uiSlot & 0x00ff) == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x5e03;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 16;
				sensor_rp_param.ParaFram.paraB.uimySlot = 20;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}	*/			
		}
	}
}



uint8_t get_slot_num()
{
	uint8_t lowLatency;
	uint8_t transmitInterval;
	
	lowLatency = (ap_param.ap_syn_param[5] & 0x10);

  transmitInterval = ((ap_param.ap_syn_param[0] >> 4) & 0x07);
       
	if(lowLatency==0)  //     
			 transmitInterval += 2;
	else
			transmitInterval = (2 - transmitInterval) & 0x01;
	
	if((transmitInterval+1)>=1u && transmitInterval<=6)
		 transmitInterval=1<<transmitInterval;

	return transmitInterval*16;
}




void ApPacketsetting(u16_t uiCurSlotNr,u8_t ucCurPacketerNr)
{
    u32_t retValue=0;
    u16_t i;      
	
	
    if(uiCurSlotNr>32 && uiCurSlotNr<64)//slot from 6 to 32；36 to 62 
    {
			s_sApAckPacket[1].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
			s_sApAckPacket[1].sPhr.ucType = SNP_PACKET_TYPE_ACK;
			s_sApAckPacket[1].sPhr.ucSensorMode = 0;
			s_sApAckPacket[1].sPhr.uiDevId = ap_param.band_id;
			s_sApAckPacket[1].sPhr.uiFcf = 0x0080;
			
			
       i=uiCurSlotNr-33;
       retValue=(u32_t)1u<<i;  
       if((s_sApAckPacket[1].ulSlotBm & retValue) ==0)
       {
         s_sApAckPacket[1].ulSlotBm |=retValue;
         s_sApAckPacket[1].aucSerNr[(uiIndex[1]&0x0F)]=ucCurPacketerNr;         
         uiIndex[1]++;
       }
    }
    else  if(uiCurSlotNr<32)//时间槽如果落在小于36的区间则回复检测器s_sApAckPacket[0]
    {
			s_sApAckPacket[0].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
			s_sApAckPacket[0].sPhr.ucType = SNP_PACKET_TYPE_ACK;
			s_sApAckPacket[0].sPhr.ucSensorMode = 0;
			s_sApAckPacket[0].sPhr.uiDevId = ap_param.band_id;
			s_sApAckPacket[0].sPhr.uiFcf = 0x0080;			
			
      i=uiCurSlotNr-3;  
      retValue =(u32_t)1u<<i; 
      if((s_sApAckPacket[0].ulSlotBm & retValue) ==0)
      {
        s_sApAckPacket[0].ulSlotBm |=retValue;
        s_sApAckPacket[0].aucSerNr[(uiIndex[0]& 0x0F)]=ucCurPacketerNr;        
        uiIndex[0]++;
      }
    }   
		
	
		
 		else if((uiCurSlotNr & 0X7F)>96)//slot from 70 to 96；100 to 126 
     {
			 s_sApAckPacket[3].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
				s_sApAckPacket[3].sPhr.ucType = SNP_PACKET_TYPE_ACK;
				s_sApAckPacket[3].sPhr.ucSensorMode = 0;
				s_sApAckPacket[3].sPhr.uiDevId = ap_param.band_id;
				s_sApAckPacket[3].sPhr.uiFcf = 0x0080;
			 
        i=uiCurSlotNr-97;
        retValue=(u32_t)1u<<i;  
        if((s_sApAckPacket[3].ulSlotBm & retValue) ==0)
        {
          s_sApAckPacket[3].ulSlotBm |=retValue;
          s_sApAckPacket[3].aucSerNr[(uiIndex[3]&0x0F)]=ucCurPacketerNr;         
          uiIndex[3]++;
        }
     }
     else if((uiCurSlotNr & 0X7F)<96 && (uiCurSlotNr & 0X7F)>66)
     {
			 s_sApAckPacket[2].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
				s_sApAckPacket[2].sPhr.ucType = SNP_PACKET_TYPE_ACK;
				s_sApAckPacket[2].sPhr.ucSensorMode = 0;
				s_sApAckPacket[2].sPhr.uiDevId = ap_param.band_id;
				s_sApAckPacket[2].sPhr.uiFcf = 0x0080;
			 
       i=uiCurSlotNr-67;  
       retValue =(u32_t)1u<<i; 
       if((s_sApAckPacket[2].ulSlotBm & retValue) ==0)
       {
         s_sApAckPacket[2].ulSlotBm |=retValue;
         s_sApAckPacket[2].aucSerNr[(uiIndex[2]& 0x0F)]=ucCurPacketerNr;        
         uiIndex[2]++;
       }
     }   
		
		
}




























