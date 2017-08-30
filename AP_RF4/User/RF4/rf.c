#include "stm32f4xx_hal.h"
#include "snp.h"
#include "typedef_struct.h"
#include "ap_param.h"
#include "rf_hal.h"
#include "string.h"
#include "to_n1.h"


SNP_SYNC_PACKET_t syn_packet;
SNP_AP_ACK_PACKET_t ack_packet;


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

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;



void ApPacketsetting(u16_t uiCurSlotNr,u8_t ucCurPacketerNr);



int test;


void clear_recode();
uint8_t get_slot_num();


void rf_send_syn_packet(void)
{
  memcpy( (uint8_t *)&syn_packet.sensor_param[0], (uint8_t *)ap_param.ap_syn_param, 6 );
	syn_packet.sPhr.ucSize = sizeof( SNP_SYNC_PACKET_t)+1;
	syn_packet.sPhr.uiFcf = 0x0080;
	syn_packet.sPhr.ucSerNr++;
	syn_packet.sPhr.ucType = SNP_PACKET_TYPE_WORK;
	syn_packet.sPhr.uiDevId = ap_param.band_id + 0x1113;
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
			syn_packet.uiBindId = ap_param.band_id + 0x1113;
			syn_packet.sPhr.ucSensorMode = sensor_rp_param.ucSensorMode;
			sensor_rp_param.ParaFram.uiCmd = 0;
			syn_packet.sPhr.ucType |= 1<<4;
		}
		else
		{
			memset(&syn_packet.uiCmd,0,14);
			syn_packet.uiBindId = ap_param.band_id + 0x1113;
			syn_packet.sPhr.ucSensorMode = 0;
			syn_packet.sPhr.ucType &= ~(1<<4);
		}
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
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f);
		rf_write_buff(RF1, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5);
		rf_write_buff(RF2, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<6);
		rf_write_buff(RF3, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5) |(1<<6);
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
}


void rf_send_updata_packet()
{
	test = 2;	
}

void rf_send_ack_packet(int slot)
{

	if(s_sApAckPacket[slot].ulSlotBm >0) //有slot事件处理
	{
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
 
		sprintf(gprs_debug_buff,"send_ack:slot=%d %X\r\n",slot,s_sApAckPacket[slot].ulSlotBm);
		debug_uart_send_string(gprs_debug_buff);
		
		s_sApAckPacket[slot].ulSlotBm=0;
		uiIndex[slot]=0;		
		memset(s_sApAckPacket[slot].aucSerNr,0,sizeof(s_sApAckPacket[slot].aucSerNr));  //2013 7/25 lhj
			 

	}
}


void HAL_SYSTICK_Callback(void)
{

	systerm_info.slot++;


	if((systerm_info.slot%512) == 0) //gprs使用秒标志
	{
		gprs_sec_flag += 1;
		rf_sec_flag +=1;
	}
	if((systerm_info.slot%64) == 0)  //发送同步包
		rf_send_syn_packet();

	if((systerm_info.slot%64) == 1)  //发送升级包
		rf_send_updata_packet();	

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

	insert_to_n1_buff((unsigned char *)&g_sSnnDataMessage,g_sSnnDataMessage.basePhr.sPhr.ucLength,AP_RF_DATA);  //发给SM 2013 0824 mod lhj 应答机制	
	

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
	
	if(ptr->uiFcf != 0x4180)
		return;
	
//	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE)
//	{
//		if(ptr_s_stat->uiSlot != slot)
//			return;
//	}
	if(ptr->ucType == SNP_PACKET_TYPE_EVENT)
	{
		if(ptr_s_event->slot != slot)
			return;
	}	
//	if(ptr->ucType == SNP_PACKET_TYPE_RP_STATE)
//	{
//		if(ptr_rp_stat->uiSlot != slot)
//			return;
//	}	
	
	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_STATE ||ptr->ucType == SNP_PACKET_TYPE_EVENT) //sensor状态包 事件包 rp 状态包
	{

		if(check_recode_data_if_repeat(ptr->uiDevId,ptr->ucSerNr)==0)
		{
			sprintf(gprs_debug_buff,"receive_rf_data:id=%04X type=%d syn=%d slot=%d\r\n",ptr->uiDevId,ptr->ucType,ptr->ucSerNr,systerm_info.slot%get_slot_num());
			debug_uart_send_string(gprs_debug_buff);			
			make_data_to_n1(index);
			make_ack(ptr->ucSerNr);
			
			if(ptr->uiDevId == 0x0273)
			{
				if(ptr_rp_stat->uiSlot != 0x1b && ptr_rp_stat->uiGrade != 1){
				sensor_rp_param.ParaFram.uiPoll = 0x0273;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0x81;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 1;
				sensor_rp_param.ParaFram.paraA.uiChannel = 11;
				sensor_rp_param.ParaFram.paraB.uimySlot = 0x1b;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 1;
				}
			}
			
			if(ptr->uiDevId == 0x5e03 && ptr_s_stat->sPhr.ucType == 6)
			{
				if(ptr_s_stat->uiSlot == 0){
				sensor_rp_param.ParaFram.uiPoll = 0x5e03;
				sensor_rp_param.ParaFram.uiCmd = 11;
				sensor_rp_param.ParaFram.uiBindId = 0;
				sensor_rp_param.ParaFram.uiSlotStateL = 0;
				sensor_rp_param.ParaFram.uiSlotStateM = 0;
				sensor_rp_param.ParaFram.uiSlotStateH = 0;
				sensor_rp_param.ParaFram.paraA.uiGrade = 2;
				sensor_rp_param.ParaFram.paraA.uiChannel = 10;
				sensor_rp_param.ParaFram.paraB.uimySlot = 21;
				sensor_rp_param.ParaFram.paraB.uiSlotStateE = 0;
				}
			}
			
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




























