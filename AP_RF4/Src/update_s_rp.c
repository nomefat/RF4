#include "stm32f4xx_hal.h"
#include "snp.h"
#include "typedef_struct.h"
#include "ap_param.h"
#include "rf_hal.h"
#include "string.h"
#include "to_n1.h"
#include "eeprom.h"
#include "debug.h"
#include "update_s_rp.h"
#include "typedef_struct.h"





#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02




struct_update_s_rp_manage update_s_rp_manage ;


void rf_send_updata_packet(uint8_t s_or_rp,uint16_t flash_seq);


//记录和更新已经进入更新模式的sensor或者rp
void get_s_rp_input_update_stat(uint16_t dev_id, uint16_t flash_times)
{
	uint8_t i = 0;
	
	if(update_s_rp_manage.dev_num>=64)
		return;

//	if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq == 0)
//		update_s_rp_manage.upadate_s_rp_enable = 1;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)     //如果设备已经在表中了，更新设备参数
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_id == dev_id)
		{
			update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = flash_times;
			return;
		}
	}
	
	//设备没有在记录中
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_id = dev_id;
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_packet_seq = flash_times;
	update_s_rp_manage.dev_num++;
	

}


int32_t check_update_s_rp_list(void)
{
	int i = 0;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq < update_s_rp_manage.now_upadate_packet_seq) //设备上传的包序号小于AP的包序号，说明设备已经掉队
		{
			if(update_s_rp_manage.dev_num>1) //大于一个设备 把末尾的设备放在这里
			{
				update_s_rp_manage.update_s_rp_list[i].dev_id = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_id ;
				update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_packet_seq ;
				update_s_rp_manage.dev_num--;
			}
			else //等于1个设备 直接清零
			{
				update_s_rp_manage.update_s_rp_list[i].dev_id = 0 ;
				update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = 0 ;				
				update_s_rp_manage.dev_num = 0;
				update_s_rp_manage.now_upadate_packet_seq = 0;
				update_s_rp_manage.now_send_times = 0;
			}
		}
	}
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq <= update_s_rp_manage.now_upadate_packet_seq) //设备上传的包序号小于AP的包序号，说明设备已经掉队
		{
				return -1; //有一个没有完成ack
		}
	}
	
	return 0;  // 所有的都完成了ack
}



/*
	函数：
	功能： 更新时间槽调用该函数 ，负责发送更新RP SENSOR的固件

*/
void rf_send_update_packet(void)
{
	if(update_s_rp_manage.upadate_s_rp_enable !=0) //如果已经使能更新
	{
		if(update_s_rp_manage.dev_num == 0)     //如果已经使能更新　但是没有设备进入，取消更新使能
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.upadate_s_rp_enable = 0;
			update_s_rp_manage.now_upadate_packet_seq = 0;
			return;
		}
		if(check_update_s_rp_list()==0) //全部收到ａｃｋ　进行下一包
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq++;
		}
		//发送一包数据
		rf_send_updata_packet(update_s_rp_manage.upadate_s_rp_enable,update_s_rp_manage.now_upadate_packet_seq);
		update_s_rp_manage.now_send_times++;   //发送次数增加
		
		if(update_s_rp_manage.now_send_times>ONE_PACKET_SEND_TIMES_MAX)    //大于指定发送次数　发送包需要增加
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq++;
		}
	}
}





void enable_sensor_update()
{
	update_s_rp_manage.now_send_times = 0;
	update_s_rp_manage.upadate_s_rp_enable = FIRM_RP;
	update_s_rp_manage.now_upadate_packet_seq = 0;	
}


void enable_rp_update()
{
	update_s_rp_manage.now_send_times = 0;
	update_s_rp_manage.upadate_s_rp_enable = FIRM_SENSOR;
	update_s_rp_manage.now_upadate_packet_seq = 0;	
	
}
















