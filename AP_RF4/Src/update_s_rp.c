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


//��¼�͸����Ѿ��������ģʽ��sensor����rp
void get_s_rp_input_update_stat(uint16_t dev_id, uint16_t flash_times)
{
	uint8_t i = 0;
	
	if(update_s_rp_manage.dev_num>=64)
		return;

//	if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq == 0)
//		update_s_rp_manage.upadate_s_rp_enable = 1;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)     //����豸�Ѿ��ڱ����ˣ������豸����
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_id == dev_id)
		{
			update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = flash_times;
			return;
		}
	}
	
	//�豸û���ڼ�¼��
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_id = dev_id;
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_packet_seq = flash_times;
	update_s_rp_manage.dev_num++;
	

}


int32_t check_update_s_rp_list(void)
{
	int i = 0;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq < update_s_rp_manage.now_upadate_packet_seq) //�豸�ϴ��İ����С��AP�İ���ţ�˵���豸�Ѿ�����
		{
			if(update_s_rp_manage.dev_num>1) //����һ���豸 ��ĩβ���豸��������
			{
				update_s_rp_manage.update_s_rp_list[i].dev_id = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_id ;
				update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_packet_seq ;
				update_s_rp_manage.dev_num--;
			}
			else //����1���豸 ֱ������
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
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq <= update_s_rp_manage.now_upadate_packet_seq) //�豸�ϴ��İ����С��AP�İ���ţ�˵���豸�Ѿ�����
		{
				return -1; //��һ��û�����ack
		}
	}
	
	return 0;  // ���еĶ������ack
}



/*
	������
	���ܣ� ����ʱ��۵��øú��� �������͸���RP SENSOR�Ĺ̼�

*/
void rf_send_update_packet(void)
{
	if(update_s_rp_manage.upadate_s_rp_enable !=0) //����Ѿ�ʹ�ܸ���
	{
		if(update_s_rp_manage.dev_num == 0)     //����Ѿ�ʹ�ܸ��¡�����û���豸���룬ȡ������ʹ��
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.upadate_s_rp_enable = 0;
			update_s_rp_manage.now_upadate_packet_seq = 0;
			return;
		}
		if(check_update_s_rp_list()==0) //ȫ���յ����롡������һ��
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq++;
		}
		//����һ������
		rf_send_updata_packet(update_s_rp_manage.upadate_s_rp_enable,update_s_rp_manage.now_upadate_packet_seq);
		update_s_rp_manage.now_send_times++;   //���ʹ�������
		
		if(update_s_rp_manage.now_send_times>ONE_PACKET_SEND_TIMES_MAX)    //����ָ�����ʹ��������Ͱ���Ҫ����
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
















