#ifndef _UPDATE_S_RP_H_
#define _UPDATE_S_RP_H_

#include "stm32f4xx_hal.h"






#define S_RP_COUNT 64                       //������64���豸ͬʱ����
#define ONE_PACKET_SEND_TIMES_MAX 50        //ͬһ�����°��ط�����  

typedef struct _update_s_rp_list
{
	uint16_t dev_id;                      //�豸ID
	uint16_t dev_packet_seq;              //�豸���µ��ڼ�����
}struct_update_s_rp_list;            



typedef struct _update_s_rp_manage
{
	uint8_t upadate_s_rp_enable;
	uint16_t now_upadate_packet_seq;    //�����������ڼ���
	uint16_t now_send_times;            //һ�����������͵Ĵ���
	uint8_t dev_num;
	struct_update_s_rp_list update_s_rp_list [S_RP_COUNT];  //��������״̬��S����RP

}struct_update_s_rp_manage;




void enable_sensor_update();
void enable_rp_update();


#endif


