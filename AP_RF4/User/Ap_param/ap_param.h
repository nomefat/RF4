#ifndef _AP_PARAM_H_
#define _AP_PARAM_H_


#include "stm32f4xx_hal.h"


#define AP_VERSION  0X0005


#pragma pack(1)



typedef struct _ap_param{

	uint16_t ap_id;                                                           //     N1ֻ��
	uint16_t ap_version;                                                      //     N1ֻ��
	uint16_t rp_version;                //eeprom ��rp�̼��汾��                       N1ֻ��
	uint16_t sensor_version;						   //eeprom ��sensor�̼��汾��                N1ֻ��	
	uint16_t band_id;	
	uint32_t ap_channel;
	uint32_t gprs_server_ip;
	uint16_t gprs_server_port;
	uint8_t ap_syn_param[6];


}struct_ap_param;



typedef struct{
	
 uint8_t uiCmd;  //����2-�������У׼ 3-�����ģʽ����  11 -����ȫ���� 
 uint16_t uiPoll;   //Ŀ��ID
 
 uint16_t uiBindId; //��ID
 struct{

  uint16_t uimySlot:8,     //8λ-����ʱ���
     uiSlotStateE:8; //8λ-����ʱ�����չ
  
 }paraB; 
 uint16_t uiSlotStateL;//����ʱ��۵�16
 uint16_t uiSlotStateM;//����ʱ�����16
 uint16_t uiSlotStateH;//����ʱ��۸�16
 struct{  
  uint8_t uiGrade:3, //����ͬ��������0-3
    uiChannel:5;//���õ�ͨ��0-31
 }paraA;
 
}rp_param;




typedef struct{

 uint8_t ucSensorMode;
 rp_param ParaFram;
 
}struct_sensor_rp_param;



typedef struct _dev_list{

	uint16_t bind_id;
	uint16_t ap_channel;
	



}struct_dev_list;



#pragma pack()


extern struct_ap_param ap_param;


void init_ap_param(void);



#endif


