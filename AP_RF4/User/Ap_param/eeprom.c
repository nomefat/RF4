#include "stm32f4xx_hal.h"





/*
1.eeprom ��ַ����
	0-1024 ��  AP_param
	1024-10*1024 dev_list
	10*1024 -110*1204 rp_firmware
	110*1024-210*1024 sensor_firmware

*/




#define EEPROM_AP_PARAM_BEGIN 0                     //AP������ʼ��ַ
#define EEPROM_DEV_LIST_BEGIN 1024                  //�豸����ʼ��ַ
#define EEPROM_RP_FIRMWARE_BEGIN 10*1024            //RP�̼���ʼ��ַ
#define EEPROM_SENSOR_FIRMWARE_BEGIN 110*1024       //sensor�̼���ʼ��ַ
#define EEPROM_ELSE_BEGIN 210*1024                  //������ʼ��ַ













