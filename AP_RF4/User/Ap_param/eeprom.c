#include "stm32f4xx_hal.h"





/*
1.eeprom 地址分配
	0-1024 ：  AP_param
	1024-10*1024 dev_list
	10*1024 -110*1204 rp_firmware
	110*1024-210*1024 sensor_firmware

*/




#define EEPROM_AP_PARAM_BEGIN 0                     //AP参数起始地址
#define EEPROM_DEV_LIST_BEGIN 1024                  //设备表起始地址
#define EEPROM_RP_FIRMWARE_BEGIN 10*1024            //RP固件起始地址
#define EEPROM_SENSOR_FIRMWARE_BEGIN 110*1024       //sensor固件起始地址
#define EEPROM_ELSE_BEGIN 210*1024                  //其他起始地址













