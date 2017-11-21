#ifndef FLASH_H_
#define FLASH_H_


#include "stm32f4xx_hal.h"





#define FLASH_AP_PARAM_BEGIN_1          0x08100000                     //AP������ʼ��ַ1
#define FLASH_AP_PARAM_BEGIN_2          0x08104000                     //AP������ʼ��ַ2


#define FLASH_RP_FIRMWARE_BEGIN       0x08120000         //RP�̼���ʼ��ַ
#define FLASH_SENSOR_FIRMWARE_BEGIN   0x08140000     //sensor�̼���ʼ��ַ




typedef struct _flash_head_crc{

	unsigned int head;
	unsigned int crc;

}struct_flash_head_crc;





void read_ap_param_flash(void);
void write_ap_param_flash(void);
int32_t write_bin_flash(uint32_t address,uint8_t *pdata,uint32_t size);







#endif


