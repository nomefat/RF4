#include "stm32f4xx_hal.h"
#include "ap_param.h"
#include "eeprom.h"
#include "string.h"









extern uint8_t ee_data_write[256];

extern uint8_t ee_data_read[256];


struct_ap_param ap_param;


struct_sensor_rp_param sensor_rp_param ;








void init_ap_param(void)
{
	ap_param.ap_id = 0x1111;
	ap_param.ap_version = AP_VERSION;
	ap_param.band_id = 0x1113;
	ap_param.ap_channel = 0x05050505;
	ap_param.ap_syn_param[0] = 0x10;
	ap_param.gprs_server_ip = (74<<24)|(83<<16)|(239<<8)|219;
	ap_param.gprs_server_port = 40005;
}


void read_ap_param(void)
{
	int32_t delay;
	int32_t times = 0;
	
	while(1)
	{
		HAL_GPIO_TogglePin(general_led_2_GPIO_Port,general_led_2_Pin);
		if(HAL_OK==ee_read_no(EEPROM_AP_PARAM_BEGIN,sizeof(struct_ap_param)))
		{
			if(ee_data_read[0] != 0xff && ee_data_read[0] != 0)
			{
				memcpy(&ap_param,ee_data_read,sizeof(struct_ap_param));
				ap_param.ap_version = AP_VERSION;
				return;
			}
			else
			{
				init_ap_param();
				memcpy(ee_data_write,&ap_param,sizeof(struct_ap_param));
				if(HAL_OK==ee_write_no(EEPROM_AP_PARAM_BEGIN,sizeof(struct_ap_param)))
				{					
					continue;
				}
			}
		}
		delay = 100000;
		while(delay--);
		times++;
	}
}

