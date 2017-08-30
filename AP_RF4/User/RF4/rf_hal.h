#ifndef _RF_HAL_H_
#define _RF_HAL_H_








#define RF1 &hspi4
#define RF2 &hspi1
#define RF3 &hspi5
#define RF4 &hspi3


#define RF_WORK                         0
#define ENABLE_NOMODULATE_CARRIER       1
#define ENABLE_MODULATE_CARRIER         2
#define RF_IDLE                         3

#define RF_POWER_ON    1
#define RF_POWER_OFF   0
#define RF_POWER_WORK  2

typedef enum 
{	
		RF_SPI_ERROR  =  -1,
		RF_NO_GET_0X84 = -2,
		RF_REG_INIT_OK =  1
}RF_init_stat_typedef;
	
	
	
typedef struct _rf_stat
{
	char rf_power_stat;
	RF_init_stat_typedef reg_init_stat;
	unsigned char mode;
}struct_rf_stat;





void rf_io_tx_4();



























#endif


