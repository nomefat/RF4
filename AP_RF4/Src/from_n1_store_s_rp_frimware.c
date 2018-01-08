#include "stm32f4xx_hal.h"
#include "string.h"
#include "ap_param.h"
#include "to_n1.h"
#include "rf_hal.h"
#include "eeprom.h"
#include "flash.h"
#include "from_n1.h"
#include "typedef_struct.h"

//�궨��ǰ�·������Ǹ��̼�
#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02


#define FIRM_HEAD_ERROR 0XFFFE
#define FIRM_LINES_ERROR 0XFFFD

extern struct_ap_param ap_param;
extern uint8_t ap_param_write_flash_flag ;

extern struct_systerm_info systerm_info;  



#pragma pack(1)

typedef struct _FW_HEADER_t_5xx
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[64u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
} FW_HEADER_t_5xx;

typedef struct _FW_HEADER_t_5xxRP
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[16u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
} FW_HEADER_t_5xxRP;
#pragma pack()


//���������Ĺ̼�ͷ ����flash�� ���ڼ���ʹ��
struct _my_firmware_head{
	unsigned char firmware_type;
	unsigned short fw_ver;   //�̼��汾
	unsigned short hw_ver;   //Ӳ���汾
	unsigned short packet_num;  //�̼����ȣ���λ����
}my_firmware_head;




typedef struct _n1_send_firmware_head
{
	uint16_t packet_count;
	uint16_t packet_syn;
	uint8_t data[1];
} struct_n1_send_firmware_head;






unsigned char firmware_head[38*5];       //����̼�ͷ �����鿴�̼��汾 rp sensor
unsigned char firmware_struct_head[32*5];  //��������ǰ4�����ݣ���һ���ṹ�壬�洢�˹̼���һЩ��Ϣ




uint16_t ee_write_packet_syn = 0;
uint32_t ee_write_addr_add = 0;

void read_firmware_rp_head();
void read_firmware_sensor_head();

FW_HEADER_t_5xxRP pheader_5xxRP;
FW_HEADER_t_5xx pheader_5xx;



//��ʮ�������ַ���ת��Ϊ����������
void strhex_to_hex(unsigned char *output,char *input,int len)
{
	char data[3];
		int i;
	data[2] = 0;

	
	for(i=0;i<len;i++)
	{
		data[0] = *input++;
		data[1] = *input++;
		sscanf(data,"%X",output++);
	}
}



/**
 * Func:  У��̼�������ÿ�еĺ�Ϊ0������У��
 * 
 * Param: �̼���ŵĵ�ַ��
 *
 * Return��0��У׼�ɹ�  -1��У׼ʧ��
 */
int verify_firmware(int addr)
{
	struct _my_firmware_head * phead = (struct _my_firmware_head * )addr;
	int i,j;
	unsigned char *pdata;
	unsigned char check;
	
	for(i=0;i<phead->packet_num;i++)
	{
		
		check = 0;
		pdata = (unsigned char *)(addr+sizeof(struct _my_firmware_head)+i*38);
		for(j=0;j<37;j++)
		{
			check +=*pdata++;
		}
		if(check!=0)
			return -1;
	}
	return 0;

}


void check_firmware_header()
{
	unsigned short *ptr = (unsigned short *)firmware_struct_head;
	int i = 0;
	volatile int sum = 0;
	FW_HEADER_t_5xx *sptr = (FW_HEADER_t_5xx *)firmware_struct_head;
	
	
	for(i=0;i<sptr->uiSize/2;i++)
	{
		sum += *ptr++;
	}

	sum -= sptr->ulCheckSum & 0xffff;
	sum -= sptr->ulCheckSum >> 16;

}



/*

	func : ��N1���ռ������rp�Ĺ̼����򣬷ֱ�洢��AP1��ͬ��flash��ַ����







*/
void receive_rp_sensor_firmware(struct_ap_n1_protocol *ptr,int who)
{

	unsigned char ack[7];                //��N1����ack
	int i;
	unsigned char check = 0;             //У�����ݰ�
	static int firmware_type = 0;        //�̼����;����洢λ�ã�rp or sensor��
//	static int packet_count = 0;         //������ ����ָʾ�洢���ڼ����ˡ�
	static int check_error = 0;
	unsigned char cmd = 0;
	FW_HEADER_t_5xx *pheader_5xx;
	FW_HEADER_t_5xxRP *pheader_5xxRP;
	
	struct _my_firmware_head  *flash_head; 
	uint16_t updata_FwVer;
	uint16_t updata_HwVer;
	static uint32_t rtc_time = 0;
	struct_n1_send_firmware_head *p_n1_send_firmware_head;
		
	p_n1_send_firmware_head = (struct_n1_send_firmware_head *)&ptr->data[0];

	uint32_t begin_address = 0;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;	
	
	
//#ifdef _DEBUG
//	//if(cmd_firmware_data->sequence==0xffff)return;
//	sprintf(debug_,"receive packet_num=%d   error=%d \r",cmd_firmware_data->sequence,check_error);
//	debug(debug_);
//#endif

	if(who == N1_SEND_RP_FIRMWARE)   //rp�̼�
	{

		begin_address = FLASH_RP_FIRMWARE_BEGIN;
		EraseInit.Sector = FLASH_SECTOR_17;
		cmd = AP_SEND_RP_FIRMWARE;
	}
	else if(who == N1_SEND_S_FIRMWARE)   //sensor�̼�
	{

		begin_address = FLASH_SENSOR_FIRMWARE_BEGIN;	
		EraseInit.Sector = FLASH_SECTOR_18;		
		cmd = AP_SEND_S_FIRMWARE;
	}



	if(p_n1_send_firmware_head->data[0] != 'l' && p_n1_send_firmware_head->packet_syn != 0xffff)   //ÿһ�ж���l��ͷ�� �������Ľ�β�С�����
	{	
		i = i;
		return;
	}
	
	if(p_n1_send_firmware_head->packet_syn==109)
		i=i;
	
	if(ee_write_packet_syn != p_n1_send_firmware_head->packet_syn)  //����N1�����Ĳ�������Ҫ�İ���� ��N1��Ҫ��Ҫ�İ����
	{
		  if(systerm_info.slot - rtc_time > 50) //�������һ�η��� ����100ms �Ż��������
			{
				rtc_time = systerm_info.slot;
				if(p_n1_send_firmware_head->packet_syn == 0) //����N1���Ƿ��͵�0��  ��ΪN1�������¿�ʼ����
				{
					ee_write_packet_syn = 0;
					ee_write_addr_add = 0;
					return;
				}
				p_n1_send_firmware_head->packet_syn =ee_write_packet_syn;
				insert_to_n1_buff(&ptr->data[0],4,cmd);					
			}
			return;
	}
	
	
	if(p_n1_send_firmware_head->packet_syn<5)   //ǰ4���ǹ̼�ͷ �����б���rp������sensor��149 or 5ϵ�У�
	{
		strhex_to_hex(firmware_head + p_n1_send_firmware_head->packet_syn*37,&(p_n1_send_firmware_head->data[1]),37);
		for(i=0;i<32+5;i++)
		{
			check += firmware_head[i+p_n1_send_firmware_head->packet_syn*37];
		}
		if(check !=0)               //error  ��У��û��ͨ��
		{	
			check_error++;		
			insert_to_n1_buff(&ptr->data[0],4,cmd);
			return;                          //У��ʧ�ܣ�������һ������ţ�����N1�ͻ��ط���һ����
		}
		
		if(p_n1_send_firmware_head->packet_syn == 0)  //��0�� ������Ӧ��flash
		{
			ee_write_addr_add = 0;
			__disable_irq() ;  //�����ж�
			HAL_FLASH_Unlock();
			EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
			EraseInit.Banks = FLASH_BANK_2;
			EraseInit.NbSectors = 1;		
			EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			HAL_FLASHEx_Erase(&EraseInit,&SectorError);
			HAL_FLASH_Lock();
			if(who == N1_SEND_RP_FIRMWARE)   //rp�̼�
			{
				ap_param.rp_version = FIRM_HEAD_ERROR;
			}
				else if(who == N1_SEND_S_FIRMWARE)   //sensor�̼�
			{
				ap_param.sensor_version = FIRM_HEAD_ERROR;	
			}
			__enable_irq() ; //�����ж�
		}

		if(0 == write_bin_flash(begin_address + ee_write_addr_add,firmware_head + p_n1_send_firmware_head->packet_syn*37,37)) //дflash�ɹ�
		{
				ee_write_addr_add += 37;
				ee_write_packet_syn++;
				p_n1_send_firmware_head->packet_syn +=1;
				insert_to_n1_buff(&ptr->data[0],4,cmd);	

		}
		else
		{
				p_n1_send_firmware_head->packet_syn = 0;
				insert_to_n1_buff(&ptr->data[0],4,cmd);			
				return;
		}		
		
		if(p_n1_send_firmware_head->packet_syn==5)
		{
			memcpy(firmware_struct_head,firmware_head+5,32);
			memcpy(firmware_struct_head+32,firmware_head+5+37,32);
			memcpy(firmware_struct_head+32*2,firmware_head+5+37*2,32);
			memcpy(firmware_struct_head+32*3,firmware_head+5+37*3,32);
			memcpy(firmware_struct_head+32*4,firmware_head+5+37*4,32);
			pheader_5xx = (FW_HEADER_t_5xx *)firmware_struct_head; 
			pheader_5xxRP = (FW_HEADER_t_5xxRP *)firmware_struct_head; 
			
			check_firmware_header();
			
			if(who == N1_SEND_RP_FIRMWARE)   //rp�̼�
			{
				if(pheader_5xxRP->ucCfgComb==0x03 || pheader_5xxRP->ucCfgComb==0x08)
					my_firmware_head.firmware_type = FIRM_SENSOR;
				else if(pheader_5xxRP->ucCfgComb==0x07)
					my_firmware_head.firmware_type = FIRM_RP;
				my_firmware_head.fw_ver = pheader_5xxRP->uiBuildNr;
				my_firmware_head.hw_ver = pheader_5xxRP->uiHwVer;
			}
			else if(who == N1_SEND_S_FIRMWARE) 
			{				
				if(pheader_5xx->ucCfgComb==0x03 || pheader_5xx->ucCfgComb==0x08)
					my_firmware_head.firmware_type = FIRM_SENSOR;
				else if(pheader_5xx->ucCfgComb==0x07)
					my_firmware_head.firmware_type = FIRM_RP;
				my_firmware_head.fw_ver = pheader_5xx->uiBuildNr;
				my_firmware_head.hw_ver = pheader_5xx->uiHwVer;				
			}
			

		}
		

		
	}

	else if(ee_write_packet_syn == p_n1_send_firmware_head->packet_syn)   //�����������Ҫ�İ���
	{
		strhex_to_hex(firmware_head,&(p_n1_send_firmware_head->data[1]),37);
		for(i=0;i<32+5;i++)
		{
			check += firmware_head[i];
		}
		if(check !=0)               //error  ��У��û��ͨ��
		{	
			check_error++;
			insert_to_n1_buff(&ptr->data[0],4,cmd);
//			debug("У�����");
			return;
		}
		
		if(0 == write_bin_flash(begin_address + ee_write_addr_add,firmware_head,37)) //дflash�ɹ�
		{
				ee_write_addr_add += 37;
				ee_write_packet_syn++;
				p_n1_send_firmware_head->packet_syn +=1;
				insert_to_n1_buff(&ptr->data[0],4,cmd);	

		}
		else
		{
				p_n1_send_firmware_head->packet_syn = 0;
				insert_to_n1_buff(&ptr->data[0],4,cmd);		
				return;
		}	


		if(p_n1_send_firmware_head->packet_syn == p_n1_send_firmware_head->packet_count) //�������
		{
			ee_write_packet_syn = 0;
			ee_write_addr_add = 0;
			if(who == N1_SEND_RP_FIRMWARE)
			{
				read_firmware_rp_head();
			}
			else if(who == N1_SEND_S_FIRMWARE)
			{
				read_firmware_sensor_head();
			}
			
		}


		
#ifdef _DEBUG
	//if(cmd_firmware_data->sequence==0xffff)return;
	sprintf(debug_,"send ack=%d %d %d %d   \r",ack[0],ack[1],ack[2],ack[3]);
	debug(debug_);
#endif		
	}
	
	
	

}







/*
�̼��洢��ʽ��[4bytes(�̼���ַ)1bytes(��У��)32bytes(�̼�����) ] X (n��)

*/
void read_firmware_rp_head()
{
	unsigned char * begin_addr = (unsigned char * )FLASH_RP_FIRMWARE_BEGIN;
	uint16_t i,j;
	uint32_t check_bytes_add = 0;
	uint16_t lines_size = 0;
	uint8_t *p_data;

	
	for(i=0;i<8;i++)  //��ȡ�̼�ͷ
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xxRP+i*32),begin_addr+i*37+5,32);
	}
	
	if(pheader_5xxRP.ulId != 0xdada10af || pheader_5xxRP.ucCfgComb != 7)
	{
		ap_param.rp_version = FIRM_HEAD_ERROR;	
		return;
	}
	
	begin_addr += 37*8;
	lines_size = pheader_5xxRP.uiFwSize/32;
	if(lines_size>2000)
	{
		ap_param.rp_version = FIRM_LINES_ERROR;	
		return;
	}	
	for(i=0;i<lines_size;i++)
	{
		p_data = begin_addr+i*37;  //ָ��ÿ�е���ʼ��ַ
		for(j=5;j<37;j++)
		{
			check_bytes_add +=p_data[j];
		}
	}
	if(check_bytes_add == pheader_5xxRP.ulCheckSum)
	{
		ap_param.rp_version = pheader_5xxRP.uiFwVer;
		ap_param_write_flash_flag = 1;
	}
	
}



/*
�̼��洢��ʽ��[4bytes(�̼���ַ)1bytes(��У��)32bytes(�̼�����) ] X (n��)

*/
void read_firmware_sensor_head()
{
	unsigned char * begin_addr = (unsigned char * )FLASH_SENSOR_FIRMWARE_BEGIN;
	uint16_t i,j;
	uint32_t check_bytes_add = 0;
	uint16_t lines_size = 0;
	uint8_t *p_data;

	
	for(i=0;i<8;i++)  //��ȡ�̼�ͷ
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xx+i*32),begin_addr+i*37+5,32);
	}
	
	if(pheader_5xx.ulId != 0xdada10af || pheader_5xx.ucCfgComb != 3)
	{
		ap_param.sensor_version = FIRM_HEAD_ERROR;	
		return;
	}
	
	begin_addr += 37*8;
	lines_size = pheader_5xx.uiFwSize/32;
	
	for(i=0;i<lines_size;i++)
	{
		p_data = begin_addr+i*37;  //ָ��ÿ�е���ʼ��ַ
		for(j=5;j<37;j++)
		{
			check_bytes_add +=p_data[j];
		}
	}
	if(check_bytes_add == pheader_5xx.ulCheckSum)
	{
		ap_param.sensor_version = pheader_5xx.uiFwVer;
		ap_param_write_flash_flag = 1;
	}
	
}





















void n1_send_firmware(struct_ap_n1_protocol *ptr,int who)
{
	uint32_t begin_address = 0;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	uint8_t data[2];
	
	if(who == N1_SEND_RP_FIRMWARE)
	{

		begin_address = FLASH_RP_FIRMWARE_BEGIN;
		EraseInit.Sector = FLASH_SECTOR_17;
	}
	else if(who == N1_SEND_S_FIRMWARE)
	{

		begin_address = FLASH_SENSOR_FIRMWARE_BEGIN;	
		EraseInit.Sector = FLASH_SECTOR_18;		
	}
	if(ptr->data[1] == 0)
	{
		ee_write_addr_add = 0;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);		
	}
	
	if(0 == write_bin_flash(begin_address + ee_write_addr_add,&ptr->data[2],ptr->lengh-3))
	{
			ee_write_addr_add += ptr->lengh-4;
			ee_write_packet_syn++;
			data[0] = ptr->data[0];
			data[1] = ee_write_packet_syn;
			insert_to_n1_buff(data,2,1);		
			if(ptr->data[0] == ptr->data[1]+1)       //�������
			{
				
			}
	}
	else
	{
			data[0] = ptr->data[0];
			data[1] = 0;
			insert_to_n1_buff(data,2,1);			
	}

	
}