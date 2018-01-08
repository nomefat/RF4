#include "main.h"
#include "stm32f4xx_hal.h"
#include "debug_uart.h"
#include "string.h"
#include "rf_hal.h"
#include "math.h"
#include "ap_param.h"
#include "debug.h"


extern UART_HandleTypeDef huart6;


extern SPI_HandleTypeDef* rf_index[];
extern void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel );



#define Q_LEN 256           //���г���
char debug_uart_dma_buff[Q_LEN];       //��������

char debug_uart_buff[Q_LEN+1];

char debug_send_buff[Q_LEN];

struct _cmd_param_int{
	int param_num;
	int param[10];
}cmd_param_int;


struct _cmd_list{
	char *cmd;
	void (*func)(char *param);
};

#define CMD_CALLBACK_LIST_BEGIN struct _cmd_list cmd_list[] = {NULL,NULL,
#define CMD_CALLBACK_LIST_END NULL,NULL};
#define CMD_CALLBACK(cmd_string,callback)	cmd_string,callback,




extern uint8_t ap_param_write_flash_flag;

extern uint8_t rf_scan_channel_enable;

extern uint8_t rf_send_1000_p_enable; 

extern struct_test_1000p_data test_1000p_data;

extern unsigned char rf_send_data[];

extern char gprs_debug_buff[256];

extern int enable_print_sensor_event;   //ʹ��rf ������¼���ӡ����
	
extern int gprs_print_rx_tx_data_enable;   //ʹ��gprs�շ����ݴ�ӡ����









void debug_uart_send_string(char *pstr)
{
	int num = 0;
	char *ptr = pstr;
	while(*ptr++)
	{
		num++;
		if(num>5000)return;
	}
	
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)pstr,num);
	
}





/**********************************************************************************************
***func: ���ڵ�Ƭ��һ�ο���DMA����
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_debug_dma_receive()
{
	SET_BIT((&huart6)->Instance->CR1, USART_CR1_IDLEIE);  //�򿪴��ڿ����ж�
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //��DMA����
}




/**********************************************************************************************
***func:���ڿ����жϻص�����
***     �����ж������ж�һ�����ݵĽ���
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_debug_idle_callback()
{
	HAL_DMA_Abort((&huart6)->hdmarx);
	huart6.RxState = HAL_UART_STATE_READY;
	huart6.hdmarx->State = HAL_DMA_STATE_READY;
	debug_uart_buff[0] = Q_LEN-DMA2_Stream1->NDTR;
	memcpy(debug_uart_buff+1,(char*)debug_uart_dma_buff,Q_LEN-DMA2_Stream1->NDTR);
	
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //��DMA����

}


/*
 * ���ܣ��Ӷ�����ȡ��һ���������ַ������
 * ʧ�ܣ�����-1
 * �ɹ�������0
 *cmd ��������ָ�룬param ��Ų�����ָ�롣
*/
int get_q_string(char *cmd,char *param)
{
	int i = 1;
	int timeout = 0;


	for(;;){
		if(debug_uart_buff[i] == ' ')
		{
			cmd[i-1] = 0; 
			i++;
			break;
		}
		else if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			cmd[i-1] = 0; 
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;
		cmd[i-1] = debug_uart_buff[i];
		i++;
				
	}

	for(;;){

		if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			*param = 0; 
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;	
		
		*param++ = debug_uart_buff[i];
		i++;	
	}

	return 0;
}

int str_to_int(char *str)
{
	int i = 0,j = 0;
	int ret = 0;
	for(;;){
	if(str[i++]==0||i>20)
		break;
	}
	j = i = i-2;
	for(;i>=0;i--)
	{
		ret += (str[i]-'0')*(pow(10,(j-i)));
	}
	return ret;
}

//
struct _cmd_param_int* get_str_param(char *param)
{
	char *ptr_now = param;
	char *ptr_pre = param;
	int i = 0;
	
	cmd_param_int.param_num = 0;
	for(;;){                     //�ָ���������տո�

		if(*ptr_now==' ')
		{
			*ptr_now = 0;
			cmd_param_int.param[i++] = str_to_int(ptr_pre);
			ptr_pre = ptr_now+1;
			cmd_param_int.param_num++;
		}
		ptr_now++;		
		if(*ptr_now==0)
		{
			cmd_param_int.param[i] = str_to_int(ptr_pre);
			cmd_param_int.param_num++;
			return &cmd_param_int;
		}
		if(ptr_now-param>100)		
			return &cmd_param_int;
	}
	
}

void print_now_rfmode()
{

}

void help(char *param)
{
	debug_uart_send_string("*******************************************************\r\n\
AP4RF_V0.1 cmd help\r\n\
ap....................��ӡAP����\
rf_manage:\
	1.setrfmode [] []...�ֱ�����4·��Ƶģʽ...����1:1-4��ʾ(rf1-rf4) ����2:0-3(����ģʽ,���ز�,���Ʋ�,����)\r\n\
	2.setrfch [] [].....�ֱ�����4·��Ƶͨ��...����1:1-4��ʾ(rf1-rf4) ����2:0-31 ��ͨ��\r\n\
	3.scan_ch [] .......4·��Ƶ�����ǵ��ز�ɨ��0-15ͨ�� ǰ����rf���ڵ��ز�ģʽ\r\n\
	4.rf_print [].......0ʧ��sensor�¼�����ӡ 1ʹ��sensor�¼�����ӡ\r\n\
\
gprs_manage:\
	1.gprs ..............��ӡһ��gprs����״̬\r\n\
	2.sendgprs200 .......gprs����200�ֽڵ����ݵ�������\r\n\
	3.gprs_print []......0��ʾ�رմ�ӡ 1��ʾ�򿪴�ӡ\r\n\
\
6.setrpch \r\n\
sensor_manage:\
	7.setrpslot \r\n\
	8.get_sensor \r\n\
\
*******************************************************\r\n");

}


extern struct_rf_stat rf_stat[4];
void set_rf_mode(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	if(ps_pram->param[0]>4 || ps_pram->param[0]<1)
	{
		debug_uart_send_string("ѡ��RF����1-4��\r\n");
		return;
	}
	if(ps_pram->param[1]>3 || ps_pram->param[1]<0)
	{
		debug_uart_send_string("RFģʽ����0-3��\r\n");
		return;
	}	
	rf_stat[ps_pram->param[0]-1].mode = ps_pram->param[1];
	rf_stat[ps_pram->param[0]-1].rf_power_stat = RF_POWER_OFF;
	
	debug_uart_send_string("RF�Ѿ�������ģʽ\r\n");
}

void setrfch(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
	uint8_t *ptr = (uint8_t *)&ap_param.ap_channel;
	
	if(ps_pram->param[0]>4 || ps_pram->param[0]<1)
	{
		debug_uart_send_string("ѡ��RF����1-4��\r\n");
		return;
	}	
	if(ps_pram->param[1]>30 || ps_pram->param[1]<0)
	{
		debug_uart_send_string("RFͨ������0-2��\r\n");
		return;
	}	
	

	
	rf_set_channel(rf_index[4-ps_pram->param[0]],ps_pram->param[1]);
	ptr[4-ps_pram->param[0]] = ps_pram->param[1];
//	ap_param_write_flash_flag = 1;
	sprintf(gprs_debug_buff,"setrfch ok %d %d\r\n",ps_pram->param[0],ps_pram->param[1]);
	debug_uart_send_string(gprs_debug_buff);	
}


extern void send_gprs_200_();
void send_gprs_200()
{
	send_gprs_200_();
	debug_uart_send_string("sendgprs200 ok..\r\n");
}

extern char* make_gprs_stat();
void get_gprs_stat()
{
	debug_uart_send_string(make_gprs_stat());	
}

char* make_rf_stat();
void get_rf_stat()
{
	debug_uart_send_string(make_rf_stat());		
}


extern struct_sensor_rp_param sensor_rp_param ;

void set_rp_ch(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
		
	sensor_rp_param.ParaFram.uiPoll = 0x0273;
	sensor_rp_param.ParaFram.uiCmd = 1;
	sensor_rp_param.ParaFram.uiBindId = 0;
	sensor_rp_param.ParaFram.uiSlotStateL = 0x81;
	sensor_rp_param.ParaFram.uiSlotStateM = 0;
	sensor_rp_param.ParaFram.uiSlotStateH = 0;
	sensor_rp_param.ParaFram.paraA.uiGrade = 1;
	sensor_rp_param.ParaFram.paraA.uiChannel = ps_pram->param[0];
	sensor_rp_param.ParaFram.paraB.uimySlot = 0x1b;	
	sensor_rp_param.ParaFram.paraB.uiSlotStateE = 1;	
}

void set_rp_slot(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
		
	sensor_rp_param.ParaFram.uiPoll = 0x0273;
	sensor_rp_param.ParaFram.uiCmd = 5;
	sensor_rp_param.ParaFram.uiBindId = 0;
	sensor_rp_param.ParaFram.uiSlotStateL = 0x81;
	sensor_rp_param.ParaFram.uiSlotStateM = 0;
	sensor_rp_param.ParaFram.uiSlotStateH = 0;
	sensor_rp_param.ParaFram.paraA.uiGrade = 1;
	sensor_rp_param.ParaFram.paraA.uiChannel = 11;
	sensor_rp_param.ParaFram.paraB.uimySlot = ps_pram->param[0];
	sensor_rp_param.ParaFram.paraB.uiSlotStateE = 1;		
}


void restart_sensor(char *param)
{
	re_start_sensor_event_record();
	debug_uart_send_string("restart sensor event record \r\n");
}

extern char debug_sensor_event_str[SENSOR_NUM][43];
void get_sensor(char *param)
{
	debug_sensor_event_to_str();
	debug_uart_send_string(&debug_sensor_event_str[0][0]);
}

void scan_channel_ebale(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	if(ps_pram->param[0] == 1)
	{
		rf_scan_channel_enable = 1;
		debug_uart_send_string("enable scan rf channel \r\n");
	}
	else
	{
		rf_scan_channel_enable = 0;
		debug_uart_send_string("disenable scan rf channel \r\n");
	}
	
}


void rf_send(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	if(ps_pram->param[0] >4 || ps_pram->param[0] <1)
	{	
		debug_uart_send_string("RF mast 1-4 \r\n");
	}
	rf_send_1000_p_enable = 0;
	rf_send_1000_p_enable |= (1<<(ps_pram->param[0]-1));
	rf_send_data[1] = 0;
	rf_send_data[2] = 0;
	rf_send_data[3] = 0;
	rf_send_data[4] = 0;
//	test_1000p_data.pakcet_count = ps_pram->param[1];
	
}


void enable_rf_print_sensor_event(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	if(ps_pram->param[0] >0 )
	{
		enable_print_sensor_event = 1;
		debug_uart_send_string("enable print sensor event \r\n");
	}
	else
	{
		enable_print_sensor_event = 0;
		debug_uart_send_string("disenable print sensor event \r\n");
	}
}

void enable_gprs_print_tx_rx_data(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	if(ps_pram->param[0] >0 )
	{
		gprs_print_rx_tx_data_enable = 1;
		debug_uart_send_string("enable gprs print tx_rx data \r\n");
	}
	else
	{
		gprs_print_rx_tx_data_enable = 0;
		debug_uart_send_string("disenable gprs print tx_rx data \r\n");
	}
}


void print_version()
{

	if(RF_DEFAULT_MODE == ENABLE_NOMODULATE_CARRIER)
	{
		if(rf_scan_channel_enable == 0)
			debug_uart_send_string("033ap:���ز�ģʽ \r\n");
		else
			debug_uart_send_string("033ap:���ز�ɨƵģʽ \r\n");
	}
	else if(RF_DEFAULT_MODE == ENABLE_MODULATE_CARRIER)
	{
		if(rf_scan_channel_enable == 0)
			debug_uart_send_string("033ap:���Ʋ�ģʽ \r\n");
		else
			debug_uart_send_string("033ap:���Ʋ�ɨƵģʽ \r\n");
	}	
	else if(RF_DEFAULT_MODE == RF_WORK)
	{
		sprintf(gprs_debug_buff,"033ap:����ģʽ version=%d.%d\r\n",AP_VERSION>>8,AP_VERSION&0xff);
		debug_uart_send_string(gprs_debug_buff);
	}		
	else if(RF_DEFAULT_MODE == RF_IDLE)
	{
		debug_uart_send_string("033ap:����ģʽģʽ(������1000���շ�����) \r\n");
	}			
}


void get_ap_param(char *param)
{
		sprintf(gprs_debug_buff,"033ap:version=%d.%d\r\napid=%x\r\nch=%d %d %d %d\r\nserverip=%d.%d.%d.%d port=%d\r\n",
	AP_VERSION>>8,AP_VERSION&0xff,ap_param.ap_id,(ap_param.ap_channel>>24)&0XFF,(ap_param.ap_channel>>16)&0XFF,(ap_param.ap_channel>>8)&0XFF
	,(ap_param.ap_channel)&0XFF,ap_param.gprs_server_ip&0xff,(ap_param.gprs_server_ip>>8)&0xff,(ap_param.gprs_server_ip>>16)&0xff,ap_param.gprs_server_ip>>24,ap_param.gprs_server_port);
		debug_uart_send_string(gprs_debug_buff);	
}

//�ڴ˴������������ַ����ͻص�����
CMD_CALLBACK_LIST_BEGIN


CMD_CALLBACK("?",help)		
CMD_CALLBACK("setrfmode",set_rf_mode)		
CMD_CALLBACK("sendgprs200",send_gprs_200)	
CMD_CALLBACK("gprs",get_gprs_stat)		
CMD_CALLBACK("setrfch",setrfch)
CMD_CALLBACK("rf",get_rf_stat)
CMD_CALLBACK("setrpch",set_rp_ch)
CMD_CALLBACK("setrpslot",set_rp_slot)
CMD_CALLBACK("restart_sensor",restart_sensor)
CMD_CALLBACK("get_sensor",get_sensor)
CMD_CALLBACK("scan_ch",scan_channel_ebale)
CMD_CALLBACK("send",rf_send)
CMD_CALLBACK("gprs_print",enable_gprs_print_tx_rx_data)
CMD_CALLBACK("rf_print",enable_rf_print_sensor_event)
CMD_CALLBACK("ap",get_ap_param)


CMD_CALLBACK_LIST_END









char cmd[100];
char param[32];
int get_cmd(void)
{
	int i = 0;
	if(get_q_string(cmd,param) == -1)
		return 0;
	
	for(;;){
		if(strcmp(cmd,cmd_list[i].cmd)==0)
			return i;	
		if(cmd_list[++i].cmd==NULL)
			return 0;
	}
}



void debug_cmd_handle(void)
{
	int func_index = get_cmd();
	if(func_index<=0)
		return;
	cmd_list[func_index].func(param);
}







