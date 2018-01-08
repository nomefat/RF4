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



#define Q_LEN 256           //队列长度
char debug_uart_dma_buff[Q_LEN];       //队列数组

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

extern int enable_print_sensor_event;   //使能rf 检测器事件打印功能
	
extern int gprs_print_rx_tx_data_enable;   //使能gprs收发数据打印功能









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
***func: 用于单片第一次开启DMA接收
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_debug_dma_receive()
{
	SET_BIT((&huart6)->Instance->CR1, USART_CR1_IDLEIE);  //打开串口空闲中断
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //打开DMA接收
}




/**********************************************************************************************
***func:串口空闲中断回调函数
***     空闲中断用来判断一包数据的结束
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
	
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //打开DMA接收

}


/*
 * 功能：从队列中取出一个完整的字符串命令。
 * 失败：返回-1
 * 成功：返回0
 *cmd 存放命令的指针，param 存放参数的指针。
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
	for(;;){                     //分割参数，按照空格

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
ap....................打印AP参数\
rf_manage:\
	1.setrfmode [] []...分别设置4路射频模式...参数1:1-4表示(rf1-rf4) 参数2:0-3(工作模式,单载波,调制波,空闲)\r\n\
	2.setrfch [] [].....分别设置4路射频通道...参数1:1-4表示(rf1-rf4) 参数2:0-31 个通道\r\n\
	3.scan_ch [] .......4路射频启动是单载波扫描0-15通道 前提是rf处于单载波模式\r\n\
	4.rf_print [].......0失能sensor事件包打印 1使能sensor事件包打印\r\n\
\
gprs_manage:\
	1.gprs ..............打印一次gprs工作状态\r\n\
	2.sendgprs200 .......gprs发送200字节的数据到服务器\r\n\
	3.gprs_print []......0表示关闭打印 1表示打开打印\r\n\
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
		debug_uart_send_string("选择RF错误（1-4）\r\n");
		return;
	}
	if(ps_pram->param[1]>3 || ps_pram->param[1]<0)
	{
		debug_uart_send_string("RF模式错误（0-3）\r\n");
		return;
	}	
	rf_stat[ps_pram->param[0]-1].mode = ps_pram->param[1];
	rf_stat[ps_pram->param[0]-1].rf_power_stat = RF_POWER_OFF;
	
	debug_uart_send_string("RF已经设置完模式\r\n");
}

void setrfch(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
	uint8_t *ptr = (uint8_t *)&ap_param.ap_channel;
	
	if(ps_pram->param[0]>4 || ps_pram->param[0]<1)
	{
		debug_uart_send_string("选择RF错误（1-4）\r\n");
		return;
	}	
	if(ps_pram->param[1]>30 || ps_pram->param[1]<0)
	{
		debug_uart_send_string("RF通道错误（0-2）\r\n");
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
			debug_uart_send_string("033ap:单载波模式 \r\n");
		else
			debug_uart_send_string("033ap:单载波扫频模式 \r\n");
	}
	else if(RF_DEFAULT_MODE == ENABLE_MODULATE_CARRIER)
	{
		if(rf_scan_channel_enable == 0)
			debug_uart_send_string("033ap:调制波模式 \r\n");
		else
			debug_uart_send_string("033ap:调制波扫频模式 \r\n");
	}	
	else if(RF_DEFAULT_MODE == RF_WORK)
	{
		sprintf(gprs_debug_buff,"033ap:工作模式 version=%d.%d\r\n",AP_VERSION>>8,AP_VERSION&0xff);
		debug_uart_send_string(gprs_debug_buff);
	}		
	else if(RF_DEFAULT_MODE == RF_IDLE)
	{
		debug_uart_send_string("033ap:空闲模式模式(可启动1000包收发测试) \r\n");
	}			
}


void get_ap_param(char *param)
{
		sprintf(gprs_debug_buff,"033ap:version=%d.%d\r\napid=%x\r\nch=%d %d %d %d\r\nserverip=%d.%d.%d.%d port=%d\r\n",
	AP_VERSION>>8,AP_VERSION&0xff,ap_param.ap_id,(ap_param.ap_channel>>24)&0XFF,(ap_param.ap_channel>>16)&0XFF,(ap_param.ap_channel>>8)&0XFF
	,(ap_param.ap_channel)&0XFF,ap_param.gprs_server_ip&0xff,(ap_param.gprs_server_ip>>8)&0xff,(ap_param.gprs_server_ip>>16)&0xff,ap_param.gprs_server_ip>>24,ap_param.gprs_server_port);
		debug_uart_send_string(gprs_debug_buff);	
}

//在此处添加你的命令字符串和回调函数
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







