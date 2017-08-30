#include "stm32f4xx_hal.h"
#include "to_n1.h"
#include "string.h"
#include "math.h"
#include "gprs_hal.h"
#include "debug_uart.h"


typedef struct{
	int head;
	int n1_id;
	unsigned short length;
	unsigned char data[1];
}strcut_n1_data_head;


#define GPRS_DEBUG_PRINT

#define GPRS_STR_QUEUE_LENGH 4096      //字符串队列长度，用于存储gprs模块返回的字符串
#define GPRS_DATA_BUFF_LENGH (1024 + 512)      //用于从队列中取出完整的一条命令或者数据 (经过实际测试 可能含有几条命令)

#define GPRS_SEND_DATA_BUFF_LENGH 256


#define GPRS_CONNECT_FAIL_TIMEOUT   2

#define debug(str) debug_uart_send_string(str)

uint8_t n1_to_server_data[256];         //N1发送给gprs 服务器的数据


uint8_t gprs_str_queue[GPRS_STR_QUEUE_LENGH];  //用于存储gprs模块返回的字符串

uint8_t gprs_data_buff[GPRS_DATA_BUFF_LENGH];   //用于从队列中取出完整的一条命令或者数据 


int gprs_str_queue_write = 0;               //写偏移
int gprs_str_queue_read = 0;								//读偏移

int gprs_receive_packet_flag = 0;


int gprs_cmd_param_num = 0;
unsigned char gprs_cmd_param[10][50];  //用来存储字符串中的参数

uint8_t gprs_sec_flag = 0;         //秒点标志 由同步包中断置1

char gprs_debug_buff[256];

char at_cmd_conn[50];


struct struct_gprs_stat{
	char ati_ok;
	char creg_ok;       
	char cgreg_ok;  	
	char csq;              //信号质量
	struct {
		char connect_ok;
		int connect_fail_times;	
		int send_no_timeout;            //发送后直接没有收到 > 超时
		int gprs_send_error_timeout;   // tcp发送数据没有成功计数	
	}con_client[3];
	char send_data_id;       //记录那个链接 发送数据
	unsigned short send_data_num;  //记录连接发送的数据个数
	unsigned char *send_data_buff;	
	char check_id;      //查询那个连接 发送成功字节个数
	char reboot_flag;
#define GPRS_REBOOT_SETP0 0               //执行关闭GSM模块电源指令
#define GPRS_REBOOT_SETP1 1               //执行打开GSM模块电源指令
#define GPRS_REBOOT_SETP2 4               //GSM已经上电 不执行操作
}gprs_stat;



#define GPRS_CMD_OK 0
#define GPRS_CMD_CGREG 1
#define GPRS_CMD_CSQ 2
#define GPRS_CMD_SEND_READY 3
#define GPRS_CMD_CREG 4
#define GPRS_CMD_REVISION 5
#define GPRS_CMD_CONNECT_OK_0 6
#define GPRS_CMD_CONNECT_OK_1 7
#define GPRS_CMD_CONNECT_OK_2 8
#define GPRS_CMD_CONNECT_FAIL_0 9
#define GPRS_CMD_CONNECT_FAIL_1 10
#define GPRS_CMD_CONNECT_FAIL_2 11
#define GPRS_CMD_CONNECT_CLOSED_0 12
#define GPRS_CMD_CONNECT_CLOSED_1 13
#define GPRS_CMD_CONNECT_CLOSED_2 14
#define GPRS_CMD_REBOOT   15
#define GPRS_CMD_QISTAT   16
#define GPRS_CMD_QISACK   17
#define GPRS_CMD_RECEIVE  18


char *gprs_str_cmd[] = {

	"OK",
	"+CGREG",
	"+CSQ",
	">",
	"+CREG",
	"Revision",
	"0, CONNECT OK",
	"1, CONNECT OK",
	"2, CONNECT OK",
	"0, CONNECT FAIL",
	"1, CONNECT FAIL",
	"2, CONNECT FAIL",
	"0, CLOSED",
	"1, CLOSED",
	"2, CLOSED",	
	"Call Ready",
	"+QISTATE",
	"+QISACK",
	"+RECEIVE",
};




#define AT_CMD_ATI             "ATI\r\n"          //AT指令 判断GSM模块是否启动
#define AT_CMD_AT_CSQ         "AT+CSQ\r\n"      //信号强度
#define AT_CMD_AT_CREG  			"AT+CREG?\r\n"    //GSM网络是否注册    +CREG: 0,1  // <stat>=1,GSM网络已经注册上
#define AT_CMD_AT_CGREG       "AT+CGREG?\r\n"   //GPRS网络是否注册   +CGREG: 0,1    // <stat>=1,GPRS网络已经注册上
#define AT_CMD_AT_SET_BAUD    "AT+IPR=115200&W\r\n"   //配置固定波特率
#define AT_CMD_AT_QIMUX       "AT+QIMUX=1\r\n"   //配置多链接模式
#define AT_CMD_AT_QISTAT      "AT+QISTATE\r\n"
#define AT_CMD_AT_CLOSE_CONNECT "AT+QICLOSE=%d\r\n"    //关闭指定的tcp链接
#define AT_CMD_AT_QISACK      "AT+QISACK=%d\r\n"        //查询发送的数据
#define AT_CMD_AT_SEND         "AT+QISEND=%d,%d\r\n"       //发送数据  连接编号 数量


extern UART_HandleTypeDef huart2;


/*
 * 功能：往队列中写入一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char gprs_str_write_queue(uint8_t data)
{
	if((gprs_str_queue_write+1)%GPRS_STR_QUEUE_LENGH==gprs_str_queue_read)
		return -1;
	gprs_str_queue[gprs_str_queue_write] = data;
	gprs_str_queue_write = (gprs_str_queue_write+1)%GPRS_STR_QUEUE_LENGH;
	return 0;
}

/*
 * 功能：从队列中读取一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char gprs_str_read_queue(uint8_t *pdata)
{
	if(gprs_str_queue_write==gprs_str_queue_read)
		return -1;		
	*pdata = gprs_str_queue[gprs_str_queue_read];
	gprs_str_queue_read = (gprs_str_queue_read+1)%GPRS_STR_QUEUE_LENGH;
	return 0;
}

/*
 * 功能：将串口dma接收完的一包数据 copy到环形队列
 * 失败：返回-1
 * 成功：返回0
*/
void gprs_str_copy_to_queue(unsigned short len,char* p_data)
{
	int i = 0;

	gprs_str_write_queue(len);
	gprs_str_write_queue(len>>8);	
	for(i=0;i<len;i++)
	{
		gprs_str_write_queue(*p_data++);	
	
	}
	gprs_receive_packet_flag++;  //每次完成一个数据包的copy 此变量++
}



int gprs_str_to_int(char *pstr)
{
	char *p_str = pstr;
	int lengh = 0;
	int offset = 0;
	int ret = 0;
	int i = 0;
	
	while(1)
	{
		if(*p_str == ' ')
			offset++;
		if(*p_str == 0)
			break;
		p_str++;
		lengh++;
	}
	p_str = pstr+offset;
	lengh = lengh - offset;
	
	for(i=0;i<lengh;i++)
	{
	 ret += (*(p_str+i)-'0')*pow(10,(lengh-i-1));	
	}
	return ret;
}


void gprs_uart_send_string(char *pstr)
{
	int num = 0;
	char *ptr = pstr;
	while(*ptr++)
	{
		num++;
		if(num>5000)return;
	}
	
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)pstr,num);
	
}

/*
	说明：GSM网络正常后 调用这个函数查询 gprs网络是否正常

*/
void gprs_at_get_CGREG(void)
{
	gprs_uart_send_string(AT_CMD_AT_CGREG);	  //查询gsm网络

}

/*
	说明：查询那个链接的状态     gprs模块支持同时保存多个tcp连接

*/
void gprs_at_get_tcp_online(int num)
{
	gprs_stat.con_client[num].connect_ok = 0;


}

/*
	说明： 每次上电 后调用ATI 返回revision字符串后 执行这个函数

*/
void gprs_at_init(void)
{
	int i;
//	gprs_uart_send_string(AT_CMD_AT_SET_BAUD);   //设置波特率 
//	i = 200000;
//	while(i--);
	gprs_uart_send_string(AT_CMD_AT_QIMUX);	  //配置为多链接模式
	i = 200000;
	while(i--);
	gprs_uart_send_string("ATE1\r\n");	
}

void gprs_reboot(void)
{
	
	gprs_stat.reboot_flag = 1;
/*	gprs_off();
	i = 100000;
	while(i--);
	gprs_on();
	memset(&gprs_stat,0,sizeof(struct struct_gprs_stat));	
	gprs_stat.send_data_id = -1; //表示没有连接要发送数据
	*/
}

char at_cmd_conn[50];
void gprs_at_tcp_conn(int num,unsigned long ip,unsigned short port)
{

	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,"AT+QIOPEN=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n",num,ip>>24,(ip>>16)&0xff,(ip>>8)&0xff,ip&0xff,port);
	//sprintf(at_cmd_conn,"AT+QIOPEN=0,\"TCP\",\"219.239.83.74\",40005\r\n");
	gprs_uart_send_string(at_cmd_conn);

}

void gprs_at_tcp_close(int num)
{
	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,"AT+QICLOSE=%d\r\n" ,num);
	gprs_uart_send_string(at_cmd_conn);

}

//每次有新数据从N1产生就调用一次该函数，
int gprs_send_data_flag(int num, unsigned short data_num,unsigned char * data_buff)
{
	if(gprs_stat.send_data_id >=0)        //有还没发送的数据
		return -1;
	
	if(num > 2)        //有还没发送的数据
		return -2;	
	
	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,AT_CMD_AT_SEND,num,data_num);
	gprs_uart_send_string(at_cmd_conn);                    //发送一次发送数据请求的命令
	gprs_stat.con_client[num].send_no_timeout = 0;
	gprs_stat.send_data_id = num;
	gprs_stat.send_data_num = data_num;
	gprs_stat.send_data_buff = data_buff;
}







void gprs_send_data(int num, unsigned short data_num,unsigned char * data_buff)
{
	if(gprs_stat.con_client[num].connect_ok == 0)
		return;
	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,AT_CMD_AT_SEND,num,data_num);
	gprs_uart_send_string(at_cmd_conn);
	gprs_stat.send_data_id = num;
	gprs_stat.send_data_num = data_num;
	gprs_stat.send_data_buff = data_buff;
}

int gprs_str_cmd_handle(char *pstr)
{
		int str_cmd_num;
		int i;
		unsigned short *data_len;
	  int lengh = 0;
		str_cmd_num = sizeof(gprs_str_cmd)/4;
		for(i=0;i<str_cmd_num;i++)
		{
			if(strcmp(pstr,gprs_str_cmd[i])==0)
				break;	
		}
		
		if(i == str_cmd_num)
			return 0;
		
		switch(i)
		{
			case GPRS_CMD_OK: break;
			case GPRS_CMD_CREG: 
				if(gprs_cmd_param[1][0] == '0'+1 || gprs_cmd_param[1][0] == '0'+5) 
				{
					gprs_stat.creg_ok = 1;
					gprs_at_get_CGREG();
				}		
				break;		
			case GPRS_CMD_CGREG: 
				if(gprs_cmd_param[1][0] == '0'+1 ||gprs_cmd_param[1][0] == '0'+5) 			
				{
					gprs_stat.cgreg_ok = 1;			
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:cgreg ok\r\n");

#endif					
				}	
				else if(gprs_cmd_param[1][0] == '0'+1 ||gprs_cmd_param[1][0] == '0'+3) 	
				{
#ifdef GPRS_DEBUG_PRINT					
					debug("gprs:cgreg 0,3\r\n");

#endif
				}
			
			break;
			case GPRS_CMD_CSQ: gprs_stat.csq =  (gprs_cmd_param[0][0]-'0')*10 + gprs_cmd_param[0][1]-'0' ;
#ifdef GPRS_DEBUG_PRINT

		sprintf(gprs_debug_buff,"gprs:rssi %d\r\n",gprs_stat.csq);
		debug(gprs_debug_buff);

#endif	
				break;
			case GPRS_CMD_SEND_READY: 
				gprs_stat.con_client[gprs_stat.send_data_id].send_no_timeout	= 0;			
				HAL_UART_Transmit_DMA(&huart2,gprs_stat.send_data_buff,gprs_stat.send_data_num);		
				gprs_stat.send_data_id = -1;	
				gprs_stat.send_data_num = 0;

			
				if(gprs_stat.send_data_id == 0)
				{
					
					
#ifdef GPRS_DEBUG_PRINT

					sprintf(gprs_debug_buff,"gprs: client_0 send data %d\r\n",gprs_stat.send_data_num);
					debug(gprs_debug_buff);

#endif			


				}
				break;
			case GPRS_CMD_REVISION:
				gprs_stat.ati_ok = 1;
				gprs_at_init();
				break;
			case GPRS_CMD_CONNECT_OK_0 :
				gprs_stat.con_client[0].connect_ok = 1; 	
				gprs_stat.con_client[0].connect_fail_times = 0;
//				gprs_send_data(0,gprs_ap_data_buff[0],gprs_ap_data_buff); 
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:0,connect ok\r\n");

#endif						
				break;
			case GPRS_CMD_CONNECT_OK_1 :gprs_stat.con_client[1].connect_ok = 1;break;
			case GPRS_CMD_CONNECT_OK_2 :gprs_stat.con_client[2].connect_ok = 1;break;
			case GPRS_CMD_CONNECT_FAIL_0 :
				gprs_stat.con_client[0].connect_fail_times += 1; 
			  gprs_at_tcp_close(0);

			  if(gprs_stat.con_client[0].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)
				{
					debug("gprs:0,connect fail timeout reboot\r\n");
					gprs_stat.con_client[0].connect_fail_times = 0;
					gprs_reboot();
					break;
				}
				debug("gprs:0,connect fail\r\n");				
				break;
			case GPRS_CMD_CONNECT_FAIL_1 :gprs_stat.con_client[1].connect_fail_times += 1; gprs_at_tcp_close(1);if(gprs_stat.con_client[1].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)gprs_reboot();break;
			case GPRS_CMD_CONNECT_FAIL_2 :gprs_stat.con_client[2].connect_fail_times += 1; gprs_at_tcp_close(2);if(gprs_stat.con_client[2].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)gprs_reboot();break;
			case GPRS_CMD_CONNECT_CLOSED_0 : 
				gprs_stat.con_client[0].connect_ok = 0;  
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:0,closed\r\n");

#endif						
				break;
			case GPRS_CMD_CONNECT_CLOSED_1 : gprs_stat.con_client[1].connect_ok = 0;  break;
			case GPRS_CMD_CONNECT_CLOSED_2 : gprs_stat.con_client[2].connect_ok = 0;  break;
			
			case GPRS_CMD_REBOOT	:

				gprs_at_init();
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:reinit\r\n");

#endif						
				break;

			case GPRS_CMD_QISACK :
#ifdef GPRS_DEBUG_PRINT

					sprintf(gprs_debug_buff,"gprs:send_data=%d ack_data=%d noack_data=%d\r\n",gprs_str_to_int(&gprs_cmd_param[0][0]),gprs_str_to_int(&gprs_cmd_param[1][0]),gprs_str_to_int(&gprs_cmd_param[2][0]));
					debug(gprs_debug_buff);

#endif					
				if(gprs_cmd_param[2][0]!='0')
				{
							
					gprs_stat.con_client[0].gprs_send_error_timeout++;
					if(gprs_stat.con_client[0].gprs_send_error_timeout>2)
					{
						gprs_stat.con_client[0].gprs_send_error_timeout = 0;
						gprs_stat.con_client[0].connect_ok = 0;
					}
				}
				else
				{
					gprs_stat.con_client[0].gprs_send_error_timeout = 0;
				}
			break;
			
			case GPRS_CMD_QISTAT:
				if(gprs_cmd_param[0][0]=='0')
				{
					lengh = lengh;
				}
			
			break;
			case	GPRS_CMD_RECEIVE:
				lengh = gprs_str_to_int(&gprs_cmd_param[1][0]);
#ifdef GPRS_DEBUG_PRINT

					sprintf(gprs_debug_buff,"gprs:receive data %d\r\n",lengh);
					debug(gprs_debug_buff);

#endif			
				return lengh;
			
			break;
			default:break;

		}
		return 0;
}


/*
 * 功能：解析取得的字符串 分割成命令和参数
 * 失败：返回-1
 * 成功：返回0
*/
void gprs_get_cmd_param(char *pstr , unsigned short length)
{
	int i,j;
	int flag_param = 0;
	int n1_data_length = 0;
	int strat_string = 0;

	memset(gprs_cmd_param,0,500);
	for(i=0;i<length;i++)
	{		
		
		if(pstr[i] == '\r' || pstr[i] == '\n' || pstr[i] == '>')  //索引到字符串结束  将该字节置0 作为字符串结束标志
		{
			if(pstr[i] != '>')
				pstr[i] = 0;
			if(pstr[i] == '>')
				strat_string = i;			
			if(pstr[i+1] == '\n' || pstr[i+1] == ' ' )
				pstr[++i] = 0;

			n1_data_length = gprs_str_cmd_handle(&pstr[strat_string]);
			if(n1_data_length>0)
			{
				insert_to_n1_buff(&pstr[i+1+10],pstr[i+1+8],AP_GPRS_SERVER_DATA);
			}
			gprs_cmd_param_num = 0;
			flag_param = 0;
			strat_string = i+1;
		}

		if(pstr[i] == ':')	//表示带有参数	
		{
			pstr[i++] = 0;   //把这个： 跨过去 以免写进参数里
			gprs_cmd_param_num++;
			flag_param = 1;
			j = 0;			
		}
		
		if(pstr[i] == ',' && flag_param == 1)
		{
			pstr[i++] = 0;   //把这个, 跨过去 以免写进参数里
			gprs_cmd_param_num++;		
			j = 0;
		}
		
		if(flag_param == 1)
		{
			if(j!=0 || pstr[i]!=0x20)
			{	
				gprs_cmd_param[gprs_cmd_param_num-1][j++] = pstr[i];
				pstr[i] = 0;		
			}				
		}

	}
		


}



/*
* 功能 : main函时调用 来处理gprs模块的一些请求和状态查询等任务 * 成功：返回0
*/
void gprs_data_handle(void)
{
	unsigned short length = 0;
	uint8_t data = 0;
	int i;
	int begin;
	char ret = 0;
	uint8_t *p_gprs_data_buff = gprs_data_buff;
	strcut_n1_data_head *p_n1_data_head;
	
	//判断是否有新数据
	if(gprs_receive_packet_flag == 0)
		return;
	
	gprs_receive_packet_flag--; //有可能积攒了多条数据
	
	for(i=0;i<2;i++)
	{
		ret = gprs_str_read_queue(&data);
		if(ret!=0)
			return;
		length |= data<<(i*8);
	}
	
	if(length<=0)
		return;
	
	if(length > GPRS_DATA_BUFF_LENGH)
		length = GPRS_DATA_BUFF_LENGH;
	
	for(i=0;i<length;i++)
	{
		ret = gprs_str_read_queue(p_gprs_data_buff++);
		if(ret!=0)
			return;			
	}
	
	
	begin = 0;
	i = 0;
	while(i<length)
	{
		p_n1_data_head = (strcut_n1_data_head *)&gprs_data_buff[i];
		if(p_n1_data_head->head == 0x584d5555)  //服务器下发N1的数据
		{
			insert_to_n1_buff(&gprs_data_buff[i],p_n1_data_head->length+12,AP_GPRS_SERVER_DATA);
			i += p_n1_data_head->length+12;
			begin += p_n1_data_head->length+12;
		}
		else if(gprs_data_buff[i] == '\r' || gprs_data_buff[i] == '\n' ||gprs_data_buff[i] == '>')
		{
			i++;
			if(gprs_data_buff[i] == '\r' || gprs_data_buff[i] == '\n' ) 
			{
				i++;
			}
			gprs_get_cmd_param(&gprs_data_buff[begin] , i-begin);
			begin = i;
		}
		else
			i++;
	}
	
}

void close_tcp_conn(int index)
{

	sprintf(at_cmd_conn,AT_CMD_AT_CLOSE_CONNECT ,index);				
	gprs_uart_send_string(at_cmd_conn);	
	gprs_stat.con_client[index].connect_ok = 0;
}



/*
	说明： main主循环调用

*/
void gprs_main_call()
{
	static int timeout = 0;
	static int send_cmd_times = 0;   //向gprs模块发送命令计数
	static int send_cmd_timeout = 0;   //向gprs模块发送命令后 超时计数	
	static int get_link_stat = 0;
	
	gprs_data_handle();

	if(gprs_sec_flag < 1) //1sec进入下面一次
		return;
	
	
	gprs_sec_flag = 0;
	
	if(gprs_stat.reboot_flag == GPRS_REBOOT_SETP0) //关GSM电源
	{
		grps_power_off();
		gprs_stat.reboot_flag++;
		gprs_stat.ati_ok = 0;
		gprs_stat.creg_ok = 0;
		gprs_stat.cgreg_ok = 0;
		gprs_stat.check_id = -1;
		memset(gprs_stat.con_client,0,sizeof(gprs_stat.con_client));
		return;
	}
	else if(gprs_stat.reboot_flag == GPRS_REBOOT_SETP1)  //开GSM电源
	{
		grps_power_on();
		gprs_stat.reboot_flag++;
		timeout = 0;
		send_cmd_timeout = 0;
		send_cmd_times = 0;		
		return;		
	}

	if(gprs_stat.ati_ok == 0)                         //命令1
	{
		timeout++;
		if(timeout>90)
			gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;      //关GSM电源
		gprs_uart_send_string(AT_CMD_ATI);  
		return;
	}
	if(gprs_stat.creg_ok == 0)                           //命令2
	{
		timeout++;
		if(timeout>90)
			gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;		//关GSM电源
		gprs_uart_send_string(AT_CMD_AT_CREG);  
		return;
	}	
	if(gprs_stat.cgreg_ok == 0)                            //命令3
	{
		timeout++;
		if(timeout>90)
			gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;			//关GSM电源
		gprs_uart_send_string(AT_CMD_AT_CGREG);  
		return;
	}	

	if(gprs_stat.con_client[0].connect_ok !=1)                //判断是否联网状态
	{
		if(send_cmd_timeout-- <= 0)
		{
			gprs_at_tcp_conn(0,(219<<24)|(239<<16)|(83<<8)|74,40005);      //发送一次命令后  20秒超时后才可以再次发送
			send_cmd_timeout = 20;
			send_cmd_times++;
			if(send_cmd_times>6)       //连续链接了6次 都没有成功
			{
				send_cmd_times = 0;
				gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;			//关GSM电源				
			}
		}
	}
	else     //联网状态
	{
		send_cmd_timeout = 0;
		send_cmd_times = 0;		
		if(gprs_stat.send_data_id == 0 && gprs_stat.send_data_num >0)     //有数据需要发送
		{	
			if(gprs_stat.con_client[0].send_no_timeout>0)      //已经发送过一次发送命令了  还没有收到> 此时需要等超时后才能再次发送
			{
				gprs_stat.con_client[0].send_no_timeout--;
				return;
			}
			else
			{
				gprs_stat.con_client[0].send_no_timeout = 10;
				memset(at_cmd_conn,0,50);
				sprintf(at_cmd_conn,AT_CMD_AT_SEND,0,gprs_stat.send_data_num);
				gprs_uart_send_string(at_cmd_conn);
			}
		}
		else
		{
			get_link_stat++;
			if(get_link_stat>30)
			{
				get_link_stat = 0;
				memset(at_cmd_conn,0,50);
				sprintf(at_cmd_conn,AT_CMD_AT_QISACK ,0);				
				gprs_uart_send_string(at_cmd_conn);
			}
		}
		
	}
	

}


void send_gprs_data(void *pdata,int len)
{
	memcpy(n1_to_server_data,pdata,len);
	gprs_stat.send_data_id = 0;
	gprs_stat.send_data_buff = n1_to_server_data;
	gprs_stat.send_data_num = 200;	
}


void send_gprs_200_()
{
	memset(n1_to_server_data,0x55,200);
	gprs_stat.send_data_id = 0;
	gprs_stat.send_data_buff = n1_to_server_data;
	gprs_stat.send_data_num = 200;
}

char* make_gprs_stat()
{
	sprintf(gprs_debug_buff,"gprs:ati=%d creg=%d cgreg=%d tcp=%d \r\n",gprs_stat.ati_ok,gprs_stat.creg_ok,gprs_stat.cgreg_ok,gprs_stat.con_client[0].connect_ok);
	return gprs_debug_buff;
}


