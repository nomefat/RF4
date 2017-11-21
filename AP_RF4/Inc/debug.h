#ifndef _DEBUG_H_
#define _DEBUG_H_




#define SENSOR_NUM       256



typedef struct _sensor_event_list
{
	
	unsigned short sensor_id;          //�豸�ɣ�
	unsigned char slot;                  //ʱ���
	signed char rssi;                  //�ź�ǿ��
	unsigned char syn;                 //�����
	unsigned char last_repeat_times;
	unsigned int all_packet_count;        //�ܰ���
	unsigned int receive_packet_count;    //���յ��İ���
	unsigned int repeat_packet_count;     //�ظ�����
	unsigned char repeat_packet_times_count[10];
	
}struct_sensor_event_list;




void debug_insert_sensor_event(unsigned short id,unsigned char syn,signed char rssi,unsigned char slot,unsigned char repeat_times);



void re_start_sensor_event_record();

void debug_sensor_event_to_str();





#endif

