#ifndef _LORA_GATEWAY_H
#define _LORA_GATEWAY_H
#include "main.h"


extern int g_count;
extern float g_temp;
extern int rssi;

extern char debug[256];

typedef struct {
    float sensor[8];
    uint8_t cnt;
    uint8_t param;
    int rssi;
    uint8_t online;   // 1 = OK, 0 = NO RESPONSE
} NodeData_t;

extern NodeData_t g_node_data[3];

/*NODE 1*/

typedef struct
{
	float temp1;
	float temp2;
	float s3;
	float s4;
	float s5;
	float s6;
	
	uint8_t humid;
	uint8_t hour;
	uint8_t minutes;
	uint8_t seconds;
	int16_t rssi;
	uint16_t count;
	
}sensor_data_t;

extern sensor_data_t sensor_data;
extern sensor_data_t sensor_data2;
extern sensor_data_t sensor_data3;

void lora_init_config(void);
void TaskReceiveLora(void *pvParameters);
void TaskSendLora(void *pvParameters);
void TaskMaster(void *pvParameters);


#endif