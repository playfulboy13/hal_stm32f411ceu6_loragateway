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
extern uint8_t state;

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

extern uint8_t relay_cmd[3];

extern sensor_data_t sensor_data;
extern sensor_data_t sensor_data2;
extern sensor_data_t sensor_data3;

extern uint8_t node_id_relay;



// ====================
// Ki?u l?nh relay
// ====================
typedef struct {
    uint8_t node_id;
    uint8_t relay_state;
} relay_cmd_t;

// ====================
// Queue handle
// ====================
extern QueueHandle_t relayQueue;

void lora_init_config(void);
void TaskReceiveLora(void *pvParameters);
void TaskSendLora(void *pvParameters);
void TaskMaster(void *pvParameters);
void TaskControlRelay(void* pvParameters);

#endif