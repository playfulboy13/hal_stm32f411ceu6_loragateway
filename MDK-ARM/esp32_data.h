#ifndef _ESP32_DATA_H
#define _ESP32_DATA_H

#include "main.h"
#include "lora_gateway.h"

extern uint16_t lost_packets;   // ví d?: d?m gói tin m?t
extern uint16_t total_packets;  // t?ng gói dã g?i

extern uint8_t uart2_rx_byte;
extern QueueHandle_t uart2Queue;

uint16_t CRC16_Modbus(uint8_t *buf, uint16_t len);
void encrypt_xor(uint8_t *buf, uint16_t len, uint8_t key);

void TaskData(void *pvParamters);
void TaskRelayUART(void *pvParameters);

#endif
