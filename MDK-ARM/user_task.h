#ifndef _USER_TASK_H
#define _USER_TASK_H
#include "main.h"

#define DS(x)       HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define SH_CP(x)    HAL_GPIO_WritePin(SH_CP_GPIO_Port, SH_CP_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST_CP(x)    HAL_GPIO_WritePin(ST_CP_GPIO_Port, ST_CP_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

// Xung d?ch: SH_CP toggle
#define XUNG_DICH()     { SH_CP(1); SH_CP(0);}
#define XUNG_CHOT()     { ST_CP(1); ST_CP(0); } 


#define RELAY1_BIT 0
#define RELAY2_BIT 1

void xuat_1_byte(uint8_t data);
void xuat_8led_7doan(uint8_t cot_hthi, uint8_t so_hthi);
void giai_ma_quet(void);
void TaskHienThi(void *pvParameters);  // Không c?n __attribute__
void TaskHieuUngLed(void *pvParameters);
void TaskRelay(void *pvParameters);
void TaskLedPWM(void *pvParameters);
extern uint8_t led_data, relay_data;


#endif