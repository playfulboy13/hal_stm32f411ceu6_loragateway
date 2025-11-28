#include "user_task.h"

static const uint8_t ma7doan[] = {
    0xc0, 0xf9, 0xa4, 0xb0,
    0x99, 0x92, 0x82, 0xf8,
    0x80, 0x90
};

static uint8_t tt8led = 0;
uint8_t led_7dq[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};


uint8_t output_state = 0x00;
uint8_t led_data = 0x00;



void xuat_1_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS((data & 0x80) != 0);
        XUNG_DICH();
        data <<= 1;
    }
}

void xuat_8led_7doan(uint8_t cot_hthi, uint8_t so_hthi)
{
    xuat_1_byte(led_data);                  
    xuat_1_byte(output_state);                 
    xuat_1_byte((uint8_t)~(1 << cot_hthi));    
    xuat_1_byte(so_hthi);                     
    XUNG_CHOT();
}

void giai_ma_quet(void)
{
	
	
uint16_t rssi_1=abs(rssi);
	
  led_7dq[0] = 0x88; 
	led_7dq[1] = 0x92; 
	led_7dq[2] = 0x92; 
	led_7dq[3] = 0xcf; 
	led_7dq[4] = 0xbf; 
	led_7dq[5] = ma7doan[rssi_1/100%10]; 
	led_7dq[6] = ma7doan[rssi_1/10%10]; 
	led_7dq[7] = ma7doan[rssi_1%10]; 
    
}

void TaskHienThi(void *pvParameters)
{
  

    while (1)
    {
			  giai_ma_quet(); 
        xuat_8led_7doan(tt8led, led_7dq[tt8led]);
        tt8led = (tt8led + 1) % 8;
		
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

void SetLedFade(uint8_t led_index, uint8_t brightness)
{
    if (brightness > 100) brightness = 100;

    const uint8_t pwm_cycles = 5;      // s? chu k? PWM cho m?i m?c sáng
    const uint16_t pwm_period = 2;     // m?i chu k? ~2ms

    for (uint8_t cycle = 0; cycle < pwm_cycles; cycle++)
    {
        uint16_t on_time = (brightness * pwm_period) / 100;
        uint16_t off_time = pwm_period - on_time;

        if (on_time)
        {
            led_data = (1 << led_index);
            vTaskDelay(pdMS_TO_TICKS(on_time));
        }

        led_data = 0x00;

        if (off_time)
            vTaskDelay(pdMS_TO_TICKS(off_time));
    }
}

void LedFadeChase(void)
{
    const uint16_t fade_step_delay = 30;  // m?i bu?c sáng/t?i 50ms (c?m nh?n du?c)

    for (uint8_t i = 0; i < 8; i++)
    {
        // Sáng d?n
        for (uint8_t b = 0; b <= 100; b += 5)
        {
            SetLedFade(i, b);
            vTaskDelay(pdMS_TO_TICKS(fade_step_delay));  // gi? m?i m?c sáng lâu hon
        }
        // T?i d?n
        for (int8_t b = 100; b >= 0; b -= 5)
        {
            SetLedFade(i, b);
            vTaskDelay(pdMS_TO_TICKS(fade_step_delay));
        }
    }
}


void TaskHieuUngLed(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        // Hi?u ?ng 1: LED ch?y t? trái sang ph?i
        for (uint8_t i = 0; i < 8; i++)
        {
            led_data = (1 << i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Hi?u ?ng 2: LED ch?y t? ph?i sang trái
        for (int8_t i = 7; i >= 0; i--)
        {
            led_data = (1 << i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Hi?u ?ng 3: Nh?p nháy toàn b?
        for (uint8_t i = 0; i < 5; i++)
        {
            led_data = 0xFF;
            vTaskDelay(pdMS_TO_TICKS(200));
            led_data = 0x00;
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Hi?u ?ng 4: Sáng d?n t?ng LED t? trái
        uint8_t temp = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            temp |= (1 << i);
            led_data = temp;
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Hi?u ?ng 5: T?t d?n t?ng LED t? ph?i
        for (int8_t i = 7; i >= 0; i--)
        {
            temp &= ~(1 << i);
            led_data = temp;
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Hi?u ?ng 6: LED b?t d?i x?ng vào gi?a
        for (uint8_t i = 0; i < 4; i++)
        {
            led_data = (1 << i) | (1 << (7 - i));
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Hi?u ?ng 7: LED ch?p nháy 01010101 <=> 10101010
        for (uint8_t i = 0; i < 4; i++)
        {
            led_data = 0x55;
            vTaskDelay(pdMS_TO_TICKS(200));
            led_data = 0xAA;
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Hi?u ?ng 8: Quét ki?u ti?n - lùi 2 bit
        for (uint8_t i = 0; i <= 6; i++)
        {
            led_data = (3 << i);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        for (int8_t i = 5; i >= 0; i--)
        {
            led_data = (3 << i);
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Hi?u ?ng 9: Sóng cu?n vào gi?a r?i ra ngoài
        for (uint8_t i = 0; i < 4; i++)
        {
            led_data |= (1 << i) | (1 << (7 - i));
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        for (int8_t i = 3; i >= 0; i--)
        {
            led_data &= ~((1 << i) | (1 << (7 - i)));
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Hi?u ?ng 10: LED ch?y 2 bit li?n k? ki?u xoay vòng
        const uint8_t patterns[] = {0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x81};
        for (uint8_t i = 0; i < sizeof(patterns); i++)
        {
            led_data = patterns[i];
            vTaskDelay(pdMS_TO_TICKS(150));
        }
				
				// Hi?u ?ng 11: Sáng d?n t? ngoài vào trong
				uint8_t pattern = 0x00;
				for (uint8_t i = 0; i < 4; i++)
				{
						pattern |= (1 << i) | (1 << (7 - i));
						led_data = pattern;
						vTaskDelay(pdMS_TO_TICKS(150));
				}

				// Hi?u ?ng 12: T?t d?n t? trong ra ngoài
				for (int8_t i = 3; i >= 0; i--)
				{
						pattern &= ~((1 << i) | (1 << (7 - i)));
						led_data = pattern;
						vTaskDelay(pdMS_TO_TICKS(150));
				}
				
				
				// Hi?u ?ng 13: LED xen k? b?t d?n vào gi?a
				const uint8_t pattern_in[] = {0x81, 0x42, 0x24, 0x18};
				for (uint8_t i = 0; i < 4; i++)
				{
						led_data = pattern_in[i];
						vTaskDelay(pdMS_TO_TICKS(200));
					
				// Hi?u ?ng 14: LED d?n ra ngoài r?i t?t
				const uint8_t pattern_out[] = {0x18, 0x24, 0x42, 0x81, 0x00};
				for (uint8_t i = 0; i < 5; i++)
				{
						led_data = pattern_out[i];
						vTaskDelay(pdMS_TO_TICKS(200));
				}
				
				// Hi?u ?ng 15: T?ng c?p d?i x?ng sáng r?i t?t
				for (uint8_t i = 0; i < 4; i++)
				{
						led_data = (1 << i) | (1 << (7 - i));
						vTaskDelay(pdMS_TO_TICKS(200));
						led_data = 0x00;
						vTaskDelay(pdMS_TO_TICKS(100));
				}


}

				
    }
}

static void relay1_on(void)
{
	output_state|=(1<<RELAY1_BIT);
}

static void relay1_off(void)
{
	output_state&=~(1<<RELAY1_BIT);
}

static void relay2_on(void)
{
	output_state|=(1<<RELAY2_BIT);
}

static void relay2_off(void)
{
	output_state&=~(1<<RELAY2_BIT);
}


void TaskRelay(void *pvParameters)
{
    while (1)
    {
      relay1_on();
			relay2_on();
      vTaskDelay(pdMS_TO_TICKS(1000));
			relay1_off();
			relay2_off();
			vTaskDelay(pdMS_TO_TICKS(1000));
    }
}