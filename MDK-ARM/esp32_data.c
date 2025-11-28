#include "esp32_data.h" // gi? nguyên n?u b?n c?n typedef g_node_data, huart2...

uint16_t lost_packets = 0;   // ví d?: d?m gói tin m?t
uint16_t total_packets = 0;  // t?ng gói dã g?i

uint16_t CRC16_Modbus(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for(uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];
        for(int i = 0; i < 8; i++)
        {
            if(crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void encrypt_xor(uint8_t *buf, uint16_t len, uint8_t key)
{
    for(uint16_t i = 0; i < len; i++)
        buf[i] ^= key;
}

void TaskData(void *pvParameters)
{
    uint8_t frame[256];      // buffer cho frame
    const uint8_t xor_key = 0x5A;

    while(1)
    {
        uint16_t pos = 0;

        // HEADER
        frame[pos++] = 0xAA;
        frame[pos++] = 0x55;

        // LEN placeholder (1 byte)
        uint16_t len_index = pos;
        frame[pos++] = 0; // fill later

        uint16_t payload_start = pos;

        // 3 NODE, m?i node: CNT(1) + RSSI(1) + 8*4 bytes float = 34 bytes
        for(uint8_t n = 0; n < 3; n++)
        {
            float *s = g_node_data[n].sensor; // gi? s? g_node_data[n].sensor[8]

            // CNT
            frame[pos++] = g_node_data[n].cnt;

            // RSSI (int8)
            frame[pos++] = (int8_t)g_node_data[n].rssi;

            // 8 floats (little-endian)
            for(uint8_t i = 0; i < 8; i++)
            {
                float val = s[i];
                memcpy(&frame[pos], &val, sizeof(float));
                pos += sizeof(float);
            }
        }

        // ---- 5 c?m bi?n gi? d?nh ----
        float fake_sensors[5] = { 12.3f, 45.6f, 78.9f, 23.4f, 56.7f };
        for(uint8_t i = 0; i < 5; i++)
        {
            memcpy(&frame[pos], &fake_sensors[i], sizeof(float));
            pos += sizeof(float);
        }

        // ---- Thông tin LoRa ----
        total_packets++;
        // Gi? l?p lost rate = lost_packets / total_packets * 100 (%)
        uint8_t lost_rate = (total_packets == 0) ? 0 : (lost_packets * 100 / total_packets);
        frame[pos++] = lost_rate;          // 1 byte t? l? m?t gói (%)
        frame[pos++] = (uint8_t)total_packets; // t?ng gói g?i (1 byte, don gi?n)

        // payload length (fit 1 byte vì max v?n < 255)
        uint16_t payload_len = pos - payload_start;
        frame[len_index] = (uint8_t)payload_len;

        // XOR payload
        encrypt_xor(&frame[payload_start], payload_len, xor_key);

        // CRC over header+len+payload
        uint16_t crc = CRC16_Modbus(frame, (uint16_t)(3 + payload_len));
        frame[pos++] = (uint8_t)(crc & 0xFF);
        frame[pos++] = (uint8_t)((crc >> 8) & 0xFF);

        // Transmit (DMA)
        HAL_UART_Transmit_DMA(&huart2, frame, pos);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

