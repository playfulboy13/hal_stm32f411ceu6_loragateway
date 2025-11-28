#include "lora_gateway.h"

LoRa myLoRa;
int g_count = 0;
float g_temp = 0.0f;
int rssi = 0;

char debug[256];

#define FRAME_START   0xAA
#define FRAME_END     0x55

#define CMD_READ_REQ  0x00
#define CMD_READ_ACK  0x01


#define MAX_RETRY       3
#define CMD_ACK         0xA1

// ===============================
//   DANH SÁCH 3 NODE
// ===============================
uint8_t node_list[] = {0x01, 0x02, 0x03};
#define NODE_COUNT      (sizeof(node_list)/sizeof(node_list[0]))
	


NodeData_t g_node_data[3];

sensor_data_t sensor_data;
sensor_data_t sensor_data2;
sensor_data_t sensor_data3;

void lora_init_config(void)
{
		myLoRa = newLoRa();
    myLoRa.CS_port         = NSS_GPIO_Port;
    myLoRa.CS_pin          = NSS_Pin;
    myLoRa.reset_port      = RST_GPIO_Port;
    myLoRa.reset_pin       = RST_Pin;
    myLoRa.DIO0_port       = DIO0_GPIO_Port;
    myLoRa.DIO0_pin        = DIO0_Pin;
    myLoRa.hSPIx           = &hspi1;

    myLoRa.frequency             = 435;
    myLoRa.spredingFactor        = SF_7;
    myLoRa.bandWidth             = BW_125KHz;
    myLoRa.crcRate               = CR_4_5;
    myLoRa.power                 = POWER_20db;
    myLoRa.overCurrentProtection = 100;
    myLoRa.preamble              = 8;

    LoRa_init(&myLoRa);
}

uint8_t received_data[100];
uint8_t packet_size = 0;

void TaskReceiveLora(void *pvParameters)
{
    LoRa_init(&myLoRa);
    LoRa_startReceiving(&myLoRa);

    uint8_t rx_buf[100];
    char uart_buffer[128];
    char hex_buffer[256];  // Chu?i ch?a HEX g?i sang ESP32

    for (;;)
    {
        if (lora_flag)
        {
            lora_flag = 0;
            memset(rx_buf, 0, sizeof(rx_buf));

            uint8_t size = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));
            int16_t lora_rssi = LoRa_getRSSI(&myLoRa);

            if (size == 16 && rx_buf[0] == 0xAA && rx_buf[15] == 0x55)
            {
                uint8_t node_id = rx_buf[1];
                uint8_t cnt = rx_buf[2];
                float temp, humi;
                uint8_t hh, mm, ss;

                memcpy(&temp, &rx_buf[3], sizeof(float));
                memcpy(&humi, &rx_buf[7], sizeof(float));
                hh = rx_buf[11];
                mm = rx_buf[12];
                ss = rx_buf[13];

                // --- Ki?m tra checksum ---
                uint8_t checksum = 0;
                for (int i = 1; i <= 13; i++)
                    checksum ^= rx_buf[i];

                if (checksum == rx_buf[14])
                {
                    // --- Chuy?n sang chu?i HEX g?i qua UART ---
                    hex_buffer[0] = '\0';
                    for (int i = 0; i < size; i++)
                    {
                        char byte_str[4];
                        snprintf(byte_str, sizeof(byte_str), "%02X ", rx_buf[i]);
                        strcat(hex_buffer, byte_str);
                    }

                    // G?i thêm giá tr? RSSI ? cu?i (gi?ng ESP32 s? d?c)
                    snprintf(uart_buffer, sizeof(uart_buffer), "%d\r\n", lora_rssi);
                    strcat(hex_buffer, uart_buffer);

                    // --- G?i ra UART ---
                    HAL_UART_Transmit(&huart1, (uint8_t *)hex_buffer, strlen(hex_buffer), HAL_MAX_DELAY);

                    // --- C?p nh?t bi?n toàn c?c ---
                    g_count = cnt;
                    rssi = lora_rssi;
                }
                else
                {
                    const char *err_msg = "Checksum error!\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t *)err_msg, strlen(err_msg), HAL_MAX_DELAY);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void TaskSendLora(void *pvParameters)
{
	    uint16_t LoRa_status = LoRa_init(&myLoRa);

    uint8_t tx_buffer[32];
    uint8_t node_id = 0xA1;  // Node ID
    float temp = 25.0f;
    float humidity = 60.0f;  // Gi? l?p d? ?m ban d?u

    while (1)
    {
        // Gi? l?p c?m bi?n
      

        humidity += ((rand() % 100) - 50) / 200.0f;
        if (humidity < 40.0f) humidity = 40.0f;
        if (humidity > 90.0f) humidity = 90.0f;

        // Ðóng gói
        tx_buffer[0] = 0xAA;         // Start
        tx_buffer[1] = 23;      // Node ID
        tx_buffer[2] = 35;        // Counter

        memcpy(&tx_buffer[3], &temp, sizeof(float));       // Nhi?t d?
        memcpy(&tx_buffer[7], &humidity, sizeof(float));   // Ð? ?m

        tx_buffer[11] = 23;
        tx_buffer[12] = 56;
        tx_buffer[13] = 46;

        // Checksum XOR t? byte 1 ? 13
        uint8_t checksum = 0;
        for (int i = 1; i <= 13; i++)
            checksum ^= tx_buffer[i];
        tx_buffer[14] = checksum;

        tx_buffer[15] = 0x55;        // End

        // G?i qua LoRa
        if (LoRa_transmit(&myLoRa, tx_buffer, 16, 100) == 1) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

       

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/////////////




uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void float_to_bytes(float f, uint8_t *buf)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = f;
    memcpy(buf, u.b, 4);
}

float bytes_to_float(uint8_t *buf)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    memcpy(u.b, buf, 4);
    return u.f;
}



static void uart_send_frame_for_esp32(uint8_t node_id, uint8_t *rx, int size, int rssi)
{
    // Format: AA <ID> 22 <32 bytes payload> <CNT> <PARAM> <RSSI>
    char line[200];
    int pos = 0;

    // Start of frame + Node ID + length
    pos += sprintf(line + pos, "AA %02X 22 ", node_id);

    // Append payload 32 bytes (8 float)
    for (int i = 3; i < 3 + 32; i++)  // 8 float × 4 byte = 32 byte
        pos += sprintf(line + pos, "%02X", rx[i]); // Không kho?ng tr?ng gi?a byte

    // Append CNT + PARAM
    pos += sprintf(line + pos, "%02X%02X", rx[3 + 32], rx[3 + 32 + 1]);

    // Append RSSI
    pos += sprintf(line + pos, " %d\r\n", rssi);

    // G?i UART
    HAL_UART_Transmit(&huart1, (uint8_t*)line, strlen(line), 50);
}

// =====================================================================
// ===================== SUPPORT: get index from node ID ===============
// =====================================================================

int node_index_from_id(uint8_t id)
{
    for (int i = 0; i < NODE_COUNT; i++)
    {
        if (node_list[i] == id) return i;
    }
    return -1;
}

// =====================================================================
// ========================= TASK MASTER OPTIMIZED ======================
// =====================================================================

void TaskMaster(void *pvParameters)
{
    (void)pvParameters;
    lora_init_config();

    uint8_t rx_buf[64];
    uint8_t size;

    for (;;)
    {
        for (int n = 0; n < NODE_COUNT; n++)
        {
            uint8_t node_id = node_list[n];
            bool success = false;

            // === TÍNH TOTAL PACKET m?i l?n POLL 1 NODE ===
            total_packets++;

            for (int retry = 0; retry < MAX_RETRY; retry++)
            {
                LoRa_startReceiving(&myLoRa);
                vTaskDelay(pdMS_TO_TICKS(8));

                uint8_t request[4] = {FRAME_START, node_id, CMD_READ_REQ, FRAME_END};
                LoRa_transmit(&myLoRa, request, 4, 100);

                char log_tx[80];
                sprintf(log_tx, "MASTER: Request sent to NODE %02X (retry=%d)\r\n",
                        node_id, retry);
                HAL_UART_Transmit(&huart1, (uint8_t*)log_tx, strlen(log_tx), 50);

                uint32_t t0 = xTaskGetTickCount();
                const TickType_t timeout_ticks = pdMS_TO_TICKS(500);
                bool received = false;

                while ((xTaskGetTickCount() - t0) < timeout_ticks)
                {
                    // =============================
                    //    CÓ FRAME M?I T? INTERRUPT
                    // =============================
                    if (lora_flag)
                    {
                        taskENTER_CRITICAL();
                        lora_flag = 0;
                        taskEXIT_CRITICAL();

                        // Ð?c LIÊN T?C t?t c? gói trong FIFO
                        while (1)
                        {
                            size = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));
                            if (size == 0)
                                break; // không còn gói nào n?a

                            rssi = LoRa_getRSSI(&myLoRa);
                            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

                            if (rx_buf[0] != FRAME_START || rx_buf[size - 1] != FRAME_END)
                                continue;

                            // ======== DATA FRAME ========
                            if (size >= 40 && rx_buf[1] == node_id)
                            {
                                uint8_t len = rx_buf[2];
                                if (len != 34) continue;

                                int crc_index = 3 + len;
                                if (crc_index + 1 >= size) continue;

                                uint16_t crc_recv =
                                    (rx_buf[crc_index] << 8) | rx_buf[crc_index + 1];

                                uint16_t crc_calc =
                                    crc16_ccitt(&rx_buf[1], 1 + 1 + len);

                                if (crc_calc != crc_recv)
                                    continue;

                                // ------- Parse Sensor -------
                                float sensor[8];
                                int offset = 3;
                                for (int i = 0; i < 8; i++)
                                {
                                    sensor[i] = bytes_to_float(&rx_buf[offset]);
                                    offset += 4;
                                }

                                uint8_t g_count      = rx_buf[offset++];
                                uint8_t custom_param = rx_buf[offset++];

                                int idx = node_index_from_id(node_id);
                                if (idx >= 0)
                                {
                                    taskENTER_CRITICAL();
                                    memcpy(g_node_data[idx].sensor, sensor, sizeof(sensor));
                                    g_node_data[idx].cnt    = g_count;
                                    g_node_data[idx].param  = custom_param;
                                    g_node_data[idx].rssi   = rssi;
                                    g_node_data[idx].online = 1;
                                    taskEXIT_CRITICAL();
                                }

                                uart_send_frame_for_esp32(node_id, rx_buf, size, rssi);

                                received = true;
                                success  = true;
                                break; // dúng DATA ? không c?n d?c ti?p
                            }

                            // ======== ACK FRAME ========
                            if (size == 4 &&
                                rx_buf[1] == node_id &&
                                rx_buf[2] == CMD_ACK)
                            {
                                // ch? b? qua ACK, không break
                                continue;
                            }
                        }
                    }

                    if (received)
                        break;

                    vTaskDelay(pdMS_TO_TICKS(5));
                }

                if (received)
                    break;

                // ==== thông báo retry ====
                char debug[80];
                sprintf(debug, "MASTER: Retry %d for NODE %02X...\r\n",
                        retry + 1, node_id);
                HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 30);

                vTaskDelay(pdMS_TO_TICKS(80));
            }

            // ===============================
            // NODE KHÔNG TR? L?I ? LOST PACKET
            // ===============================
            if (!success)
            {
                lost_packets++;

                char debug[80];
                sprintf(debug, "MASTER: NO RESPONSE FROM NODE %02X!\r\n", node_id);
                HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 30);

                int idx = node_index_from_id(node_id);
                if (idx >= 0)
                {
                    taskENTER_CRITICAL();
                    g_node_data[idx].online = 0;
                    g_node_data[idx].rssi   = -127;
                    taskEXIT_CRITICAL();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}




///////////////////////////////////////////////////////////////////////////









