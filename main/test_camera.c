#include <stdio.h>
#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TXD_PIN (GPIO_NUM_7)
#define RXD_PIN (GPIO_NUM_6)

#define MATRIX_ROW 4 // 矩阵大小
#define MATRIX_COL 4 // 矩阵大小

#define UART_NUM UART_NUM_1

#define BUF_SIZE 1024
uint8_t uart_buffer[BUF_SIZE];//全局缓冲区，用于接收最新数据
SemaphoreHandle_t buffer_mutex;

uint8_t new_martix[MATRIX_ROW][MATRIX_COL];
uint8_t temp_martix[MATRIX_ROW][MATRIX_COL];

static const char *Task = "test";

void initialUart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// void print_depth_data(const uint8_t *data, int length) {
//     for (int i = 0; i < length; i++) {
//         printf("0x%02x ", data[i]);
//         if ((i + 1) % 16 == 0) {
//             printf("\n");
//         }
//     }
//     printf("\n");
// }

void send_command(const char* cmd) {
    uart_write_bytes(UART_NUM, cmd, strlen(cmd));
}

uint16_t unpack_uint16(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void copy_array_segment(const uint8_t *source, uint8_t *destination, int start, int end) {
    if (start < 0 || end < 0 || start > end) {
        printf("Invalid indices\n");
        return;
    }
    int length = end - start + 1; 
    memcpy(destination, source + start, length);
}


//暂定深度相机接收到的是25*25深度图
/**
 * depth_size是深度相机发出的深度图大小
 * region_size是指区域大小，按照几乘以几的区域取平均
 * 最右边一列和最下边一行舍去
 */
void convert_and_process_data(int depth_size, int region_size) {
    // 提取前625个元素并转换为25x25的二维数组
    uint8_t array_25x25[depth_size][depth_size];
    for (int i = 0; i < depth_size; ++i) {
        for (int j = 0; j < depth_size; ++j) {
            array_25x25[i][j] = uart_buffer[i * depth_size + j];
        }
    }

    // 将25x25的二维数组转换为4x4的二维数组
    for (int i = 0; i < MATRIX_ROW; ++i) {
        for (int j = 0; j < MATRIX_COL; ++j) {
            float sum = 0;
            for (int m = 0; m < region_size; ++m) {
                for (int n = 0; n < region_size; ++n) {
                    int row = i * region_size + m;
                    int col = j * region_size + n;
                    if (row < depth_size && col < depth_size) {
                        sum += array_25x25[row][col];
                    }
                }
            }
            new_martix[i][j] = 31 - sum / (region_size * region_size * 8); // 计算平均值
        }
    }

    // 打印4x4的二维数组
    printf("4x4 array:\n");
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            printf("%u ", new_martix[i][j]);
        }
        printf("\n");
    }
}

void process_data(const uint8_t *data, int length) {
    int start_position = -1;
    for (int i = 0; i < length - 3; ++i) {
        if(data[i]==0x00 &&data[i+1]==0xff && data[i+2]==0x81 && data[i+3]==0x02){
            start_position = i;
            break;
        }
    }
    ESP_LOGI(Task, "start_position %d :", start_position);
    if(start_position < 0){
        return;
    }
    else{//找到当前的深度图可能的起始位置
        int dataLen = unpack_uint16(&data[start_position + 2]);//剩余数据长度   641
        // ESP_LOGI(Task, "数据包的长度为 %d :", dataLen);
        int frameLen = 2 + 2 + dataLen + 2;//该深度图数据总长
        // ESP_LOGI(Task, "frameLen %d :", frameLen);
        // int framedataLen = dataLen - 16;//图像帧长度
        if(frameLen > length - start_position){
            return;
        }
        ESP_LOGI(Task, "data[end]=0x %02x :", data[start_position + frameLen - 1]);
        if(data[start_position + frameLen - 1] == 0xdd){//以0xdd结尾
        //后续考虑将代码改为自动获取数据帧的大小是几乘几
            //将从start_position+1+2+16+1是图像帧的起点
            //终点是start_position + frameLen - 2

        xSemaphoreTake(buffer_mutex, portMAX_DELAY);
        copy_array_segment(data, uart_buffer, start_position + 2 + 2 + 16, start_position + frameLen - 2);    
        convert_and_process_data(25, 6);
        xSemaphoreGive(buffer_mutex);
        }
    }
}


void uart_task(void *arg) {
    uint8_t data[BUF_SIZE];
    static const char *RX_TASK_TAG = "UART_TASK";
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        ESP_LOGI(RX_TASK_TAG, "Read %d bytes:", len);
        if (len > 0) {
            process_data(data, len);         
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void process_task(void *arg) {
    while (1) {
        xSemaphoreTake(buffer_mutex, portMAX_DELAY);
        memcpy(temp_martix, new_martix, sizeof(temp_martix));
        xSemaphoreGive(buffer_mutex);
         // 处理数据帧
        printf("Processing frame:\n");
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                printf("%u ", temp_martix[i][j]);
            }
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟0.1秒 
    }
}

// void app_main(void) {
//     initialUart();
//     send_command("AT+BINN=4\r\n");
//     send_command("AT+BAUD=2\r\n");
//     send_command("AT+FPS=1\r\n");
//     send_command("AT+DISP=4\r\n");

//     buffer_mutex = xSemaphoreCreateMutex();
//     if (buffer_mutex == NULL) {
//         ESP_LOGE(Task, "Failed to create buffer mutex");
//         return;
//     }
//     memset(new_martix, 0, sizeof(new_martix));
//     xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
//     xTaskCreate(process_task, "process_task", 4096, NULL, 10, NULL);
// }
