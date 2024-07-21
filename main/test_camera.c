#include <stdio.h>
#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define TXD_PIN (GPIO_NUM_7)
#define RXD_PIN (GPIO_NUM_6)

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define QUEUE_SIZE 10

QueueHandle_t data_queue;
SemaphoreHandle_t data_queue_semaphore;

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
    uint16_t value;
    memcpy(&value, data, sizeof(uint16_t));
    return value;
}

void copy_array_segment(const uint8_t *source, uint8_t *destination, int i, int j) {
    if (i < 0 || j < 0 || i > j) {
        printf("Invalid indices\n");
        return;
    }
    int length = j - i + 1; 
    memcpy(destination, source + i, length);
}

void process_data(const uint8_t *data, int length) {
    uint8_t buffer[BUF_SIZE];
    memset(buffer, 0, sizeof(buffer));
    int buffer_len = 0;
    int start_position = -1;
    for (int i = 0; i < length - 3; ++i) {
        buffer[buffer_len++] = data[i];
        if(data[i]==0x00 &&data[i+1]==0xff && data[i+2]==0x81 && data[i+3]==0x02){
            start_position = i;
            break;
        }
    }
    ESP_LOGI(Task, "start_position %d :", start_position);
    if(start_position < 0){
        return;
    }
    else{//找到当前的深度图
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
        // ESP_LOGI(Task, "拷贝数据长度为 %d :", (start_position + frameLen - 2)-(start_position+1+2+16+1));
        copy_array_segment(data, buffer, start_position+1+2+16+1, start_position + frameLen - 2);
        printf("Copied segment: ");
        // for (int k = 0; k < (start_position + frameLen - 2)-(start_position+1+2+16+1); k++) {
        //     printf("0x%02x ", buffer[k]);
        // }
        printf("\n");
        if (xQueueSend(data_queue, buffer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE("UART", "Failed to send data to queue");
        }
        }
    }
}


void uart_task(void *arg) {
    uint8_t data[BUF_SIZE];
    static const char *RX_TASK_TAG = "UART_TASK";
    int count = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        ESP_LOGI(RX_TASK_TAG, "Read %d bytes:", len);
        if (len > 0) {
            // print_depth_data(data, len);
            if(count == 0){
                count = 1;
            }else{
                process_data(data, len);
            }         
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void process_task(void *arg) {
    uint8_t frame[BUF_SIZE];
    while (1) {
        if (xQueueReceive(data_queue, frame, portMAX_DELAY)) {
            // 处理数据帧
            printf("Processing frame:\n");
            for (int i = 0; i < BUF_SIZE; ++i) {
                printf("0x%02x ", frame[i]);
                if ((i + 1) % 25 == 0) {
                    printf("\n");
                }
            }
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // 延迟0.1秒 
    }
}

void app_main(void) {
    initialUart();
    send_command("AT+BINN=4\r\n");
    send_command("AT+BAUD=2\r\n");
    send_command("AT+FPS=10\r\n");
    send_command("AT+DISP=4\r\n");

    data_queue = xQueueCreate(QUEUE_SIZE, BUF_SIZE);//存放队列
    if (data_queue == NULL) {
        ESP_LOGE(Task, "Failed to create data queue");
        return;
    }

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(process_task, "process_task", 4096, NULL, 10, NULL);
}




//使用中断方式
// #include <stdio.h>
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"

// #include"string.h"
// #define TXD_PIN (GPIO_NUM_7)  // 定义 UART TX 引脚
// #define RXD_PIN (GPIO_NUM_6)  // 定义 UART RX 引脚

// #define UART_NUM UART_NUM_1   // 使用 UART1
// #define BUF_SIZE 1024         // 缓冲区大小
// #define QUEUE_SIZE 10         // 队列大小

// static QueueHandle_t data_queue;             // 数据队列句柄
// static SemaphoreHandle_t data_queue_semaphore;  // 队列互斥信号量句柄
// static QueueHandle_t uart_queue;             // UART 事件队列

// void initialUart(){
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,                    // 设置波特率
//         .data_bits = UART_DATA_8_BITS,          // 设置数据位
//         .parity = UART_PARITY_DISABLE,          // 设置奇偶校验
//         .stop_bits = UART_STOP_BITS_1,          // 设置停止位
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 设置硬件流控制
//         .source_clk = UART_SCLK_APB,            // 设置时钟源
//     };
//     // 安装 UART 驱动并创建事件队列
//     ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, QUEUE_SIZE, &uart_queue, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
//     ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM));  // 启用 UART 接收中断
// }

// void uart_event_task(void *pvParameters) {
//     uart_event_t event;
//     uint8_t data[BUF_SIZE];

//     while (1) {
//         // 等待 UART 事件
//         if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
//             if (event.type == UART_DATA) {
//                 int len = uart_read_bytes(UART_NUM, data, event.size, portMAX_DELAY);
//                 if (len > 0) {
//                     xSemaphoreTake(data_queue_semaphore, portMAX_DELAY);
//                     //这里写处理数据
//                     if (xQueueSend(data_queue, data, portMAX_DELAY) != pdPASS) {
//                         ESP_LOGE("UART", "Failed to send data to queue");
//                     }
//                     xSemaphoreGive(data_queue_semaphore);
//                 }
//             }
//         }
//     }
// }

// void process_task(void *arg) {
//     uint8_t frame[BUF_SIZE];
//     while (1) {
//         if (xQueueReceive(data_queue, frame, portMAX_DELAY)) {
//             // 处理数据帧
//             printf("Processing frame:\n");
//             for (int i = 0; i < BUF_SIZE; ++i) {
//                 printf("0x%02x ", frame[i]);
//             }
//             printf("\n");
//         }
//          vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1秒 
//     }
// }

// void send_command(const char* cmd) {
//     uart_write_bytes(UART_NUM, cmd, strlen(cmd));
// }

// void app_main(void) {
//     initialUart();

//     // 配置深度相机
//     send_command("AT+BINN=4\r\n"); // 出图限制到25*25
//     send_command("AT+BAUD=2\r\n");
//     send_command("AT+FPS=10\r\n");
//     send_command("AT+DISP=4\r\n"); // 开启 UART

//     // 创建队列和信号量
//     data_queue = xQueueCreate(QUEUE_SIZE, BUF_SIZE);
//     data_queue_semaphore = xSemaphoreCreateMutex();

//     if (data_queue == NULL || data_queue_semaphore == NULL) {
//         ESP_LOGE("MAIN", "Failed to create queue or semaphore");
//         return;
//     }

//     // 创建任务
//     xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
//     xTaskCreate(process_task, "process_task", 4096, NULL, 10, NULL);


// }







// #include <stdio.h>
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include <string.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include <time.h>

// //接法应该是rx接到esp32-c3的tx上
// #define TXD_PIN (GPIO_NUM_7)//定义要操作的引脚
// #define RXD_PIN (GPIO_NUM_6)

// #define UART_NUM      UART_NUM_1
// #define BUF_SIZE      1024


// void initialUart(){
//      const uart_config_t uart_config = {
//         .baud_rate = 115200,                    //设置波特率    115200
//         .data_bits = UART_DATA_8_BITS,          //设置数据位    8位
//         .parity = UART_PARITY_DISABLE,          //设置奇偶校验  不校验
//         .stop_bits = UART_STOP_BITS_1,          //设置停止位    1
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //设置硬件流控制 不使能
//         .source_clk = UART_SCLK_APB,            //设置时钟源
//     };
//     //安装串口驱动 串口编号、接收buff、发送buff、事件队列、分配中断的标志
//     ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
//     //串口参数配置 串口号、串口配置参数
//     ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
//     //设置串口引脚号 串口编号、tx引脚、rx引脚、rts引脚、cts引脚,后四个不使用所以指定成nochange
//     ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
// }


// void print_depth_data(const uint8_t *data, int length)
// {
//     for (int i = 0; i < length; i++) {
//         printf("0x%02x ", data[i]);
//         if ((i + 1) % 16 == 0) {
//             printf("\n");
//         }
//     }
//     printf("\n");
// }

// void send_command(const char* cmd) {
//     uart_write_bytes(UART_NUM, cmd, strlen(cmd));
// }

// void app_main(void)
// {
//     initialUart();
//     // 配置深度相机
//     send_command("AT+BINN=4\r\n");//出图限制到25*25
//     send_command("AT+BAUD=2\r\n");
//     send_command("AT+FPS=10\r\n");
//     send_command("AT+DISP=4\r\n");//开启uart
//     uint8_t data[BUF_SIZE];
//     static const char *RX_TASK_TAG = "MOTOR_TASK";  
//     while (1) {
//         int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 0);
//         ESP_LOGI(RX_TASK_TAG, "Read %d bytes:", len);
//         if (len > 0) {
//             print_depth_data(data, len);
//             //首先看一眼这里的打印，
//             // 如果打印东西过多的话就取前面的字节打印
//         }
//         //我应该是首先打印出来数据，看看data里的内容，
//         // 之后在考虑该如何写读取数据
//         // int data_len = data[2];
//         //图像的数据长度位置是要手动计算出来的，方便后续用矩阵存储
//         //这里应该写我的处理逻辑
//         vTaskDelay(5000/portTICK_PERIOD_MS);
//     }
// }
// // 包头2字节：0X00、0XFF
// // 包长度2字节：当前包剩余数据的字节数
// // 其他内容16字节：包括包序号、包长度、分辨率等等
// // 图像帧
// // 校验1字节：之前所有字节的“和”低八位
// // 包尾1字节：0XDD