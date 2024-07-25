#include <stdio.h>
#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//这两行是需要修改的
#define TXD_PIN (GPIO_NUM_7)
#define RXD_PIN (GPIO_NUM_6)

#define MATRIX_ROW 4 // 矩阵大小
#define MATRIX_COL 4 // 矩阵大小

#define UART_NUM UART_NUM_1

#define BUF_SIZE 1024
uint8_t uart_buffer[BUF_SIZE];//全局缓冲区，用于接收最新数据
SemaphoreHandle_t buffer_mutex;

#define FEN_CENG          4
#define STRENGTH_LEN      32
#define DISPLAY_LEN       1061

//行引脚
#define LINE_SCK_GPIO_PIN	   13	 // 移位寄存器时钟线引脚                  			        
#define LINE_RCK_GPIO_PIN	   11   //存储寄存器时钟线引脚
#define LINE_SDA_GPIO_PIN      10    //数据引脚

//列引脚
#define ROW_SCK_GPIO_PIN	    2	 // 移位寄存器时钟线引脚                  			        
#define ROW_RCK_GPIO_PIN	    4   //存储寄存器时钟线引脚
#define ROW_SDA_GPIO_PIN        5    //数据引脚

//列控制
//左边是定义的宏，右边是具体的宏实现
#define LINE_SCK_Low()      gpio_set_level(LINE_SCK_GPIO_PIN,0)     //SCK置低
#define LINE_SCK_High()     gpio_set_level(LINE_SCK_GPIO_PIN,1)    //SCK置高

#define LINE_RCK_Low()      gpio_set_level(LINE_RCK_GPIO_PIN,0)   //RCK置低
#define LINE_RCK_High()     gpio_set_level(LINE_RCK_GPIO_PIN,1)   //RCK置高

#define LINE_Data_Low()      gpio_set_level(LINE_SDA_GPIO_PIN,0)  //输入低电平
#define LINE_Data_High()     gpio_set_level(LINE_SDA_GPIO_PIN,1)  //输入高电平

//行控制
#define ROW_SCK_Low()      gpio_set_level(ROW_SCK_GPIO_PIN,0)     //SCK置低
#define ROW_SCK_High()     gpio_set_level(ROW_SCK_GPIO_PIN,1)    //SCK置高

#define ROW_RCK_Low()      gpio_set_level(ROW_RCK_GPIO_PIN,0)   //RCK置低
#define ROW_RCK_High()     gpio_set_level(ROW_RCK_GPIO_PIN,1)   //RCK置高

#define ROW_Data_Low()      gpio_set_level(ROW_SDA_GPIO_PIN,0)  //输入低电平
#define ROW_Data_High()     gpio_set_level(ROW_SDA_GPIO_PIN,1)  //输入高电平

uint8_t control_martix[MATRIX_ROW][MATRIX_COL]; // 用于周期显示的矩阵，要做减法的

uint8_t new_martix[MATRIX_ROW][MATRIX_COL]; // 永远用于保存现在接收到的最新矩阵

uint8_t motor_martix[MATRIX_ROW][MATRIX_COL];   // 保留正在处理的矩阵，只是为了拷贝
uint8_t fenceng[FEN_CENG] = {2, 10, 15, 18};
uint8_t depth_count = 0;

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

//电机控制
void initialMotor(){
    gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL<<LINE_SCK_GPIO_PIN)|(1ULL<<LINE_RCK_GPIO_PIN)|(1ULL<<LINE_SDA_GPIO_PIN)|
            (1ULL<<ROW_SCK_GPIO_PIN)|(1ULL<<ROW_RCK_GPIO_PIN)|(1ULL<<ROW_SDA_GPIO_PIN),  
            .pull_down_en = 1,
            .pull_up_en = 0,
        };
    gpio_config(&io_conf);
    LINE_SCK_Low();
    LINE_RCK_Low();
    LINE_Data_Low();

    ROW_SCK_Low();
    ROW_RCK_Low();
    ROW_Data_Low();
}

void LINE_Save(){
    LINE_RCK_Low();
    LINE_RCK_High();
}

void ROW_Save(){
    ROW_RCK_Low();
    ROW_RCK_High();
}

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
            new_martix[i][j] = 31 - ((sum / (region_size * region_size ) )/ 8); // 计算平均值
        }
    }

    // 打印4x4的二维数组
    // printf("4x4 array:\n");
    // for (int i = 0; i < 4; ++i) {
    //     for (int j = 0; j < 4; ++j) {
    //         printf("%u ", new_martix[i][j]);
    //     }
    //     printf("\n");
    // }
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
        depth_count = 1;
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

void motor_task(void *arg) {
    static const char *RX_TASK_TAG = "MOTOR_TASK";                 //日志信息的标签 接收任务
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);           //设置log打印

    uint16_t count_fenceng = 0;
    while (1) {
        if(depth_count){//获得深度图
            if(count_fenceng % (STRENGTH_LEN * DISPLAY_LEN) == 0){//当前的图片处理完了
                if(count_fenceng != 0){
                    count_fenceng = 0;
                }
                xSemaphoreTake(buffer_mutex, portMAX_DELAY);
                memcpy(motor_martix,new_martix,sizeof(motor_martix));
                xSemaphoreGive(buffer_mutex);
            }
            if(count_fenceng % STRENGTH_LEN == 0){
                memcpy(control_martix,motor_martix,sizeof(control_martix));
            }
            for(int i = 0; i<MATRIX_ROW; i++){//多少行
                LINE_SCK_Low();
                if(i==0){//首先将Data置高电平表示选中第一行  //之后选中其他行的逻辑就通过移位寄存器来实现了
                    LINE_Data_High();
                }else{
                    LINE_Data_Low();
                }
                LINE_SCK_High();//第一次就为1000,此时数据在移位寄存器中
                //这个时候时钟信号已经有了，就只需要考虑每一列的数据了
                for(int j=MATRIX_COL-1 ;j>=0; j--){//多少列
                    ROW_SCK_Low();//列移位寄存器
                    if(!count_fenceng){
                        control_martix[i][j] = 32;
                    }
                    if(count_fenceng < STRENGTH_LEN){
                        ROW_Data_High();    
                        // ROW_Data_Low();                   
                    }else{
                        if(motor_martix[i][j] > fenceng[(count_fenceng -32)/ (STRENGTH_LEN * ((DISPLAY_LEN - 1) / FEN_CENG))]){
                            if(control_martix[i][j]==0){
                                ROW_Data_Low();
                            }else{
                                ROW_Data_High();
                                control_martix[i][j]--;
                            }
                        }
                    }
                    ROW_SCK_High();
                }
                ROW_Save();
                LINE_Save();          
            }
            count_fenceng ++;       
            // 增加寄存器全部置0的操作
            for(int j=0;j<MATRIX_COL;j++){
                LINE_SCK_Low();
                LINE_Data_Low();
                LINE_SCK_High();
            }
            LINE_Save();
            for(int j=0;j<MATRIX_COL;j++){
                ROW_SCK_Low();                
                ROW_Data_Low();         
                ROW_SCK_High();
            }
            ROW_Save();
        }
    }
}

void app_main(void) {
    initialUart();
    initialMotor();
    send_command("AT+BINN=4\r\n");
    send_command("AT+BAUD=2\r\n");
    send_command("AT+FPS=1\r\n");
    send_command("AT+DISP=4\r\n");

    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL) {
        ESP_LOGE(Task, "Failed to create buffer mutex");
        return;
    }
    memset(new_martix, 0, sizeof(new_martix));
    memset(control_martix, 0, sizeof(control_martix));
    memset(motor_martix, 0, sizeof(motor_martix));
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 10, NULL);
}
