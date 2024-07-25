#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include <time.h>
//通信部分
#define RX_BUF_SIZE       1024//接收缓冲区大小8X8

#define FEN_CENG          4
#define STRENGTH_LEN      32
#define DISPLAY_LEN       1061

#define TXD_PIN (GPIO_NUM_8)//定义要操作的引脚
#define RXD_PIN (GPIO_NUM_9)


#define MATRIX_ROW 4 // 矩阵大小
#define MATRIX_COL 4 // 矩阵大小

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

uint8_t temp_martix[MATRIX_ROW][MATRIX_COL];    // 保存现在正在处理的矩阵,也是要做减法的

uint8_t motor_martix[MATRIX_ROW][MATRIX_COL];   // 保留正在处理的矩阵，只是为了拷贝
uint8_t fenceng[FEN_CENG] = {2, 10, 15, 18};


//1号串口做通信
void initialUart(){
     const uart_config_t uart_config = {
        .baud_rate = 115200,                    //设置波特率    115200
        .data_bits = UART_DATA_8_BITS,          //设置数据位    8位
        .parity = UART_PARITY_DISABLE,          //设置奇偶校验  不校验
        .stop_bits = UART_STOP_BITS_1,          //设置停止位    1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //设置硬件流控制 不使能
        .source_clk = UART_SCLK_APB,            //设置时钟源
    };
    //安装串口驱动 串口编号、接收buff、发送buff、事件队列、分配中断的标志
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    //串口参数配置 串口号、串口配置参数
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    //设置串口引脚号 串口编号、tx引脚、rx引脚、rts引脚、cts引脚,后四个不使用所以指定成nochange
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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


void motor_work(void* arg){//强度0-32，实现类似pwm的效果

    static const char *RX_TASK_TAG = "MOTOR_TASK";                 //日志信息的标签 接收任务
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);           //设置log打印

    uint16_t count_fenceng = 0;
    uint16_t test_count = 0;
    uint16_t loop_count = 0;
    memset(new_martix, 0, sizeof(new_martix));
    memset(control_martix, 0, sizeof(control_martix));
    memset(temp_martix, 0, sizeof(temp_martix));
    memset(motor_martix, 0, sizeof(motor_martix));
    uint8_t depth_count = 0;

    time_t start_time, current_time;
    time(&start_time);
    for(int i = 0; i < MATRIX_ROW; i++){
        for(int j = 0; j < MATRIX_COL; j++){
            // new_martix[i][j] = 0;
            new_martix[i][j] = 32 + (i + 2) * (j + 2);
            // printf("new_martix[%d][%d] = %d\n", i, j, new_martix[i][j]);
        }
    }
    depth_count ++;
    //或许这改为一个bool类型的变量
    while (1)
    {   
        if(count_fenceng % (STRENGTH_LEN * DISPLAY_LEN) == 0 && depth_count){//当前的图片处理完了
            if(count_fenceng != 0){
                // depth_count --;
                count_fenceng = 0;
            }
            memcpy(temp_martix,new_martix,sizeof(temp_martix));//temp_matix存的是正在处理的图片，当为32*4的倍数时说明当前图片处理完成就开始处理最新的一张图片
            memcpy(motor_martix,new_martix,sizeof(motor_martix));
        }
        if(count_fenceng % STRENGTH_LEN == 0){
            memcpy(temp_martix,motor_martix,sizeof(temp_martix));
            memcpy(control_martix,temp_martix,sizeof(control_martix));
        }
        if(depth_count){
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
                    }else{
                        if(count_fenceng >= STRENGTH_LEN){
                            if(temp_martix[i][j]>32){
                                temp_martix[i][j] -= 32;
                                control_martix[i][j] -= 32;
                            }
                        }
                    }
                    if(count_fenceng < STRENGTH_LEN){
                        ROW_Data_High();    
                        // ROW_Data_Low();                   
                    }else{
                        // printf("temp%d\n",temp_martix[i][j]);
                        // printf("test%d\n",fenceng[(count_fenceng -32)/ (STRENGTH_LEN * ((DISPLAY_LEN - 1) / FEN_CENG))]);
                        //对的，这个if逻辑是有问题的
                        if(temp_martix[i][j] > fenceng[(count_fenceng -32)/ (STRENGTH_LEN * ((DISPLAY_LEN - 1) / FEN_CENG))]){
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
        test_count ++;
        loop_count ++;
        time(&current_time);
        if(difftime(current_time, start_time) >= 1.0){
            printf("Loop in 1 second:%d", loop_count);
            loop_count = 0;
            time(&start_time);
        }
    }
}

void app_main(void)
{   
    initialUart();
    initialMotor();
    xTaskCreate(motor_work, "motor_task", 1024*5, NULL, 10, NULL);
}