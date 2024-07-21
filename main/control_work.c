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

#define TXD_PIN (GPIO_NUM_8)//定义要操作的引脚
#define RXD_PIN (GPIO_NUM_9)
// #define TXD_PIN (GPIO_NUM_7)
// #define RXD_PIN (GPIO_NUM_6)



//电机控制部分
//#define MOTOR_SIZE 8//矩阵大小

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




uint8_t motor_martix[MATRIX_ROW][MATRIX_COL];//强度信息 0-32

uint8_t control_martix[MATRIX_ROW][MATRIX_COL];//每一轮写的数字

uint8_t new_martix[MATRIX_ROW][MATRIX_COL];//强度信息 0-32 暂存最新数据





//通信
//0号串口做通信    //这里难道不是用的1号串口做通信吗
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
 

    uint8_t count=0;
    uint8_t flag=0;
    uint8_t need_copy=0;//用来记录new的值有被复制吗
    //int big=0;
    memset(new_martix, 0, sizeof(new_martix));
    memset(control_martix, 0, sizeof(control_martix));
    memset(motor_martix, 0, sizeof(motor_martix));


/**
 *整体思路是 首先从uart通道中读取数据
 * 判断数据的长度是否为空，不为空就说明读取到了数据，数据就被存储到了new_martix中 强度信息
 * 并且将flag和need_copy都置为1
 * 当是32的倍数时，完成复制
 */

    while (1)
    {   
        //new_martix的含义是获得的深度图信息我觉得
        int length = 0;
        const int rxBytes = uart_read_bytes(UART_NUM_1, new_martix[0], RX_BUF_SIZE, 0);//读取数据
        //uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length);
        //if(length==(MATRIX_COL*MATRIX_ROW)){
            //const int rxBytes = uart_read_bytes(UART_NUM_2, new_martix[0],(MATRIX_COL*MATRIX_ROW), 0);
            if (rxBytes > 0) {                                     //判断数据长度                                
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes:", rxBytes);
                flag=1;
                need_copy=1;
                // 测试效果
                    // for(int i=0;i<4;i++){
                    //     for(int j=0;j<4;j++){
                    //         printf("%d ",new_martix[i][j]);
                    //         }
                    //     printf("\n");
                    // }

                    // printf("\n");
            }
        //}

        // for(int i=0;i<4;i++){
        //             for(int j=0;j<4;j++){
        //                 printf("%d ",new_martix[i][j]);
        //                 }
        //             printf("\n");
        //         }

        //         printf("\n");

        if (count%32==0){
            // if(flag==1){
            //     memcpy(motor_martix,new_martix,sizeof(motor_martix));
            //     flag=0;
            // }         
            memcpy(motor_martix,new_martix,sizeof(motor_martix));
            flag=0;   
            memcpy(control_martix,motor_martix,sizeof(motor_martix));
            time_t now_time;
            time(&now_time);    
            //vTaskDelay(1000);
            //printf("当前时间为%s ",ctime (&now_time));
            //printf("%d",big);
            need_copy=0;

        }


        //他这个事边缘触发   上升沿和下降延都可以触发
        for(int i=0;i<MATRIX_ROW;i++){//一行一行控制
            LINE_SCK_Low();// 行移位寄存器引脚
            if(i==0){//初始是低电平，所以当i等于0的时候置为高就为上升沿了
                LINE_Data_High();
            }else{
                LINE_Data_Low();//由于是每行保存，保存时置为高，所以要触发就要设置为低
            }
            LINE_SCK_High();//置为高就说明可以接收数据输入了

            for(int j=MATRIX_COL-1;j>=0;j--){

                ROW_SCK_Low();//列移位寄存器引脚

                if(flag==1){

                    int delta=new_martix[i][j]-(motor_martix[i][j]-control_martix[i][j]);//应转-已经转了
                    if(delta<=0){
                        //电机输出低电平
                        ROW_Data_Low();
                    }else{
                        //电机输出高电平
                        ROW_Data_High();
                        control_martix[i][j]=(uint8_t)(delta-1);
                    }
                }else{
                     if(control_martix[i][j]>32 && need_copy==0 && new_martix[i][j]>32){
                       new_martix[i][j]-=32;
                    }   
                    if(control_martix[i][j]==0){
                        //电机输出低电平
                        ROW_Data_Low();
                    }else{
                        //电机输出高电平
                        ROW_Data_High();
                        control_martix[i][j]--;
                    }                   
                }
                ROW_SCK_High();
            }
            ROW_Save();
            LINE_Save();
        }

        // 增加寄存器全部置0的操作

        count++;
        flag=0;

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

void testOnebyOne(void* arg){//强度0-32，实现类似pwm的效果



    static const char *RX_TASK_TAG = "MOTOR_TASK";                 //接收任务
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);           //设置log打印
 

    uint8_t count=0;
    uint8_t flag=0;
    //int big=0;

    while (1)
    {       
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
        
        
        for(int k=0;k<5;k++){
            for(int i=0;i<5;i++){

                int count=0;

                

                while (count<3200)
                {


                     for(int j=0;j<MATRIX_COL;j++){

                        LINE_SCK_Low();
                        LINE_Data_Low();
                        LINE_SCK_High();
                    }

                    LINE_Save();


                    for(int j=0;j<MATRIX_COL;j++){

                        ROW_SCK_Low();

                            if(j==(MATRIX_COL-1-i)){
                                //电机输出低电平
                                if(count<32){
                                    ROW_Data_High();
                                }
                                else if(count%32<=8){
                                ROW_Data_High();
                                }
                                else{
                                ROW_Data_Low();
                                }
                            }else{
                                //电机输出高电平
                                ROW_Data_Low();
                            }

                        ROW_SCK_High();
                
                    }
            
                    ROW_Save();

                    for(int j=0;j<MATRIX_COL;j++){

                        LINE_SCK_Low();
                        if(j==(MATRIX_COL-1-k)){
                            LINE_Data_High();
                        }else{
                            LINE_Data_Low();
                        }
                        LINE_SCK_High();
                    }

                    LINE_Save();

                    count++;
                    printf("%d\n",count);
                }
                
            
            }
        }
        
    }
    



}






// void app_main(void)
// {   
//     initialUart();
//     initialMotor();
//     //xTaskCreate(testmotor, "motor", 1024*3, NULL, 10, NULL);
//     xTaskCreate(motor_work, "motor_task", 1024*3, NULL, 10, NULL);
//     // xTaskCreate(testpwm, "motor_task", 1024*3, NULL, 10, NULL);
//     //xTaskCreate(testOnebyOne, "motor_task", 1024*6, NULL, 10, NULL);
// }