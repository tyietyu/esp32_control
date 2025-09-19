#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

#include "input_driver.h"
#include "display_driver.h"
#include "motor_control.h"

static const char *TAG = "MAIN";

typedef enum {
    STATE_IDLE,         // 空闲状态
    STATE_MANUAL_AIM,   // 手动摇杆控制状态
    STATE_LAUNCHING,    // 发射流程状态
    STATE_RANDOM_MODE   // 随机模式状态
} system_state_t;

volatile system_state_t g_current_state = STATE_IDLE; 
SemaphoreHandle_t g_state_mutex;
SemaphoreHandle_t g_launch_trigger;
SemaphoreHandle_t g_random_trigger;

static adc_continuous_handle_t adc_handle = NULL;
static QueueHandle_t adc_data_queue;


//================================================================================
// 任务 1: 数码管显示任务
//================================================================================
void display_task(void *pvParameters)
{
    uint32_t adc_value = 0;
    while (1) {
        if (xQueueReceive(adc_data_queue, &adc_value, portMAX_DELAY)) {
            // 将ADC值 (0-4095) 转换为速度 (0-30 m/s)
            float speed = (float)adc_value * 300 / 4095;
            display_set_float(speed); 
        }
    }
}

//================================================================================
// 任务 2: 发射流程任务
//================================================================================
void launch_task(void *pvParameters)
{
    while (1) {
        // 等待发射触发信号
        xSemaphoreTake(g_launch_trigger, portMAX_DELAY);

        ESP_LOGI(TAG, "发射任务开始...");
        
        // 1. 电机1正转，直到触发限位器2
        ESP_LOGI(TAG, "电机1正转...");
        motor_forward_for_duration(0, 60000); // 启动电机1，设置一个超长时间
        while (read_limitStop_IO_level(2) == 1) {
            vTaskDelay(pdMS_TO_TICKS(20)); // 等待限位器2触发 (低电平)
        }
        motor_stop(0); // 立即停止电机1
        ESP_LOGI(TAG, "触发限位器2");

        // 2. 电机1反转，直到触发限位器1
        ESP_LOGI(TAG, "电机1反转...");
        motor_reverse_for_duration(0, 60000); // 启动电机1反转
        while (read_limitStop_IO_level(1) == 1) {
            vTaskDelay(pdMS_TO_TICKS(20)); // 等待限位器1触发 (低电平)
        }
        motor_stop(0); // 立即停止电机1
        ESP_LOGI(TAG, "触发限位器1,发射流程结束。");
        
        // 流程结束，将状态切回IDLE
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_current_state = STATE_IDLE;
        xSemaphoreGive(g_state_mutex);
    }
}

//================================================================================
// 任务 3: 随机模式任务
//================================================================================
void random_mode_task(void *pvParameters)
{
    while(1) {
        // 等待随机模式触发信号
        xSemaphoreTake(g_random_trigger, portMAX_DELAY);
        ESP_LOGI(TAG, "随机模式任务开始...");

        int64_t start_time = esp_timer_get_time();
        // 持续5秒
        while ((esp_timer_get_time() - start_time) < 5000000) {
            // 随机决定电机2的动作
            int motor2_action = rand() % 3; // 0: 停止, 1: 正转, 2: 反转
            if (motor2_action == 1 && read_limitStop_IO_level(3) == 1) {
                motor_forward_for_duration(1, 200); // 正转200ms
            } else if (motor2_action == 2 && read_limitStop_IO_level(4) == 1) {
                motor_reverse_for_duration(1, 200); // 反转200ms
            }

            // 随机决定电机3的动作
            int motor3_action = rand() % 3; // 0: 停止, 1: 正转, 2: 反转
            if (motor3_action == 1 && read_limitStop_IO_level(5) == 1) {
                motor_forward_for_duration(2, 200); // 正转200ms
            } else if (motor3_action == 2 && read_limitStop_IO_level(6) == 1) {
                motor_reverse_for_duration(2, 200); // 反转200ms
            }
            vTaskDelay(pdMS_TO_TICKS(250)); // 每250ms改变一次动作
        }

        // 5秒后停止电机2和3
        motor_stop(1);
        motor_stop(2);
        ESP_LOGI(TAG, "随机模式结束，准备触发发射...");
        
        // 触发发射任务
        xSemaphoreGive(g_launch_trigger);
    }
}


//================================================================================
// 任务 4: 核心控制与输入扫描任务
//================================================================================
void control_task(void *pvParameters)
{
    // 用于连续ADC读取的缓冲区
    uint8_t result[ADC_READ_LEN] = {0};
    uint32_t ret_num = 0;
    uint32_t adc_joy_x = 2028;
    uint32_t adc_joy_y = 2048;

    // 摇杆死区定义
    const int JOYSTICK_DEADZONE_LOW = 1800;
    const int JOYSTICK_DEADZONE_HIGH = 2200;

    while (1) {
        // --- ADC数据处理 ---
        adc_continuous_read(adc_handle, result, ADC_READ_LEN, &ret_num, 0);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (void*)(&result[i]);
            uint32_t chan = ADC_GET_CHANNEL(p);
            uint32_t data = ADC_GET_DATA(p);
            
            if (chan == ADC1_CHAN1) {
                xQueueSend(adc_data_queue, &data, 0);
            } else if (chan == ADC1_CHANx) {
                adc_joy_x = data;
            } else if (chan == ADC1_CHANy) {
                adc_joy_y = data;
            }
        }
        
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        switch (g_current_state) 
        {
            case STATE_IDLE:
                if (read_key_level(1) == 0 && read_limitStop_IO_level(1) == 0)  // 按键1按下且限位1触发
                { 
                    g_current_state = STATE_LAUNCHING;
                    xSemaphoreGive(g_launch_trigger); // 触发发射任务
                    ESP_LOGI(TAG, "状态切换: IDLE -> LAUNCHING");
                } else if (read_key_level(2) == 0)  // 按键2按下
                { 
                    g_current_state = STATE_RANDOM_MODE;
                    xSemaphoreGive(g_random_trigger); // 触发随机任务
                    ESP_LOGI(TAG, "状态切换: IDLE -> RANDOM_MODE");
                } else if (adc_joy_x < JOYSTICK_DEADZONE_LOW || adc_joy_x > JOYSTICK_DEADZONE_HIGH ||
                           adc_joy_y < JOYSTICK_DEADZONE_LOW || adc_joy_y > JOYSTICK_DEADZONE_HIGH)
             {
                    g_current_state = STATE_MANUAL_AIM; // 摇杆被触动
                    ESP_LOGI(TAG, "状态切换: IDLE -> MANUAL_AIM");
                }
                break;

            case STATE_MANUAL_AIM:
                // 摇杆X轴控制电机2
                if (adc_joy_x < JOYSTICK_DEADZONE_LOW && read_limitStop_IO_level(3) == 1) {
                    motor_forward_for_duration(1, 100); // 持续发送指令保持转动
                } else if (adc_joy_x > JOYSTICK_DEADZONE_HIGH && read_limitStop_IO_level(4) == 1) {
                    motor_reverse_for_duration(1, 100);
                } else {
                    motor_stop(1);
                }

                // 摇杆Y轴控制电机3
                if (adc_joy_y < JOYSTICK_DEADZONE_LOW && read_limitStop_IO_level(5) == 1) {
                    motor_forward_for_duration(2, 100); // 向上
                } else if (adc_joy_y > JOYSTICK_DEADZONE_HIGH && read_limitStop_IO_level(6) == 1) {
                    motor_reverse_for_duration(2, 100); // 向下
                } else {
                    motor_stop(2);
                }

                // 如果摇杆回中，则返回IDLE状态
                if (adc_joy_x >= JOYSTICK_DEADZONE_LOW && adc_joy_x <= JOYSTICK_DEADZONE_HIGH &&
                    adc_joy_y >= JOYSTICK_DEADZONE_LOW && adc_joy_y <= JOYSTICK_DEADZONE_HIGH) {
                    g_current_state = STATE_IDLE;
                    motor_stop(1);
                    motor_stop(2);
                    ESP_LOGI(TAG, "状态切换: MANUAL_AIM -> IDLE");
                }
                break;

            case STATE_LAUNCHING:
                break;
            case STATE_RANDOM_MODE:
                break;
        }

        xSemaphoreGive(g_state_mutex);
        vTaskDelay(pdMS_TO_TICKS(50)); // 每50ms扫描一次
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "初始化硬件驱动...");
    limitStop_IO_init();
    key_init();
    display_init();
    motor_init();
    
    ESP_LOGI(TAG, "初始化ADC...");
    continuous_adc_init(channel, 4, &adc_handle);
    adc_continuous_start(adc_handle);

    // --- 3. 初始化FreeRTOS组件 ---
    ESP_LOGI(TAG, "初始化FreeRTOS组件...");
    g_state_mutex = xSemaphoreCreateMutex();
    g_launch_trigger = xSemaphoreCreateBinary();
    g_random_trigger = xSemaphoreCreateBinary();
    adc_data_queue = xQueueCreate(10, sizeof(uint32_t));

    // --- 4. 创建所有任务 ---
    ESP_LOGI(TAG, "创建任务...");
    xTaskCreate(display_task, "display_task", 2048, NULL, 5, NULL);
    xTaskCreate(launch_task, "launch_task", 2048, NULL, 10, NULL);
    xTaskCreate(random_mode_task, "random_mode_task", 2048, NULL, 10, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "系统初始化完成，进入主循环。");

}

