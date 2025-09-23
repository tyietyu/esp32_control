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
#include "bdc_motor.h"

#include "input_driver.h"
#include "display_driver.h"
#include "motor_control.h"

static const char *TAG = "MAIN";

// 系统状态定义
typedef enum
{
    STATE_IDLE,       // 空闲状态
    STATE_MANUAL_AIM, // 手动摇杆控制状态
} system_state_t;

// --- 全局变量和FreeRTOS句柄 ---
volatile system_state_t g_current_state = STATE_IDLE;
SemaphoreHandle_t g_state_mutex;      // 状态互斥锁
SemaphoreHandle_t g_launch_trigger;   // 发射任务触发信号量
SemaphoreHandle_t g_random_trigger;   // 随机任务触发信号量

static adc_continuous_handle_t adc_handle = NULL;
static QueueHandle_t adc_data_queue;

//================================================================================
// 任务 1: 数码管显示任务
//================================================================================
void display_task(void *pvParameters)
{
    uint32_t adc_value = 0;
    while (1)
    {
        if (xQueueReceive(adc_data_queue, &adc_value, portMAX_DELAY))
        {
            // 将ADC值 (0-4095) 转换为速度 (0-30 m/s)
            float speed = (float)adc_value * 30.0f / 4095.0f;
            ESP_LOGD(TAG, "ADC Value: %d, Speed: %d m/s", (int)adc_value, (int)speed);
            display_set_float(speed);
        }
    }
}

//================================================================================
// 任务 2: 发射流程任务
//================================================================================
void launch_task(void *pvParameters)
{
    while (1)
    {
        // 等待发射触发信号, 获取信号后开始执行
        xSemaphoreTake(g_launch_trigger, portMAX_DELAY);

        ESP_LOGI(TAG, "发射任务开始...");

        // 1. 电机1正转，直到触发限位器2
        ESP_LOGI(TAG, "电机1正转...");
        motor_start_forward(0);
        // 等待限位器2被触发(变为0)。循环条件是它还未被触发(为1)
        while (read_limitStop_IO_level(2) == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(20)); // 短暂延时，让出CPU
        }
        motor_stop(0);
        ESP_LOGI(TAG, "触发限位器2");

        vTaskDelay(pdMS_TO_TICKS(100)); // 触发后短暂延时，防止机械抖动

        // 2. 电机1反转，直到触发限位器1
        ESP_LOGI(TAG, "电机1反转...");
        motor_start_reverse(0);
        // 等待限位器1被触发(变为0)。循环条件是它还未被触发(为1)
        while (read_limitStop_IO_level(1) == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(20)); // 短暂延时，让出CPU
        }
        motor_stop(0);
        ESP_LOGI(TAG, "触发限位器1, 发射流程结束。");
    }
}

//================================================================================
// 任务 3: 随机模式任务
//================================================================================
void random_mode_task(void *pvParameters)
{
    while (1)
    {
        // 等待随机模式触发信号
        xSemaphoreTake(g_random_trigger, portMAX_DELAY);
        ESP_LOGI(TAG, "随机模式开始...");

        int64_t start_time = esp_timer_get_time();
        // 持续5秒
        while ((esp_timer_get_time() - start_time) < 5000000)
        {
            // --- 电机2随机动作 ---
            int motor2_action = rand() % 3; // 0: 停止, 1: 正转, 2: 反转
            if ((motor2_action == 1) && (read_limitStop_IO_level(3) == 1))
            {
                motor_start_forward(1);
            }
            else if ((motor2_action == 2) && (read_limitStop_IO_level(4) == 1))
            {
                motor_start_reverse(1);
            }
            else
            {
                motor_stop(1);
            }

            // --- 电机3随机动作 ---
            int motor3_action = rand() % 3;
            if ((motor3_action == 1) && (read_limitStop_IO_level(5) == 1))
            {
                motor_start_forward(2);
            }
            else if ((motor3_action == 2) && (read_limitStop_IO_level(6) == 1))
            {
                motor_start_reverse(2);
            }
            else
            {
                motor_stop(2);
            }
            
            // 延时，让电机转一会儿再改变动作
            vTaskDelay(pdMS_TO_TICKS(500 + (rand() % 500))); // 随机延时0.5-1秒
        }

        // 5秒后确保所有电机停止
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
    uint32_t adc_joy_x = 1550; // 初始值设在死区内
    uint32_t adc_joy_y = 1350; // 初始值设在死区内
    uint32_t pot_val = 0;

    // 摇杆死区定义
    const int JOYSTICK_DEADZONE_LOW_X = 1500;
    const int JOYSTICK_DEADZONE_HIGH_X = 1600;
    const int JOYSTICK_DEADZONE_LOW_Y = 1300;
    const int JOYSTICK_DEADZONE_HIGH_Y = 1400;

    while (1)
    {
        // --- ADC数据处理 ---
        adc_continuous_read(adc_handle, result, ADC_READ_LEN, &ret_num, 0);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (void *)(&result[i]);
            uint32_t chan = ADC_GET_CHANNEL(p);
            uint32_t data = ADC_GET_DATA(p);

            if (chan == ADC1_CHAN1) { pot_val = data; }
            else if (chan == ADC1_CHANx) { adc_joy_x = data; }
            else if (chan == ADC1_CHANy) { adc_joy_y = data; }
        }
        
        if (ret_num > 0) 
        { 
            xQueueSend(adc_data_queue, &pot_val, 0);
        }

        // 获取状态锁，准备访问全局状态变量
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);

        switch (g_current_state)
        {
        case STATE_IDLE:
            if ((read_key_level(2) == 0) && (read_limitStop_IO_level(1) == 0)) // 按键1按下且限位器1触发
            {
                ESP_LOGI(TAG, "按键1按下, 触发发射任务...");
                xSemaphoreGive(g_launch_trigger); // 触发发射任务
            }
            else if (read_key_level(3) == 0) // 按键2按下
            {
                ESP_LOGI(TAG, "按键2按下, 触发随机任务...");
                xSemaphoreGive(g_random_trigger); // 触发随机任务
            }
            else if ((adc_joy_x < JOYSTICK_DEADZONE_LOW_X) || (adc_joy_x > JOYSTICK_DEADZONE_HIGH_X) ||
                     (adc_joy_y < JOYSTICK_DEADZONE_LOW_Y) || (adc_joy_y > JOYSTICK_DEADZONE_HIGH_Y))
            {
                g_current_state = STATE_MANUAL_AIM; // 摇杆被触动
                ESP_LOGI(TAG, "state change: IDLE -> MANUAL_AIM");
            }
            break;

        case STATE_MANUAL_AIM:
            // --- 摇杆X轴独立控制电机2 ---
            if ((adc_joy_x < JOYSTICK_DEADZONE_LOW_X) && (read_limitStop_IO_level(3) == 1))
            {
                motor_start_forward(1); // X轴向一侧
            }
            else if ((adc_joy_x > JOYSTICK_DEADZONE_HIGH_X) && (read_limitStop_IO_level(4) == 1))
            {
                motor_start_reverse(1); // X轴向另一侧
            }
            else
            {
                motor_stop(1); // X轴在死区内
            }

            // --- 摇杆Y轴独立控制电机3 ---
            if ((adc_joy_y < JOYSTICK_DEADZONE_LOW_Y) && (read_limitStop_IO_level(5) == 1))
            {
                motor_start_forward(2); // Y轴向一侧
            }
            else if ((adc_joy_y > JOYSTICK_DEADZONE_HIGH_Y) && (read_limitStop_IO_level(6) == 1))
            {
                motor_start_reverse(2); // Y轴向另一侧
            }
            else
            {
                motor_stop(2); // Y轴在死区内
            }

            // --- 如果摇杆完全回中，则返回IDLE状态 ---
            if ((adc_joy_x >= JOYSTICK_DEADZONE_LOW_X) && (adc_joy_x <= JOYSTICK_DEADZONE_HIGH_X) &&
                (adc_joy_y >= JOYSTICK_DEADZONE_LOW_Y) && (adc_joy_y <= JOYSTICK_DEADZONE_HIGH_Y))
            {
                g_current_state = STATE_IDLE;
                ESP_LOGI(TAG, "state change: MANUAL_AIM -> IDLE");
            }
            break;
        }

        // 释放状态锁
        xSemaphoreGive(g_state_mutex);

        ESP_LOGD(TAG, "JoyX: %d, JoyY: %d", (int)adc_joy_x, (int)adc_joy_y);
        vTaskDelay(pdMS_TO_TICKS(50)); // 每50ms扫描一次
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "init hardware drivers...");
    limitStop_IO_init();
    key_init();
    display_init();
    motor_init(); // 电机ID范围为0，1，2 ---> 对应电机1，2，3

    ESP_LOGI(TAG, "init ADC...");
    continuous_adc_init(adc_channel, 4, &adc_handle);
    adc_continuous_start(adc_handle);

    // --- 初始化FreeRTOS组件 ---
    ESP_LOGI(TAG, "init FreeRTOS components...");
    g_state_mutex = xSemaphoreCreateMutex();
    g_launch_trigger = xSemaphoreCreateBinary();
    g_random_trigger = xSemaphoreCreateBinary();
    adc_data_queue = xQueueCreate(10, sizeof(uint32_t));

    // --- 创建所有任务 ---
    ESP_LOGI(TAG, "create tasks...");
    xTaskCreate(display_task, "display_task", 2048, NULL, 4, NULL);
    xTaskCreate(launch_task, "launch_task", 2048, NULL, 5, NULL);
    xTaskCreate(random_mode_task, "random_mode_task", 2048, NULL, 3, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "init completed. System is now running.");
}

