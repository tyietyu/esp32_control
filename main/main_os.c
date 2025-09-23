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

// ϵͳ״̬����
typedef enum
{
    STATE_IDLE,       // ����״̬
    STATE_MANUAL_AIM, // �ֶ�ҡ�˿���״̬
} system_state_t;

// --- ȫ�ֱ�����FreeRTOS��� ---
volatile system_state_t g_current_state = STATE_IDLE;
SemaphoreHandle_t g_state_mutex;      // ״̬������
SemaphoreHandle_t g_launch_trigger;   // �������񴥷��ź���
SemaphoreHandle_t g_random_trigger;   // ������񴥷��ź���

static adc_continuous_handle_t adc_handle = NULL;
static QueueHandle_t adc_data_queue;

//================================================================================
// ���� 1: �������ʾ����
//================================================================================
void display_task(void *pvParameters)
{
    uint32_t adc_value = 0;
    while (1)
    {
        if (xQueueReceive(adc_data_queue, &adc_value, portMAX_DELAY))
        {
            // ��ADCֵ (0-4095) ת��Ϊ�ٶ� (0-30 m/s)
            float speed = (float)adc_value * 30.0f / 4095.0f;
            ESP_LOGD(TAG, "ADC Value: %d, Speed: %d m/s", (int)adc_value, (int)speed);
            display_set_float(speed);
        }
    }
}

//================================================================================
// ���� 2: ������������
//================================================================================
void launch_task(void *pvParameters)
{
    while (1)
    {
        // �ȴ����䴥���ź�, ��ȡ�źź�ʼִ��
        xSemaphoreTake(g_launch_trigger, portMAX_DELAY);

        ESP_LOGI(TAG, "��������ʼ...");

        // 1. ���1��ת��ֱ��������λ��2
        ESP_LOGI(TAG, "���1��ת...");
        motor_start_forward(0);
        // �ȴ���λ��2������(��Ϊ0)��ѭ������������δ������(Ϊ1)
        while (read_limitStop_IO_level(2) == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(20)); // ������ʱ���ó�CPU
        }
        motor_stop(0);
        ESP_LOGI(TAG, "������λ��2");

        vTaskDelay(pdMS_TO_TICKS(100)); // �����������ʱ����ֹ��е����

        // 2. ���1��ת��ֱ��������λ��1
        ESP_LOGI(TAG, "���1��ת...");
        motor_start_reverse(0);
        // �ȴ���λ��1������(��Ϊ0)��ѭ������������δ������(Ϊ1)
        while (read_limitStop_IO_level(1) == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(20)); // ������ʱ���ó�CPU
        }
        motor_stop(0);
        ESP_LOGI(TAG, "������λ��1, �������̽�����");
    }
}

//================================================================================
// ���� 3: ���ģʽ����
//================================================================================
void random_mode_task(void *pvParameters)
{
    while (1)
    {
        // �ȴ����ģʽ�����ź�
        xSemaphoreTake(g_random_trigger, portMAX_DELAY);
        ESP_LOGI(TAG, "���ģʽ��ʼ...");

        int64_t start_time = esp_timer_get_time();
        // ����5��
        while ((esp_timer_get_time() - start_time) < 5000000)
        {
            // --- ���2������� ---
            int motor2_action = rand() % 3; // 0: ֹͣ, 1: ��ת, 2: ��ת
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

            // --- ���3������� ---
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
            
            // ��ʱ���õ��תһ����ٸı䶯��
            vTaskDelay(pdMS_TO_TICKS(500 + (rand() % 500))); // �����ʱ0.5-1��
        }

        // 5���ȷ�����е��ֹͣ
        motor_stop(1);
        motor_stop(2);
        ESP_LOGI(TAG, "���ģʽ������׼����������...");

        // ������������
        xSemaphoreGive(g_launch_trigger);
    }
}

//================================================================================
// ���� 4: ���Ŀ���������ɨ������
//================================================================================
void control_task(void *pvParameters)
{
    // ��������ADC��ȡ�Ļ�����
    uint8_t result[ADC_READ_LEN] = {0};
    uint32_t ret_num = 0;
    uint32_t adc_joy_x = 1550; // ��ʼֵ����������
    uint32_t adc_joy_y = 1350; // ��ʼֵ����������
    uint32_t pot_val = 0;

    // ҡ����������
    const int JOYSTICK_DEADZONE_LOW_X = 1500;
    const int JOYSTICK_DEADZONE_HIGH_X = 1600;
    const int JOYSTICK_DEADZONE_LOW_Y = 1300;
    const int JOYSTICK_DEADZONE_HIGH_Y = 1400;

    while (1)
    {
        // --- ADC���ݴ��� ---
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

        // ��ȡ״̬����׼������ȫ��״̬����
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);

        switch (g_current_state)
        {
        case STATE_IDLE:
            if ((read_key_level(2) == 0) && (read_limitStop_IO_level(1) == 0)) // ����1��������λ��1����
            {
                ESP_LOGI(TAG, "����1����, ������������...");
                xSemaphoreGive(g_launch_trigger); // ������������
            }
            else if (read_key_level(3) == 0) // ����2����
            {
                ESP_LOGI(TAG, "����2����, �����������...");
                xSemaphoreGive(g_random_trigger); // �����������
            }
            else if ((adc_joy_x < JOYSTICK_DEADZONE_LOW_X) || (adc_joy_x > JOYSTICK_DEADZONE_HIGH_X) ||
                     (adc_joy_y < JOYSTICK_DEADZONE_LOW_Y) || (adc_joy_y > JOYSTICK_DEADZONE_HIGH_Y))
            {
                g_current_state = STATE_MANUAL_AIM; // ҡ�˱�����
                ESP_LOGI(TAG, "state change: IDLE -> MANUAL_AIM");
            }
            break;

        case STATE_MANUAL_AIM:
            // --- ҡ��X��������Ƶ��2 ---
            if ((adc_joy_x < JOYSTICK_DEADZONE_LOW_X) && (read_limitStop_IO_level(3) == 1))
            {
                motor_start_forward(1); // X����һ��
            }
            else if ((adc_joy_x > JOYSTICK_DEADZONE_HIGH_X) && (read_limitStop_IO_level(4) == 1))
            {
                motor_start_reverse(1); // X������һ��
            }
            else
            {
                motor_stop(1); // X����������
            }

            // --- ҡ��Y��������Ƶ��3 ---
            if ((adc_joy_y < JOYSTICK_DEADZONE_LOW_Y) && (read_limitStop_IO_level(5) == 1))
            {
                motor_start_forward(2); // Y����һ��
            }
            else if ((adc_joy_y > JOYSTICK_DEADZONE_HIGH_Y) && (read_limitStop_IO_level(6) == 1))
            {
                motor_start_reverse(2); // Y������һ��
            }
            else
            {
                motor_stop(2); // Y����������
            }

            // --- ���ҡ����ȫ���У��򷵻�IDLE״̬ ---
            if ((adc_joy_x >= JOYSTICK_DEADZONE_LOW_X) && (adc_joy_x <= JOYSTICK_DEADZONE_HIGH_X) &&
                (adc_joy_y >= JOYSTICK_DEADZONE_LOW_Y) && (adc_joy_y <= JOYSTICK_DEADZONE_HIGH_Y))
            {
                g_current_state = STATE_IDLE;
                ESP_LOGI(TAG, "state change: MANUAL_AIM -> IDLE");
            }
            break;
        }

        // �ͷ�״̬��
        xSemaphoreGive(g_state_mutex);

        ESP_LOGD(TAG, "JoyX: %d, JoyY: %d", (int)adc_joy_x, (int)adc_joy_y);
        vTaskDelay(pdMS_TO_TICKS(50)); // ÿ50msɨ��һ��
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "init hardware drivers...");
    limitStop_IO_init();
    key_init();
    display_init();
    motor_init(); // ���ID��ΧΪ0��1��2 ---> ��Ӧ���1��2��3

    ESP_LOGI(TAG, "init ADC...");
    continuous_adc_init(adc_channel, 4, &adc_handle);
    adc_continuous_start(adc_handle);

    // --- ��ʼ��FreeRTOS��� ---
    ESP_LOGI(TAG, "init FreeRTOS components...");
    g_state_mutex = xSemaphoreCreateMutex();
    g_launch_trigger = xSemaphoreCreateBinary();
    g_random_trigger = xSemaphoreCreateBinary();
    adc_data_queue = xQueueCreate(10, sizeof(uint32_t));

    // --- ������������ ---
    ESP_LOGI(TAG, "create tasks...");
    xTaskCreate(display_task, "display_task", 2048, NULL, 4, NULL);
    xTaskCreate(launch_task, "launch_task", 2048, NULL, 5, NULL);
    xTaskCreate(random_mode_task, "random_mode_task", 2048, NULL, 3, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "init completed. System is now running.");
}

