#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bdc_motor.h"

const static char *TAG = "MOTOR_CONTROL";

#define MOTOR_L1 GPIO_NUM_15
#define MOTOR_R1 GPIO_NUM_16
#define MOTOR_L2 GPIO_NUM_17
#define MOTOR_R2 GPIO_NUM_18
#define MOTOR_L3 GPIO_NUM_19
#define MOTOR_R3 GPIO_NUM_21

#define TIMER_RESOLUTION_HZ     10000000 // 1MHz, 1us per tick
#define PWM_FREQUENCY_HZ        25000      // 25kHz 频率
#define MOTOR_DUTY_TICK_MAX     (TIMER_RESOLUTION_HZ / PWM_FREQUENCY_HZ ) // 400 ticks (1 tick = 0.1us)

#define MOTOR_DUTY_CYCLE_PERCENT  90   // 90% 占空比
#define MOTOR_SPEED_TICKS       ((MOTOR_DUTY_TICK_MAX * MOTOR_DUTY_CYCLE_PERCENT) / 100)

const uint32_t motor_gpio_a[3] = {MOTOR_L1, MOTOR_L2, MOTOR_L3};
const uint32_t motor_gpio_b[3] = {MOTOR_R1, MOTOR_R2, MOTOR_R3};

bdc_motor_handle_t motors[3] = {NULL};
static esp_timer_handle_t motor_stop_timers[3] = {NULL};

static void motor_stop_cb(void *arg)
{
    uint8_t motor_index = (uint8_t)(intptr_t)arg;
    ESP_LOGI(TAG, "Timer expired, stopping motor %d", motor_index);
    motor_stop(motor_index);
}

void motor_init()
{
    ESP_LOGI(TAG, "Initializing motors...");

    for (int i = 0; i < 3; i++)
    {
        bdc_motor_config_t motor_config = {
            .pwm_freq_hz = PWM_FREQUENCY_HZ,
            .pwma_gpio_num = motor_gpio_a[i],
            .pwmb_gpio_num = motor_gpio_b[i],
        };

        bdc_motor_mcpwm_config_t mcpwm_config = {
            .group_id = 0, 
            .resolution_hz = TIMER_RESOLUTION_HZ,
        };

        ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motors[i]));
    }

    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(bdc_motor_enable(motors[i]));
    }

    for (int i = 0; i < 3; i++) 
    {
        esp_timer_create_args_t timer_args = {
            .callback = &motor_stop_cb,
            .arg = (void *)(intptr_t)i, 
            .name = "motor_stop_timers"
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor_stop_timers[i]));
    }

    ESP_LOGI(TAG, "Motor initialization completed.");
}

void motor_forward_for_duration(uint8_t motor_index, uint32_t duration_ms)
{
    if (motor_index >= 3)
    {
        ESP_LOGE(TAG, "Invalid motor index: %d", motor_index);
        return;
    }

    ESP_LOGI(TAG, "Motor %d FORWARD for %" PRIu32 " ms", motor_index, duration_ms);

    if (esp_timer_is_active(motor_stop_timers[motor_index]))
    {
        esp_timer_stop(motor_stop_timers[motor_index]);
    }
    // 启动电机
    ESP_ERROR_CHECK(bdc_motor_forward(motors[motor_index]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(motors[motor_index], MOTOR_SPEED_TICKS));

    // 启动一次性定时器，时间单位是微秒 (us)
    ESP_ERROR_CHECK(esp_timer_start_once(motor_stop_timers[motor_index], duration_ms * 1000));
}

void motor_reverse_for_duration(uint8_t motor_index, uint32_t duration_ms)
{
    if (motor_index >= 3) {
        ESP_LOGE(TAG, "Invalid motor index: %d", motor_index);
        return;
    }
    
    ESP_LOGI(TAG, "Motor %d REVERSE for %" PRIu32 " ms", motor_index, duration_ms);
    if (esp_timer_is_active(motor_stop_timers[motor_index]))
    {
        esp_timer_stop(motor_stop_timers[motor_index]);
    }
    // 启动电机
    ESP_ERROR_CHECK(bdc_motor_reverse(motors[motor_index]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(motors[motor_index], MOTOR_SPEED_TICKS));

    // 启动一次性定时器，时间单位是微秒 (us)
    ESP_ERROR_CHECK(esp_timer_start_once(motor_stop_timers[motor_index], duration_ms * 1000));
}

void motor_stop(uint8_t motor_index)
{
    if (motor_index >= 3) {
        ESP_LOGE(TAG, "Invalid motor index: %d", motor_index);
        return;
    }

    // 如果定时器还在运行，先停止它，防止冲突
    if (esp_timer_is_active(motor_stop_timers[motor_index])) {
        esp_timer_stop(motor_stop_timers[motor_index]);
    }

    // 停止电机
    ESP_ERROR_CHECK(bdc_motor_brake(motors[motor_index]));
}

#if 0
void test(void)
{
    //正转2s
    // motor_start_forward(2);
    motor_forward_for_duration(2, 10000);
    vTaskDelay(pdMS_TO_TICKS(10000));
    //停止
    motor_stop(2);
    vTaskDelay(pdMS_TO_TICKS(5000));
    //反转2s
    motor_reverse_for_duration(2, 10000);
    vTaskDelay(pdMS_TO_TICKS(10000));
    //停止
    motor_stop(2);
}

void motor_start_forward(uint8_t motor_index)
{
    if (motor_index >= 3) return;
    ESP_LOGI(TAG, "Motor %d START FORWARD", motor_index);
    ESP_ERROR_CHECK(bdc_motor_forward(motors[motor_index]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(motors[motor_index], MOTOR_SPEED_TICKS));
}

// 持续反转，直到手动调用 motor_stop
void motor_start_reverse(uint8_t motor_index)
{
    if (motor_index >= 3) return;
    ESP_LOGI(TAG, "Motor %d START REVERSE", motor_index);
    ESP_ERROR_CHECK(bdc_motor_reverse(motors[motor_index]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(motors[motor_index], MOTOR_SPEED_TICKS));
}
#endif
