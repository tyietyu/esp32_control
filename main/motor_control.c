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


const static char *TAG = "MOTOR_CONTROL";

#define MOTOR_L1 GPIO_NUM_15
#define MOTOR_R1 GPIO_NUM_16
#define MOTOR_L2 GPIO_NUM_17
#define MOTOR_R2 GPIO_NUM_18
#define MOTOR_L3 GPIO_NUM_19
#define MOTOR_R3 GPIO_NUM_21

#define TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define PWM_FREQUENCY_HZ 20000      // 20kHz 频率
#define TIMER_PERIOD (TIMER_RESOLUTION_HZ / PWM_FREQUENCY_HZ)

// 在这里修改占空比 (0 到 100)
#define MOTOR_DUTY_CYCLE_PERCENT 100
#define MOTOR_DUTY_CYCLE_VALUE (TIMER_PERIOD * MOTOR_DUTY_CYCLE_PERCENT / 100)
#define DEAD_TIME_TICKS 20 // 20个ticks * 1us/tick = 20us 死区时间

typedef struct
{
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t gen_a;
    mcpwm_gen_handle_t gen_b;
} motor_handle_t;

static mcpwm_timer_handle_t timer = NULL;
static motor_handle_t motors[3]; // 3个电机
static esp_timer_handle_t motor_timers[3];

void motor_stop(int motor_index);

/**
 * @brief 定时器到期后执行的回调函数
 * 这个函数会在esp_timer的专用任务中被调用
 * @param arg 传递给回调的参数，这里我们用它来区分是哪个电机的定时器
 */
static void motor_timer_callback(void* arg)
{
    int motor_index = (int)arg;
    ESP_LOGI(TAG, "timer end ,stop motor %d", motor_index);
    motor_stop(motor_index);
}


void motor_init(void)
{
    // 1. 创建共享的MCPWM定时器
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .period_ticks = TIMER_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    const int motor_pins[3][2] = {
        {MOTOR_L1, MOTOR_R1},
        {MOTOR_L2, MOTOR_R2},
        {MOTOR_L3, MOTOR_R3}};

    // 2. 循环创建每个电机的MCPWM句柄和esp_timer句柄
    for (int i = 0; i < 3; i++)
    {
        mcpwm_operator_config_t operator_config = {
            .group_id = 0,
            .flags.update_dead_time_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motors[i].oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motors[i].oper, timer));

        mcpwm_comparator_config_t compare_config = {
            .flags.update_cmp_on_tez = true
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].oper, &compare_config, &motors[i].comparator));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motors[i].comparator, MOTOR_DUTY_CYCLE_VALUE));

        mcpwm_generator_config_t gen_a_config = {
            .gen_gpio_num = motor_pins[i][0]
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gen_a_config, &motors[i].gen_a));

        mcpwm_generator_config_t gen_b_config = {
            .gen_gpio_num = motor_pins[i][1]
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gen_b_config, &motors[i].gen_b));

        // 设置互补PWM信号的动作（用于正转）
        // 当定时器计数到0时: 发生器A输出高电平, 发生器B输出低电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motors[i].gen_a,
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motors[i].gen_b,
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));

        // 当定时器计数到比较值时: 发生器A输出低电平, 发生器B输出高电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motors[i].gen_a,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motors[i].comparator, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motors[i].gen_b,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motors[i].comparator, MCPWM_GEN_ACTION_HIGH)));

        mcpwm_dead_time_config_t dead_time_config = {
            .posedge_delay_ticks = DEAD_TIME_TICKS,
            .negedge_delay_ticks = DEAD_TIME_TICKS,
            .flags.invert_output = false
        };
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(motors[i].gen_a, motors[i].gen_a, &dead_time_config));

        const esp_timer_create_args_t timer_args = {
        .callback = &motor_timer_callback,
        .arg = (void*) i, // 将电机索引作为参数传递
        .name = "motor_stop_timer"
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor_timers[i]));
    }

    // 3. 启动共享的MCPWM定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "motor initialized successfully.");
}

/**
 * @brief 控制电机正转并持续指定时间
 * @param motor_index 要控制的电机编号 (0, 1, 或 2)
 * @param duration_ms 旋转持续时间（毫秒）
 */
void motor_forward_for_duration(int motor_index, uint32_t duration_ms)
{
    if (motor_index < 0 || motor_index > 2) return;
    ESP_LOGI(TAG, "motor  %d forward, duration %u ms", (int)motor_index, (int)duration_ms);

    if (esp_timer_is_active(motor_timers[motor_index])) 
    {
        esp_timer_stop(motor_timers[motor_index]);
    }

    mcpwm_dead_time_config_t dead_time_config = { .flags.invert_output = false };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(motors[motor_index].gen_a, motors[motor_index].gen_a, &dead_time_config));
    // 启动一次性定时器，时间单位是微秒
    ESP_ERROR_CHECK(esp_timer_start_once(motor_timers[motor_index], duration_ms * 1000));
}

/**
 * @brief 控制电机反转并持续指定时间
 * @param motor_index 要控制的电机编号 (0, 1, 或 2)
 * @param duration_ms 旋转持续时间（毫秒）
 */
void motor_reverse_for_duration(int motor_index, uint32_t duration_ms)
{
    if (motor_index < 0 || motor_index > 2) return;
    ESP_LOGI(TAG, "motor %d reverse, duration %u ms", (int)motor_index, (int)duration_ms);

    if (esp_timer_is_active(motor_timers[motor_index]))
    {
        esp_timer_stop(motor_timers[motor_index]);
    }

    mcpwm_dead_time_config_t dead_time_config = { .flags.invert_output = true };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(motors[motor_index].gen_a, motors[motor_index].gen_a, &dead_time_config));
    // 启动一次性定时器，时间单位是微秒
    ESP_ERROR_CHECK(esp_timer_start_once(motor_timers[motor_index], duration_ms * 1000));
}

/**
 * @brief 停止指定电机
 * @param motor_index 要控制的电机编号 (0, 1, 或 2)
 */ 
void motor_stop(int motor_index)
{
    if (motor_index < 0 || motor_index > 2) return;
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motors[motor_index].gen_a, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motors[motor_index].gen_b, 0, true));
}

