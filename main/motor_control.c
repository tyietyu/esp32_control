#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"


#define MOTOR_L1           GPIO_NUM_15
#define MOTOR_R1           GPIO_NUM_16
#define MOTOR_L2           GPIO_NUM_17
#define MOTOR_R2           GPIO_NUM_18
#define MOTOR_L3           GPIO_NUM_19
#define MOTOR_R3           GPIO_NUM_21

#define TIMER_RESOLUTION_HZ     1000000     // 1MHz, 1us per tick
#define TIMER_PERIOD            1000        // 1000 ticks, 对应 1ms (1kHz 频率)
#define DUTY_CYCLE_VALUE        500         // 占空比设置值, 500 / 1000 = 50%
#define DEAD_TIME_TICKS         10

void motor_pwm_task(void *pvParameters)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .period_ticks = TIMER_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, 
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true, 
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &compare_config, &comparator));

    // 设置比较值以达到50%的占空比
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, DUTY_CYCLE_VALUE));

    mcpwm_gen_handle_t generatorA = NULL;
    mcpwm_gen_handle_t generatorB = NULL;
    mcpwm_generator_config_t gen_config_a = {
        .gen_gpio_num = GEN_GPIOA,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config_a, &generatorA));
    mcpwm_generator_config_t gen_config_b = {
        .gen_gpio_num = GEN_GPIOB,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config_b, &generatorB));

    // 当定时器从0开始计数时 (up, empty): GenA输出高, GenB输出低
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    // 当计数值达到比较值时 (up, compare): GenA输出低, GenB输出高
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_HIGH)));

    mcpwm_dead_time_config_t dead_time_config = {
        .posedge_delay_ticks = DEAD_TIME_TICKS, // 上升沿延迟
        .negedge_delay_ticks = DEAD_TIME_TICKS, // 下降沿延迟
        .flags.invert_output = false, // true 会交换A和B的死区设置
    };
    // 将死区配置应用到发生器A上，B会自动应用相反的设置
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generatorA, generatorA, &dead_time_config));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    
    uint32_t duty = 100;
    while (1) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
        duty += 100;
        if (duty > 900) {
            duty = 100;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

