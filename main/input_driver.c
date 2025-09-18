#include "input_driver.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"

// limitStop Switch GPIO Definitions
#define limitStop_IO1           GPIO_NUM_32
#define limitStop_IO2           GPIO_NUM_33
#define limitStop_IO3           GPIO_NUM_25
#define limitStop_IO4           GPIO_NUM_26
#define limitStop_IO5           GPIO_NUM_27
#define limitStop_IO6           GPIO_NUM_14

//key GPIO Definitions
#define KEY1        GPIO_NUM_12
#define KEY2        GPIO_NUM_13
#define KEYX        GPIO_NUM_4
#define KEYY        GPIO_NUM_5

//ADC Definitions
#define ADC1_CHAN1      ADC_CHANNEL_0 // GPIO36
#define ADC1_CHAN2      ADC_CHANNEL_3 // GPIO39
#define ADC1_CHANx      ADC_CHANNEL_6 // GPIO34
#define ADC1_CHANy      ADC_CHANNEL_7 // GPIO35

#define ADC_UNIT                            ADC_UNIT_1
#define ADC_UNIT_STR(unit)                  #unit
#define ADC_GET_CHANNEL(p_data)             ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)                ((p_data)->type1.data)
#define ADC_READ_LEN                        256

static adc_channel_t channel[4] = {ADC1_CHAN1, ADC1_CHAN2, ADC1_CHANx, ADC1_CHANy};
static TaskHandle_t s_task_handle;

esp_err_t limitStop_IO_init(void)
{
    gpio_config_t limitStop_io_conf = {
        .pin_bit_mask = (1ULL << limitStop_IO1) | (1ULL << limitStop_IO2) | (1ULL << limitStop_IO3) |
                        (1ULL << limitStop_IO4) | (1ULL << limitStop_IO5) | (1ULL << limitStop_IO6),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limitStop_io_conf));
}

esp_err_t key_init(void)
{
    gpio_config_t key_io_conf = {
        .pin_bit_mask = (1ULL << KEY1) | (1ULL << KEY2) | (1ULL << KEYX) | (1ULL << KEYY),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&key_io_conf));
}

uint8_t read_limitStop_IO_level(uint8_t limitStop_IO_num)
{
    switch (limitStop_IO_num)
    {
    case 1:
        return gpio_get_level(limitStop_IO1);
        break;
    case 2:
        return gpio_get_level(limitStop_IO2);
        break;
    case 3:
        return gpio_get_level(limitStop_IO3);
        break;
    case 4:
        return gpio_get_level(limitStop_IO4);
        break;
    case 5:
        return gpio_get_level(limitStop_IO5);
        break;
    case 6:
        return gpio_get_level(limitStop_IO6);   
        break;
    default:
    break;
    }
}

uint8_t read_key_level(uint8_t key_num)
{
    switch (key_num)
    {
    case 1:
        return gpio_get_level(KEY1);
        break;
    case 2:
        return gpio_get_level(KEY2);
        break;
    case 3:
        return gpio_get_level(KEYX);
        break;
    case 4:
        return gpio_get_level(KEYY);
        break;
    default:
    break;
    }
}

bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}