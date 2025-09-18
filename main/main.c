/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "driver/mcpwm_prelude.h"

#include "input_driver.h"
#include "display_driver.h"
#include "motor_control.h"



void app_main(void)
{

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000) / portTICK_PERIOD_MS);
    }
}
