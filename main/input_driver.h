#ifndef _INPUT_DRIVER_H_
#define _INPUT_DRIVER_H_

#include "esp_adc/adc_continuous.h"

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

adc_channel_t channel[4] = {ADC1_CHAN1, ADC1_CHAN2, ADC1_CHANx, ADC1_CHANy};



esp_err_t limitStop_IO_init(void);
esp_err_t key_init(void);
uint8_t read_limitStop_IO_level(uint8_t limitStop_IO_num);
uint8_t read_key_level(uint8_t key_num);
void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

#endif // !_INPUT_DRIVER_H_

