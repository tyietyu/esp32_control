#ifndef _INPUT_DRIVER_H_
#define _INPUT_DRIVER_H_

esp_err_t limitStop_IO_init(void);
esp_err_t key_init(void);
uint8_t read_limitStop_IO_level(uint8_t limitStop_IO_num);
uint8_t read_key_level(uint8_t key_num);
void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

#endif // !_INPUT_DRIVER_H_

