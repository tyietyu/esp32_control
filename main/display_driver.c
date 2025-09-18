#include "display_driver.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

const static char *TAG = "DISPLAY_DRIVER";

#define TM1637_SCL          GPIO_NUM_23
#define TM1637_SDA          GPIO_NUM_22             
#define TM1637_DELAY_US     5      

//TM1637 register definitions
//Data Command Settings
#define TM1637_CMD_SET_DATA         0x40
#define TM1637_MODE_WRITE_TO_REG    0x00
#define TM1637_ADDR_MODE_AUTO_INC   0x00

//Address Command Settings
#define TM1637_CMD_SET_ADDR         0xC0

//Display Control Command Settings
#define TM1637_CMD_SET_DISPLAY      0x80
#define TM1637_DISPLAY_ON           0x08
#define TM1637_BRIGHTNESS_10_16     0x03 //亮度设置 (0x00 - 0x07)

static const uint8_t segment_map[] = {
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8
    0x6f  // 9
};


/*设置显示模式 (写数据，地址自动增加)*/
const uint8_t data_cmd = TM1637_CMD_SET_DATA | TM1637_MODE_WRITE_TO_REG | TM1637_ADDR_MODE_AUTO_INC;
/*设置起始地址 (从GRID1开始)*/
const uint8_t addr_cmd = TM1637_CMD_SET_ADDR;
/*打开显示并设置亮度 (中等亮度)*/
const uint8_t display_cmd = TM1637_CMD_SET_DISPLAY | TM1637_DISPLAY_ON | TM1637_BRIGHTNESS_10_16;

static void tm1637_start(void)
{
    gpio_set_level(TM1637_SDA, 1);
    gpio_set_level(TM1637_SCL, 1);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_SDA, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_SCL, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
}

static void tm1637_stop(void) {
    gpio_set_level(TM1637_SCL, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_SDA, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_SCL, 1);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_SDA, 1);
    esp_rom_delay_us(TM1637_DELAY_US);
}

static void tm1637_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        gpio_set_level(TM1637_SCL, 0);
        esp_rom_delay_us(TM1637_DELAY_US);
        gpio_set_level(TM1637_SDA, (byte & 0x01) ? 1 : 0);
        esp_rom_delay_us(TM1637_DELAY_US);
        gpio_set_level(TM1637_SCL, 1);
        esp_rom_delay_us(TM1637_DELAY_US);
        byte >>= 1;
    }

    // Wait for ACK
    gpio_set_level(TM1637_SCL, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
    gpio_set_direction(TM1637_SDA, GPIO_MODE_INPUT);
    gpio_set_level(TM1637_SCL, 1);
    esp_rom_delay_us(TM1637_DELAY_US);
    // ACK is when SDA is pulled low by the chip
    gpio_set_direction(TM1637_SDA, GPIO_MODE_OUTPUT);
    gpio_set_level(TM1637_SCL, 0);
    esp_rom_delay_us(TM1637_DELAY_US);
}

void display_init(void) 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TM1637_SCL) | (1ULL << TM1637_SDA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .od_en = GPIO_OPEN_DRAIN_ENABLE, 
    };
    gpio_config(&io_conf);

    tm1637_start();
    tm1637_write_byte(display_cmd);
    tm1637_stop();
}

void display_set_number(uint16_t number) 
{
    if (number > 9999) return; 

    uint8_t display_data[4];
    display_data[0] = segment_map[number / 1000];
    display_data[1] = segment_map[(number / 100) % 10];
    display_data[2] = segment_map[(number / 10) % 10];
    display_data[3] = segment_map[number % 10];

    // Set data mode
    tm1637_start();
    tm1637_write_byte(data_cmd);
    tm1637_stop();

    // Send address and the 4 data bytes
    tm1637_start();
    tm1637_write_byte(addr_cmd);
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(display_data[i]);
    }
    tm1637_stop();
}

void display_clear(void)
{ 
    tm1637_start();
    tm1637_write_byte(data_cmd);
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(addr_cmd);
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(0x00);
    }
    tm1637_stop();
}