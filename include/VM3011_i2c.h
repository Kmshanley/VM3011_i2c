#pragma once
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <i2cdev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define VM3011_I2C_ADDR_LOW 0x60
#define VM3011_I2C_ADDR_HIGH 0x61

#define VM3011_I2C_Cntrl 0x0
#define VM3011_WOS_PGA_GAIN 0x1
#define VM3011_WOS_Filter 0x2
#define VM3011_WOS_PGA_MIN_THR 0x3
#define VM3011_WOS_PGA_MAX_THR 0x4
#define VM3011_WOS_THRESH 0x5

#define I2C_FREQ_HZ 1000000

typedef enum {
    VM3011_WOS_THRESH_0X = 0b000,
    VM3011_WOS_THRESH_2X = 0b001,
    VM3011_WOS_THRESH_3X = 0b010,
    VM3011_WOS_THRESH_4X = 0b011,
    VM3011_WOS_THRESH_5X = 0b100,
    VM3011_WOS_THRESH_6X = 0b101,
    VM3011_WOS_THRESH_7X = 0b110,
    VM3011_WOS_THRESH_8X = 0b111,
} VM3011_WOS_THRESH_t;

typedef enum {
    VM3011_WOS_FILTER_HP_200k = 0b00,
    VM3011_WOS_FILTER_HP_300k = 0b01,
    VM3011_WOS_FILTER_HP_400k = 0b10,
    VM3011_WOS_FILTER_HP_800k = 0b11,
} VM3011_WOS_FILTER_HP_t;

typedef enum {
    VM3011_WOS_FILTER_LP_2k = 0b00,
    VM3011_WOS_FILTER_LP_4k = 0b01,
    VM3011_WOS_FILTER_LP_6k = 0b10,
    VM3011_WOS_FILTER_LP_8k = 0b11,
} VM3011_WOS_FILTER_LP_t;

typedef enum {
    VM3011_FAST_MODE_CNT_DISABLED = 0b00,
    VM3011_FAST_MODE_CNT_2WIN = 0b01,
    VM3011_FAST_MODE_CNT_4WIN = 0b10,
    VM3011_FAST_MODE_CNT_6WIN = 0b11,
} VM3011_FAST_MODE_CNT_t;

typedef struct
{
    i2c_dev_t i2c_dev;              //!< I2C device descriptor
} VM3011_t;

esp_err_t VM3011_init_desc(VM3011_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t VM3011_free_desc(VM3011_t *dev);
uint8_t VM3011_get_ambient(VM3011_t *dev);
void VM3011_set_WOS_THRESH(VM3011_t *dev, VM3011_WOS_THRESH_t thres);
void VM3011_set_WOS_Filter(VM3011_t *dev, VM3011_WOS_FILTER_LP_t lp_freq, VM3011_WOS_FILTER_HP_t hp_freq);
void VM3011_set_PGA_min(VM3011_t *dev, uint8_t thres, VM3011_FAST_MODE_CNT_t fmode_cnt);
void VM3011_set_PGA_max(VM3011_t *dev, uint8_t thres, bool WOS_RMS);
void VM3011_clear_int(VM3011_t *dev);