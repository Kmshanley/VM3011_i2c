#include "VM3011_i2c.h"

static const char *TAG = "VM3011 driver";

esp_err_t VM3011_init_desc(VM3011_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    if (addr != VM3011_I2C_ADDR_LOW &&  addr != VM3011_I2C_ADDR_HIGH)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

uint8_t VM3011_get_ambient(VM3011_t *dev)
{
    uint8_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_read_reg(&dev->i2c_dev, VM3011_WOS_PGA_GAIN, &data, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return data;
}

void VM3011_set_WOS_THRESH(VM3011_t *dev, VM3011_WOS_THRESH_t thres)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, VM3011_WOS_THRESH, &thres, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

void VM3011_set_WOS_Filter(VM3011_t *dev, VM3011_WOS_FILTER_LP_t lp_freq, VM3011_WOS_FILTER_HP_t hp_freq)
{
    uint8_t setting = hp_freq;
    setting = setting << 2;
    setting |= lp_freq;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, VM3011_WOS_Filter, &setting, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

void VM3011_set_PGA_min(VM3011_t *dev, uint8_t thres, VM3011_FAST_MODE_CNT_t fmode_cnt)
{
    uint8_t setting = fmode_cnt;
    setting = setting << 5;
    setting |= thres;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, VM3011_WOS_PGA_MIN_THR, &setting, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

}

void VM3011_set_PGA_max(VM3011_t *dev, uint8_t thres, bool WOS_RMS)
{
    uint8_t setting = WOS_RMS;
    setting = setting << 5;
    setting |= thres;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, VM3011_WOS_PGA_MAX_THR, &setting, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

void VM3011_clear_int(VM3011_t *dev) 
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data;
    i2c_dev_read_reg(&dev->i2c_dev, VM3011_I2C_Cntrl, &data, 1);
    data |= 0b1000;
    i2c_dev_write_reg(&dev->i2c_dev, VM3011_I2C_Cntrl, &data, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}