#include "esp_log.h"
#include "esp_err.h"

#include "board_pins.h"
#include "app_config.h"

#include "driver/i2c_master.h"

i2c_master_bus_handle_t board_i2c_bus_handle;
i2c_master_bus_handle_t module_i2c_bus_handle;

static const char *TAG = "I2C Driver";

esp_err_t i2c_board_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_BOARD_I2C_PORT,
        .sda_io_num = (gpio_num_t)BOARD_I2C_SDA,
        .scl_io_num = (gpio_num_t)BOARD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = 0};

    if (i2c_new_master_bus(&i2c_bus_config, &board_i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Main I2C driver init failed.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t i2c_module_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_MOD_I2C_PORT,
        .sda_io_num = (gpio_num_t)MOD_I2C_SDA,
        .scl_io_num = (gpio_num_t)MOD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = 0};

    if (i2c_new_master_bus(&i2c_bus_config, &module_i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Main I2C driver init failed.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t i2c_board_deinit(void)
{
    return i2c_del_master_bus(board_i2c_bus_handle);
}

esp_err_t i2c_module_deinit(void)
{
    return i2c_del_master_bus(module_i2c_bus_handle);
}

esp_err_t i2c_board_rm_device(i2c_master_dev_handle_t device_handle)
{
    return i2c_master_bus_rm_device(device_handle);
}

esp_err_t i2c_module_rm_device(i2c_master_dev_handle_t device_handle)
{
    return i2c_master_bus_rm_device(device_handle);
}