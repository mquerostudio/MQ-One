#pragma once
#include "esp_err.h"

#include "board_pins.h"
#include "app_config.h"

#include "driver/i2c_master.h"

extern i2c_master_bus_handle_t board_i2c_bus_handle;
extern i2c_master_bus_handle_t module_i2c_bus_handle;

esp_err_t i2c_board_init(void);
esp_err_t i2c_module_init(void);
esp_err_t i2c_board_deinit(void);
esp_err_t i2c_module_deinit(void);
esp_err_t i2c_board_rm_device(i2c_master_dev_handle_t device_handle);
esp_err_t i2c_module_rm_device(i2c_master_dev_handle_t device_handle);