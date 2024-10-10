/**
 * @file bme280_support.h
 * @brief BME280 support header file
 */

#ifndef BME280_SUPPORT_H
#define BME280_SUPPORT_H

/*********************
 *      INCLUDES
 *********************/
#include "bme280_defs.h"
#include "driver/i2c_master.h"

/*********************
 *     DEFINES
 * *******************/

/*********************
 *    TYPEDEFS
 * *******************/
/**
 * @brief BME280 device configuration
 */
typedef struct {
  i2c_master_bus_handle_t i2c_bus;
  i2c_device_config_t i2c_dev_conf;
  i2c_master_dev_handle_t i2c_dev;
} bme280_i2c_config_t;

typedef struct {
  uint8_t osr_p;
  uint8_t osr_t;
  uint8_t osr_h;
  uint8_t filter;
  uint8_t standby_time;
} bme280_settings_t;

typedef struct {
  uint32_t pressure;
  int32_t temperature;
  uint32_t humidity;
} bme280_data_t;

/**
 * @brief BME280 driver handle
 */
typedef struct bme280_t *bme280_handle_t;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL PROTOTYPES
 **********************/
esp_err_t bme280_init_driver(const bme280_i2c_config_t *config,
                             bme280_handle_t *out_handle);
// esp_err_t bme280_deinit(bme280_handle_t handle);
esp_err_t bme280_get_sensor_settings_driver(bme280_handle_t *handle);
esp_err_t bme280_set_sensor_settings_driver(uint8_t desired_settings,
                                            bme280_settings_t *settings,
                                            bme280_handle_t *handle);
esp_err_t bme280_set_sensor_mode_driver(uint8_t sensor_mode,
                                        bme280_handle_t *handle);
esp_err_t bme280_cal_meas_delay_driver(bme280_handle_t *handle);
esp_err_t bme280_get_data_sensor_driver(bme280_data_t *data,
                                        bme280_handle_t *handle);

/**********************
 *   MACROS
 **********************/

#endif /* BME280_SUPPORT_H */
