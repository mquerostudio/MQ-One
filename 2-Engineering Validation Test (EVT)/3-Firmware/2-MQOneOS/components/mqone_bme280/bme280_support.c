/**
 * @file bme280_support.c
 * @brief BME280 support source file
 */

/*********************
 *     INCLUDES
 * ******************/
#include "bme280_support.h"
#include "bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/*********************
 *     DEFINES
 * *******************/
#define SAMPLE_COUNT UINT8_C(50)

/*********************
 *    TYPEDEFS
 * *******************/
/* BME280 struct */
struct bme280_t {
  bme280_i2c_config_t i2c_config;
  struct bme280_dev api_dev;
  struct bme280_settings api_settings;
  uint32_t period;
};

/*********************
 * STATIC PROTOTYPES
 * *******************/
static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr);
static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t length, void *intf_ptr);
static void bme280_delay_us(uint32_t period, void *intf_ptr);
static void bme280_error_codes_print_result(const char api_name[], int8_t rslt);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "BME280 Driver";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t bme280_init_driver(const bme280_i2c_config_t *config,
                             bme280_handle_t *out_handle) {
  struct bme280_t *result = calloc(1, sizeof(struct bme280_t));
  if (result == NULL) {
    return ESP_ERR_NO_MEM;
  }

  i2c_device_config_t i2c_dev_config = {
      .scl_speed_hz = config->i2c_dev_conf.scl_speed_hz,
      .device_address = config->i2c_dev_conf.device_address};

  esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &i2c_dev_config,
                                            &result->i2c_config.i2c_dev);
  if (ret != ESP_OK) {
    free(result);
    return ret;
  }

  result->i2c_config.i2c_bus = config->i2c_bus;
  result->i2c_config.i2c_dev_conf = config->i2c_dev_conf;

  result->api_dev.intf_ptr = &result->i2c_config.i2c_dev;
  result->api_dev.intf = BME280_I2C_INTF;
  result->api_dev.read = bme280_i2c_read;
  result->api_dev.write = bme280_i2c_write;
  result->api_dev.delay_us = bme280_delay_us;

  int8_t rslt = bme280_init(&result->api_dev);
  bme280_error_codes_print_result("bme280_init", rslt);

  if (rslt != BME280_OK) {
    free(result);
    return ESP_FAIL;
  }

  *out_handle = result;
  return ESP_OK;
}

// void bme280_deinit(void *intf_ptr)
// {
//     i2c_master_dev_handle_t handle = *((i2c_master_dev_handle_t *)intf_ptr);
//     i2c_master_deinit(handle);
// }

esp_err_t bme280_get_sensor_settings_driver(bme280_handle_t *handle) {
  return bme280_get_sensor_settings(&(*handle)->api_settings,
                                    &(*handle)->api_dev);
}

esp_err_t bme280_set_sensor_settings_driver(uint8_t desired_settings,
                                            bme280_settings_t *settings,
                                            bme280_handle_t *handle) {
  return bme280_set_sensor_settings(desired_settings,
                                    (struct bme280_settings *)settings,
                                    &(*handle)->api_dev);
}

esp_err_t bme280_set_sensor_mode_driver(uint8_t sensor_mode,
                                        bme280_handle_t *handle) {
  return bme280_set_sensor_mode(sensor_mode, &(*handle)->api_dev);
}

esp_err_t bme280_cal_meas_delay_driver(bme280_handle_t *handle) {
  return bme280_cal_meas_delay(&(*handle)->period, &(*handle)->api_settings);
}

esp_err_t bme280_get_data_sensor_driver(bme280_data_t *data,
                                        bme280_handle_t *handle) {
  int8_t rslt = BME280_E_NULL_PTR;
  uint8_t status_reg;
  struct bme280_data api_data;

  rslt =
      bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &(*handle)->api_dev);
  bme280_error_codes_print_result("bme280_get_regs", rslt);

  if (status_reg & BME280_STATUS_MEAS_DONE) {
    /* Measurement time delay given to read sample */
    (*handle)->api_dev.delay_us((*handle)->period,
                                &(*handle)->api_dev.intf_ptr);

    /* Read compensated data */
    rslt = bme280_get_sensor_data(BME280_ALL, &api_data, &(*handle)->api_dev);
    bme280_error_codes_print_result("bme280_get_sensor_data", rslt);
  }

  data->pressure = api_data.pressure;
  data->temperature = api_data.temperature;
  data->humidity = api_data.humidity;

  return rslt;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * I2C read function map to ESP32 Platform
 */
static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr) {
  i2c_master_dev_handle_t handle = *((i2c_master_dev_handle_t *)intf_ptr);

  uint8_t tx_buffer[1];
  tx_buffer[0] = reg_addr;

  return i2c_master_transmit_receive(handle, tx_buffer, sizeof(tx_buffer),
                                     reg_data, len, 100);
}

/**
 * I2C write function map to ESP32 Platform
 */
static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t length, void *intf_ptr) {
  i2c_master_dev_handle_t handle = *((i2c_master_dev_handle_t *)intf_ptr);

  uint8_t tx_buffer[1 + length];
  tx_buffer[0] = reg_addr;
  memcpy(&tx_buffer[1], reg_data, length);

  return i2c_master_transmit(handle, tx_buffer, sizeof(tx_buffer), 100);
}

/**
 * Delay function map to ESP32 Platform
 */
static void bme280_delay_us(uint32_t period, void *intf_ptr) {
  vTaskDelay(period / portTICK_PERIOD_MS);
}

/**
 * Prints the execution status of the APIs.
 */
static void bme280_error_codes_print_result(const char api_name[],
                                            int8_t rslt) {
  if (rslt != BME280_OK) {
    ESP_LOGE(TAG, "%s\t", api_name);

    switch (rslt) {
    case BME280_E_NULL_PTR:
      ESP_LOGE(TAG, "Error [%d] : Null pointer error.", rslt);
      ESP_LOGE(TAG,
               "It occurs when the user tries to assign value (not address) to "
               "a pointer, which has been initialized to NULL.");
      break;

    case BME280_E_COMM_FAIL:
      ESP_LOGE(TAG, "Error [%d] : Communication failure error.", rslt);
      ESP_LOGE(TAG, "It occurs due to read/write operation failure and also "
                    "due to power failure during communication.");
      break;

    case BME280_E_DEV_NOT_FOUND:
      ESP_LOGE(TAG,
               "Error [%d] : Device not found error. It occurs when the device "
               "chip id is incorrectly read.",
               rslt);
      break;

    case BME280_E_INVALID_LEN:
      ESP_LOGE(TAG,
               "Error [%d] : Invalid length error. It occurs when write is "
               "done with invalid length.",
               rslt);
      break;

    default:
      ESP_LOGE(TAG, "Error [%d] : Unknown error code.", rslt);
      break;
    }
  }
}
