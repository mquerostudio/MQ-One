/**
 * @file bno085.h
 * @brief BNO085 IMU driver
 */

#ifndef BNO085_H
#define BNO085_H

/*********************
 *      INCLUDES
 *********************/
#include "driver/i2c_master.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/*********************
 *     DEFINES
 * *******************/

/*********************
 *    TYPEDEFS
 * *******************/
/**
 * @brief BNO085 device configuration
 */
typedef struct {
  i2c_master_bus_handle_t i2c_bus;
  i2c_device_config_t i2c_dev_conf;
  i2c_master_dev_handle_t i2c_dev;
} bno085_i2c_config_t;

/**
 * @brief BNO085 driver handle
 */
typedef struct bno085_t *bno085_handle_t;

// extern bno085_handle_t bno085_handle;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL PROTOTYPES
 **********************/
esp_err_t bno085_init(const bno085_i2c_config_t *config);
void bno085_print_product_ids();
bool bno085_enable_report(sh2_SensorId_t sensorId, uint32_t interval_us);
esp_err_t bno085_set_reports(sh2_SensorId_t sensorId);
esp_err_t bno085_get_sensor_event(sh2_SensorValue_t *value);
bool bno085_was_reset(void);

/**********************
 *   MACROS
 **********************/

#endif // BNO085_H