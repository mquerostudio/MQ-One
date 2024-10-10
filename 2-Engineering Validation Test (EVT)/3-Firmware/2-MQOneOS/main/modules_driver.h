/**
 * @file one_wire_driver.h
 * @brief One Wire Driver
 */

#ifndef ONE_WIRE_DRIVER_H
#define ONE_WIRE_DRIVER_H

/*********************
 *      INCLUDES
 *********************/
#include "driver/gpio.h"
#include "ds28e07.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "onewire_bus.h"

/*********************
 *      DEFINES
 *********************/
/* Define Modules ROM IDs */
#define MODULE_DEV_ROM_ID 0x12000048589B7D2D
#define MODULE_CAMERA_ROM_ID 0x4B000048589B7E2D
#define MODULE_MOUNTAIN_ROM_ID 0x1D000048589B812D

/**********************
 *      TYPEDEFS
 **********************/
struct module_t {
  uint64_t id;
  const char *name;
  bool need_i2c;
  ds28e07_handle_t eeprom_handle;
};

typedef struct module_t *module_handle_t;

extern module_handle_t module_handle;

typedef struct module_t module_model_t;

extern TaskHandle_t task_gps_module_handle;
extern TaskHandle_t task_camera_module_handle;
extern TaskHandle_t task_ambient_sensor_module_handle;
extern TaskHandle_t imu_task_handle;
extern TaskHandle_t neopixel_task_handle;
extern TaskHandle_t microsd_task_handle;
extern TaskHandle_t app_start_task_handle;

static const module_model_t module_model[] = {
    {.id = MODULE_DEV_ROM_ID, .name = "DEV", .need_i2c = false},
    {.id = MODULE_MOUNTAIN_ROM_ID, .name = "MOUNTAIN", .need_i2c = true},
    {.id = MODULE_CAMERA_ROM_ID, .name = "CAMERA", .need_i2c = true},
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/
esp_err_t one_wire_init();
esp_err_t module_search();
esp_err_t module_init();
esp_err_t module_deinit();

void task_gps_module(void *param);
void task_ambient_sensor_module(void *param);
void task_camera_module(void *param);
void imu_task(void *param);
void neopixel_task(void *param);
void microsd_task(void *param);
void app_start_task(void *param);

/**********************
 *      MACROS
 **********************/

#endif /* ONE_WIRE_DRIVER_H */