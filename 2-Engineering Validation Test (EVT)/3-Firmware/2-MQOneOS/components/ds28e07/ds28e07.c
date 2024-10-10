/**
 * @file ds28e07.C
 * @brief DS28E07 Driver
 */

/*********************
 *      INCLUDES
 *********************/
#include "ds28e07.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "onewire_bus.h"
#include "onewire_cmd.h"
#include "onewire_crc.h"
#include <string.h>

/*********************
 *     DEFINES
 * *******************/
#define DS28E07_CMD_READ_ROM 0x33
#define DS28E07_CMD_RESUME 0xA5
#define DS28E07_CMD_OVERDRIVE_SKIP 0x3C
#define DS28E07_CMD_OVERDRIVE_MATCH 0x69
#define DS28E07_CMD_WRITE_SCRATCHPAD 0x0F
#define DS28E07_CMD_READ_SCRATCHPAD 0xAA
#define DS28E07_CMD_COPY_SCRATCHPAD 0x55
#define DS28E07_CMD_READ_MEMORY 0xF0

/*********************
 *    TYPEDEFS
 * *******************/

/*********************
 * STATIC PROTOTYPES
 * *******************/
// static esp_err_t ds18b20_send_command(ds28e07_handle_t ds28e07, uint8_t cmd);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "DS28E07 Driver";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
ds28e07_handle_t ds28e07_init(const onewire_device_t bus_handle,
                              ds28e07_handle_t *handle) {
  ds28e07_handle_t out_handle = (ds28e07_handle_t)calloc(1, sizeof(out_handle));
/*  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_NO_MEM, TAG,
                      "Failed to allocate memory for DS28E07 handle");*/

  out_handle->data = 0;

  *handle = out_handle;

  return ESP_OK;
}

esp_err_t ds28e07_deinit(ds28e07_handle_t *handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  free(handle);
  return ESP_OK;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
// static esp_err_t ds18b20_send_command(ds28e07_handle_t ds28e07, uint8_t cmd)
// {
//     // send command
//     uint8_t tx_buffer[10] = {0};
//     tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
//     memcpy(&tx_buffer[1], &ds28e07->device.address,
//     sizeof(ds28e07->device.address));
//     tx_buffer[sizeof(ds28e07->device.address) + 1] = cmd;

//     return onewire_bus_write_bytes(ds28e07->device.bus, tx_buffer,
//     sizeof(tx_buffer));
// }
