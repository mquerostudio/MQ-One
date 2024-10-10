/**
 * @file ds28e07.h
 * @brief DS28E07 Driver
 */

#ifndef DS28E07_H
#define DS28E07_H

/*********************
 *      INCLUDES
 *********************/
#include "onewire_device.h"

/*********************
 *     DEFINES
 * *******************/

/*********************
 *     TYPEDEFS
 * *******************/

/* DS28E07 device structure */
struct ds28e07_t {
  uint64_t data;
};

/* Handle to a DS28E07 device */
typedef struct ds28e07_t *ds28e07_handle_t;

/*********************
 * GLOBAL PROTOTYPES
 * *******************/
ds28e07_handle_t ds28e07_init(const onewire_device_t bus_handle,
                              ds28e07_handle_t *handle);
esp_err_t ds28e07_deinit(ds28e07_handle_t *handle);

/**********************
 *      MACROS
 **********************/

#endif /* DS28E07_H */