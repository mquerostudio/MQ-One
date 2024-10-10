/**
 * @file mqone_pa1010d.h
 * @brief MQONE_PA1010D Driver
 */

/*
'$GNGGA,140507.306,,,,,0,0,,,M,,M,,*54' --> Time, position and fix type data.
'$GPGSA,A,1,,,,,,,,,,,,,,,*1E' --> GPS receiver operating mode, satellites used
in the position solution, and DOP values.
'$GLGSA,A,1,,,,,,,,,,,,,,,*02' --> same
'$GNRMC,140507.306,V,,,,,0.00,0.00,050924,,,N*5B' --> Time, date, position,
course and speed data
'$GNVTG,0.00,T,,M,0.00,N,0.00,K,N*2C' --> Course and speed information relative
to the ground.
*/

#ifndef MQONE_PA1010D_H
#define MQONE_PA1010D_H

/*********************
 *      INCLUDES
 *********************/
#include "driver/i2c_master.h"
#include "gpgga.h"
#include "gpgll.h"
#include "gpgsa.h"
#include "gpgsv.h"
#include "gprmc.h"
#include "gptxt.h"
#include "gpvtg.h"
#include "nmea.h"

/*********************
 *     DEFINES
 * *******************/

/*********************
 *    TYPEDEFS
 * *******************/
/**
 * @brief MQONE_PA1010D device configuration
 */
typedef struct {
  i2c_master_bus_handle_t i2c_bus;
  i2c_device_config_t i2c_dev_conf;
  i2c_master_dev_handle_t i2c_dev;
} pa1010d_i2c_config_t;

/**
 * @brief PA1010D driver handle
 */
typedef struct pa1010d_t *pa1010d_handle_t;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL PROTOTYPES
 **********************/
/**
 * @brief Initialize the PA1010D driver
 *
 * @param config Pointer to the configuration struct. The driver makes a copy,
 * so can point to a local variable.
 * @param[out] out_handle  Pointer to a variable to receive the driver handle.
 * @return esp_err_t  ESP_OK on success, ESP_ERR_NO_MEM if out of memory.
 */
esp_err_t pa1010d_init(const pa1010d_i2c_config_t *config,
                       pa1010d_handle_t *out_handle);

/**
 * @brief Deinitialize the PA1010D driver
 *
 * @param handle Driver handle obtained from pa1010d_init(), or NULL
 * @return esp_err_t  ESP_OK on success.
 */
esp_err_t pa1010d_deinit(pa1010d_handle_t *handle);

/**
 * @brief Retrieves an NMEA message from the PA1010D module.
 *
 * This function reads NMEA messages from the PA1010D module and stores them in
 * the provided output buffer. The function reads one byte at a time until it
 * encounters the start of an NMEA message ('$' character), and then continues
 * reading until it reaches the end of the message ('\n' character).
 *
 * @param handle Pointer to the PA1010D handle structure.
 * @param out_buf Pointer to the output buffer where the NMEA message will be
 * stored.
 * @param out_buf_len Length of the output buffer.
 * @param timeout_ms Timeout value in milliseconds for reading each byte.
 * @return `ESP_OK` if the NMEA message is successfully retrieved, or an error
 * code if an error occurred. Possible error codes include:
 *         - `ESP_ERR_INVALID_ARG` if the output buffer length is less than 4.
 *         - Other specific error codes related to reading bytes from the
 * PA1010D module.
 */
esp_err_t pa1010d_get_nmea_msg(pa1010d_handle_t *handle, char *out_buf,
                               size_t out_buf_len, unsigned timeout_ms);

/**
 * @brief Reads and parses an NMEA sentence from the PA1010D device.
 *
 * This function reads a line of NMEA data from the device and parses it into a
 * specific NMEA message type. It checks for errors in the sentence and returns
 * the parsed message data if the type matches the expected type.
 *
 * @param handle Pointer to the PA1010D handle structure.
 * @param type The expected NMEA message type.
 * @return A pointer to the parsed message data if successful, or NULL if there
 * was an error or the type does not match.
 */
void *pa1010d_read_parse_nmea(pa1010d_handle_t *handle, nmea_t type);

/**********************
 *   MACROS
 **********************/
#endif /* MQONE_PA1010D_H */