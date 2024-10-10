/**
 * @file mqone_pa1010d.c
 * @brief MQONE_PA1010D Driver
 */

/*********************
 *     INCLUDES
 * ******************/
#include "mqone_pa1010d.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"
#include <stdio.h>

/*********************
 *     DEFINES
 * *******************/

/*********************
 *    TYPEDEFS
 * *******************/
/* PA1010D struct */
struct pa1010d_t {
  pa1010d_i2c_config_t i2c_config;
  /* no other state to store for now */
};

/*********************
 * STATIC PROTOTYPES
 * *******************/
static esp_err_t read_one_byte(pa1010d_handle_t handle, char *out_byte,
                               unsigned timeout_ms);
static esp_err_t nmea_read_line(pa1010d_handle_t handle, char **out_line_buf,
                                size_t *out_line_len, int timeout_ms);
/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "PA1010D Driver";

static char s_buf[1024 + 1];
static size_t s_total_bytes;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

esp_err_t pa1010d_init(const pa1010d_i2c_config_t *config,
                       pa1010d_handle_t *out_handle) {
  struct pa1010d_t *result = calloc(1, sizeof(struct pa1010d_t));
  if (result == NULL) {
    return ESP_ERR_NO_MEM;
  }

  i2c_device_config_t i2c_dev_config = {
      .device_address = config->i2c_dev_conf.device_address,
      .scl_speed_hz = config->i2c_dev_conf.scl_speed_hz};

  esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &i2c_dev_config,
                                            &result->i2c_config.i2c_dev);
  if (ret != ESP_OK) {
    free(result);
    return ret;
  }
  result->i2c_config.i2c_bus = config->i2c_bus;
  result->i2c_config.i2c_dev_conf = config->i2c_dev_conf;

  *out_handle = result;
  return ESP_OK;
}

esp_err_t pa1010d_deinit(pa1010d_handle_t *handle) {
  free(handle);
  return ESP_OK;
}

esp_err_t pa1010d_get_nmea_msg(pa1010d_handle_t *handle, char *out_buf,
                               size_t out_buf_len, unsigned timeout_ms) {
  char c;
  size_t len = 0;
  esp_err_t err;

  if (out_buf_len < 4) {
    return ESP_ERR_INVALID_ARG;
  }

  do {
    err = read_one_byte(*handle, &c, timeout_ms);
    if (err != ESP_OK) {
      return err;
    }
  } while (c != '$');

  out_buf[len++] = c;

  do {
    err = read_one_byte(*handle, &c, timeout_ms);
    if (err != ESP_OK) {
      return err;
    }
    if (c == 0) {
      continue;
    }
    if (c == '\n') {
      break;
    }
    if (c == '\r') {
      continue;
    }
    out_buf[len++] = c;
  } while (len < out_buf_len - 1);

  out_buf[len] = '\0';

  return ESP_OK;
}

void *pa1010d_read_parse_nmea(pa1010d_handle_t *handle, nmea_t type) {
  nmea_s *data;
  char *start;
  size_t length;
  esp_err_t err = ESP_OK;

  err = nmea_read_line(*handle, &start, &length, 100 /* ms */);

  if (err != ESP_OK) {
    return NULL;
  }

  if (length == 0) {
    ESP_LOGE(TAG, "No data read from PA1010D");
    return NULL;
  }

  data = nmea_parse(start, length, 0);

  if (data == NULL) {
    /*    ESP_LOGE(TAG, "Failed to parse NMEA sentence: %.5s (%d)", start + 1,
                 nmea_get_type(start));*/
    return NULL;
  }

  if (data->errors != 0) {
    ESP_LOGW(TAG, "The sentence contains parse errors");
    nmea_free(data);
    return NULL;
  }

  if (data->type == type) {
    void *message_data = NULL;

    switch (type) {
    case NMEA_GPGGA:
      message_data = (nmea_gpgga_s *)malloc(sizeof(nmea_gpgga_s));
      if (message_data != NULL) {
        *(nmea_gpgga_s *)message_data = *(nmea_gpgga_s *)data;
      }
      break;

    case NMEA_GPGLL:
      message_data = (nmea_gpgll_s *)malloc(sizeof(nmea_gpgll_s));
      if (message_data != NULL) {
        *(nmea_gpgll_s *)message_data = *(nmea_gpgll_s *)data;
      }
      break;

    case NMEA_GPRMC:
      message_data = (nmea_gprmc_s *)malloc(sizeof(nmea_gprmc_s));
      if (message_data != NULL) {
        *(nmea_gprmc_s *)message_data = *(nmea_gprmc_s *)data;
      }
      break;

    case NMEA_GPGSA:
      message_data = (nmea_gpgsa_s *)malloc(sizeof(nmea_gpgsa_s));
      if (message_data != NULL) {
        *(nmea_gpgsa_s *)message_data = *(nmea_gpgsa_s *)data;
      }
      break;

    case NMEA_GPGSV:
      message_data = (nmea_gpgsv_s *)malloc(sizeof(nmea_gpgsv_s));
      if (message_data != NULL) {
        *(nmea_gpgsv_s *)message_data = *(nmea_gpgsv_s *)data;
      }
      break;

    case NMEA_GPVTG:
      message_data = (nmea_gpvtg_s *)malloc(sizeof(nmea_gpvtg_s));
      if (message_data != NULL) {
        *(nmea_gpvtg_s *)message_data = *(nmea_gpvtg_s *)data;
      }
      break;

    case NMEA_GPTXT:
      message_data = (nmea_gptxt_s *)malloc(sizeof(nmea_gptxt_s));
      if (message_data != NULL) {
        *(nmea_gptxt_s *)message_data = *(nmea_gptxt_s *)data;
      }
      break;

    default:
      ESP_LOGE(TAG, "Unknown NMEA message type");
      break;
    }

    nmea_free(data);
    return message_data;
  }

  nmea_free(data);
  return NULL;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Reads one byte from the PA1010D device.
 *
 * This function reads one byte from the PA1010D device using the specified I2C
 * handle.
 *
 * @param handle The handle to the PA1010D device.
 * @param out_byte Pointer to store the read byte.
 * @param timeout_ms The timeout value in milliseconds.
 * @return `ESP_OK` if the byte was successfully read, or an error code if an
 * error occurred.
 */
static esp_err_t read_one_byte(pa1010d_handle_t handle, char *out_byte,
                               unsigned timeout_ms) {
  return i2c_master_receive(handle->i2c_config.i2c_dev, (uint8_t *)out_byte, 1,
                            timeout_ms);
}

/**
 * @brief Reads a line from the NMEA data stream.
 *
 * This function reads a line from the NMEA data stream and stores it in the
 * provided buffer. The line is terminated by a newline character ('\n').
 *
 * @param handle The handle to the PA1010D device.
 * @param[out] out_line_buf Pointer to the buffer where the line will be stored.
 * @param[out] out_line_len Pointer to the variable where the length of the line
 * will be stored.
 * @param timeout_ms The timeout value in milliseconds for reading a byte over
 * I2C.
 *
 * @note The function will wait for a byte to be available if none is currently
 * available.
 * @note The function will return immediately if an error occurs while reading a
 * byte over I2C.
 * @note The line buffer must be large enough to accommodate the entire line,
 * otherwise an overflow error will be logged.
 */
static esp_err_t nmea_read_line(pa1010d_handle_t handle, char **out_line_buf,
                                size_t *out_line_len, int timeout_ms) {
  *out_line_buf = NULL;
  *out_line_len = 0;
  esp_err_t err = ESP_OK;

  bool line_start_found = false;

  while (true) {
    char b;
    err = read_one_byte(handle, &b, timeout_ms);
    if (err != ESP_OK) {
      ESP_LOGI(TAG, "Failed to read byte over I2C: %s (0x%x)",
               esp_err_to_name(err), err);
      return err;
    }

    if (b == 0) {
      /* No byte available yet, wait a bit and retry */
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (!line_start_found) {
      if (b != '$') {
        continue;
      } else {
        line_start_found = true;
      }
    }

    s_buf[s_total_bytes] = b;
    ++s_total_bytes;

    if (b == '\n') {
      *out_line_buf = s_buf;
      *out_line_len = s_total_bytes;
      s_total_bytes = 0;
      return err;
    }

    if (s_total_bytes == sizeof(s_buf)) {
      ESP_LOGE(TAG, "Line buffer overflow");
      s_total_bytes = 0;
    }
  }
}