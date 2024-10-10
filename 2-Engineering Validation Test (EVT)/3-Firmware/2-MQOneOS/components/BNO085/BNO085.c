/**
 * @file BNO085.c
 * @brief BNO085 IMU driver
 */

/*********************
 *     INCLUDES
 * ******************/
#include "bno085.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/*********************
 *     DEFINES
 * *******************/
#define min(a, b) ((a) < (b) ? (a) : (b))

/*********************
 *    TYPEDEFS
 * *******************/
struct bno085_t {
  bno085_i2c_config_t i2c_config;
  sh2_Hal_t hal;
  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor
};

bno085_handle_t bno085_handle;

/*********************
 * STATIC PROTOTYPES
 * *******************/
static esp_err_t bno085_hal_init();
static int i2c_hal_open(sh2_Hal_t *self);
static void i2c_hal_close(sh2_Hal_t *self);
static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us);
static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static uint32_t i2c_hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);

/**********************
 *  STATIC VARIABLES
 **********************/
static bool _reset_occurred = false;
static sh2_SensorValue_t *_sensor_value = NULL;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t bno085_init(const bno085_i2c_config_t *config) {
  bno085_handle = (bno085_handle_t)malloc(sizeof(bno085_handle_t));
  if (bno085_handle == NULL) {
    return ESP_ERR_NO_MEM;
  }

  i2c_device_config_t i2c_dev_config = {
      .scl_speed_hz = config->i2c_dev_conf.scl_speed_hz,
      .device_address = config->i2c_dev_conf.device_address};
  esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &i2c_dev_config,
                                            &bno085_handle->i2c_config.i2c_dev);
  if (ret != ESP_OK) {
    ESP_LOGE("BNO085", "Error adding I2C device");
    free(bno085_handle);
    return ret;
  }

  bno085_handle->i2c_config.i2c_bus = config->i2c_bus;
  bno085_handle->i2c_config.i2c_dev_conf = config->i2c_dev_conf;

  return bno085_hal_init();
}

void bno085_print_product_ids() {
  for (int n = 0; n < bno085_handle->prodIds.numEntries; n++) {
    ESP_LOGI("BNO085", "Part %d: Version: %d.%d.%d Build %d",
             bno085_handle->prodIds.entry[n].swPartNumber,
             bno085_handle->prodIds.entry[n].swVersionMajor,
             bno085_handle->prodIds.entry[n].swVersionMinor,
             bno085_handle->prodIds.entry[n].swVersionPatch,
             bno085_handle->prodIds.entry[n].swBuildNumber);
  }
}

bool bno085_enable_report(sh2_SensorId_t sensorId, uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

esp_err_t bno085_set_reports(sh2_SensorId_t sensorId) {
  ESP_LOGI("BNO085", "Setting desired reports");
  if (!bno085_enable_report(sensorId, 5000)) {
    ESP_LOGE("BNO085", "Could not enable report");
    return ESP_FAIL;
  }
  ESP_LOGI("BNO085", "Report enabled");
  return ESP_OK;
}

esp_err_t bno085_get_sensor_event(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return ESP_FAIL;
  }

  return ESP_OK;
}

bool bno085_was_reset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static esp_err_t bno085_hal_init() {
  bno085_handle->hal.open = i2c_hal_open;
  bno085_handle->hal.close = i2c_hal_close;
  bno085_handle->hal.read = i2c_hal_read;
  bno085_handle->hal.write = i2c_hal_write;
  bno085_handle->hal.getTimeUs = i2c_hal_get_time_us;

  int status;

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&bno085_handle->hal, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&bno085_handle->prodIds, 0, sizeof(bno085_handle->prodIds));
  status = sh2_getProdIds(&bno085_handle->prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

static int i2c_hal_open(sh2_Hal_t *self) {
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  int ret = SH2_OK;
  for (uint8_t attempts = 0; attempts < 10; attempts++) {
    ESP_LOGI("BNO085", "Soft reset attempt %d", attempts);
    ret = i2c_master_transmit(bno085_handle->i2c_config.i2c_dev, softreset_pkt,
                              5, 100);
    if (ret == SH2_OK) {
      ESP_LOGI("BNO085", "Soft reset success");
      break;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP_LOGI("BNO085", "Soft reset done");
  return ret;
}

static void i2c_hal_close(sh2_Hal_t *self) {
  ESP_LOGI("BNO085", "Closing BNO085");
}

static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us) {
  uint8_t header[4];
  int ret = SH2_OK;

  ret = i2c_master_receive(bno085_handle->i2c_config.i2c_dev, header, 4, 100);
  // ESP_LOGI("BNO085", "Read header: %02x %02x %02x %02x", header[0],
  // header[1], header[2], header[3]);

  if (ret != SH2_OK) {
    ESP_LOGE("BNO085", "Error reading header");
    return ret;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  size_t i2c_buffer_max = 32;

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return SH2_ERR;
  }

  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    ret = i2c_master_receive(bno085_handle->i2c_config.i2c_dev, i2c_buffer,
                             read_size, 100);

    if (ret != SH2_OK) {
      return ret;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }
  return packet_size;
}

static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  int ret = SH2_OK;
  size_t i2c_buffer_max = 32;

  uint16_t write_size = min(i2c_buffer_max, len);

  ret = i2c_master_transmit(bno085_handle->i2c_config.i2c_dev, pBuffer,
                            write_size, 100);

  if (!ret == SH2_OK) {
    return ret;
  }

  return write_size;
}

static uint32_t i2c_hal_get_time_us(sh2_Hal_t *self) {
  // Get the time since boot in microseconds
  uint32_t t = (uint32_t)esp_timer_get_time();
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    ESP_LOGI("BNO08x", "Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}