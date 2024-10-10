#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "seesaw.h"

static const char *TAG = "Seesaw Driver";

seesaw_handle_t seesaw_handle;

/**
 * @brief Initializes the seesaw device.
 *
 * This function initializes the seesaw device with the provided configuration.
 *
 * @param bus_handle The handle to the I2C master bus.
 * @param seesaw_conf The configuration for the seesaw device.
 * @param seesaw_handle The handle to the seesaw device.
 * @return
 *     - ESP_OK if the seesaw device is successfully initialized.
 *     - Error code if initialization fails.
 */
esp_err_t seesaw_init(const i2c_master_bus_handle_t bus_handle,
                      const i2c_device_config_t *seesaw_conf,
                      seesaw_handle_t *seesaw_handle) {
  seesaw_handle_t out_handle = (seesaw_handle_t)calloc(1, sizeof(out_handle));
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_NO_MEM, TAG,
                      "Failed to allocate memory for seesaw handle");

  i2c_device_config_t i2c_dev_conf = {.scl_speed_hz = seesaw_conf->scl_speed_hz,
                                      .device_address =
                                          seesaw_conf->device_address};

  esp_err_t ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf,
                                            &out_handle->i2c_dev);
  if (ret != ESP_OK) {
    free(out_handle);
    return ret;
  }

  *seesaw_handle = out_handle;

  return ESP_OK;
}
/**
 * @brief Performs a software reset on the seesaw device.
 *
 * This function sends a command to the seesaw device to perform a software
 * reset. It writes a value of 0xFF to the SEESAW_STATUS_SWRST register of the
 * device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return `ESP_OK` if the software reset command was successfully sent, or an
 * error code if an error occurred.
 */

esp_err_t seesaw_SW_reset(const seesaw_handle_t seesaw_handle) {
  return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_STATUS_BASE,
                       SEESAW_STATUS_SWRST, 0xFF);
}

/**
 * @brief Turns on the seesaw device.
 *
 * This function turns on the seesaw device by setting the specified pin as
 * input, enabling pull-up, and setting the pin to a high state.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return
 *     - ESP_OK if the seesaw device is turned on successfully.
 *     - An error code if there was an error turning on the seesaw device.
 */
esp_err_t seesaw_turn_on(const seesaw_handle_t seesaw_handle) {

  esp_err_t ret = ESP_OK;
  uint8_t pins = (1ul << 7);
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                     SEESAW_GPIO_DIRCLR_BULK, cmd, 4); // set as input
  if (ret != ESP_OK) {
    return ret;
  }

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                     SEESAW_GPIO_PULLENSET, cmd, 4); // enable pullup
  if (ret != ESP_OK) {
    return ret;
  }

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                     SEESAW_GPIO_BULK_SET, cmd, 4); // set high
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

/**
 * @brief Turns off the specified seesaw device.
 *
 * This function sets the specified seesaw device pin as an output and sets it
 * to a low state.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return
 *     - ESP_OK if the seesaw device is turned off successfully.
 *     - An error code if there was an error turning off the seesaw device.
 */
esp_err_t seesaw_turn_off(const seesaw_handle_t seesaw_handle) {

  esp_err_t ret = ESP_OK;
  uint8_t pins = (1ul << 7);
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                     SEESAW_GPIO_DIRSET_BULK, cmd, 4); // set as output
  if (ret != ESP_OK) {
    return ret;
  }

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                     SEESAW_GPIO_BULK_CLR, cmd, 4); // set low
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

/**
 * @brief Set the mode of a pin on the seesaw device.
 *
 * This function sets the mode of a specific pin on the seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pin The pin number to set the mode for.
 * @param mode The mode to set for the pin.
 * @return `ESP_OK` on success, or an error code if an error occurred.
 */
esp_err_t seesaw_pin_mode(const seesaw_handle_t seesaw_handle, uint8_t pin,
                          uint8_t mode) {
  return seesaw_pin_mode_bulk(seesaw_handle, (1ul << pin), mode);
}

/**
 * @brief Sets the pin mode for multiple pins on a seesaw device.
 *
 * This function sets the pin mode for multiple pins on a seesaw device. The
 * pins parameter is a bitmask where each bit represents a pin. The mode
 * parameter specifies the desired pin mode, which can be one of the following:
 * - OUTPUT: Sets the pins as output.
 * - INPUT: Sets the pins as input.
 * - INPUT_PULLUP: Sets the pins as input with pull-up resistors enabled.
 * - INPUT_PULLDOWN: Sets the pins as input with pull-down resistors enabled.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pins The bitmask representing the pins to set the mode for.
 * @param mode The desired pin mode.
 * @return esp_err_t Returns ESP_OK if the operation is successful, otherwise an
 * error code.
 */
esp_err_t seesaw_pin_mode_bulk(const seesaw_handle_t seesaw_handle,
                               uint32_t pins, uint8_t mode) {
  esp_err_t ret = ESP_OK;
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};

  switch (mode) {
  case OUTPUT:
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_DIRSET_BULK, cmd, 4);
    break;
  case INPUT:
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
    break;
  case INPUT_PULLUP:
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_PULLENSET, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_BULK_SET, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    break;
  case INPUT_PULLDOWN:
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_PULLENSET, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_BULK_CLR, cmd, 4);
    if (ret != ESP_OK) {
      break;
    }
    break;
  }
  return ESP_OK;
}

/**
 * @brief Writes a digital value to a pin on the seesaw device.
 *
 * This function writes a digital value to the specified pin on the seesaw
 * device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pin The pin number to write to.
 * @param value The digital value to write (0 or 1).
 *
 * @return `ESP_OK` on success, or an error code if an error occurred.
 */
esp_err_t seesaw_digital_write(const seesaw_handle_t seesaw_handle, uint8_t pin,
                               uint8_t value) {
  return seesaw_digital_write_bulk(seesaw_handle, (1ul << pin), value);
}

/**
 * @brief Writes a digital value to multiple pins on a seesaw device.
 *
 * This function writes a digital value to multiple pins on a seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pins The pins to write to. Each bit represents a pin, where a set bit
 * indicates the corresponding pin should be written to.
 * @param value The value to write to the pins. A non-zero value sets the pins
 * to high, while a zero value sets the pins to low.
 *
 * @return
 *     - ESP_OK: Success
 *     - Error code: Fail
 */
esp_err_t seesaw_digital_write_bulk(const seesaw_handle_t seesaw_handle,
                                    uint32_t pins, uint8_t value) {
  esp_err_t ret = ESP_OK;
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};

  if (value) {
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_BULK_SET, cmd, 4);
  } else {
    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE,
                       SEESAW_GPIO_BULK_CLR, cmd, 4);
  }

  return ret;
}

/**
 * @brief Reads the digital value of a pin on the seesaw device.
 *
 * This function reads the digital value of the specified pin on the seesaw
 * device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pin The pin number to read the digital value from.
 * @return The digital value of the pin (true or false).
 */
bool seesaw_digital_read(const seesaw_handle_t seesaw_handle, uint8_t pin) {
  return seesaw_digital_read_bulk(seesaw_handle, (1ul << pin)) != 0;
}

/**
 * @brief Reads the digital values of multiple pins in bulk.
 *
 * This function reads the digital values of multiple pins in bulk from the
 * seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param pins The bitmask representing the pins to read.
 *
 * @return The digital values of the specified pins.
 *         If an error occurs, the error code is returned.
 */
uint32_t seesaw_digital_read_bulk(const seesaw_handle_t seesaw_handle,
                                  uint32_t pins) {
  esp_err_t ret = ESP_OK;
  uint8_t buf[4];

  ret = seesaw_read(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK,
                    buf, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error reading digital value");
    return ret;
  }

  uint32_t ret1 = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                  ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

  return ret1 & pins;
}

/**
 * @brief Get the current position of the encoder.
 *
 * This function reads the current position of the encoder from the seesaw
 * device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return The current position of the encoder.
 * @note This function returns an error code if there was an error reading the
 * encoder position.
 */
int32_t seesaw_encoder_get_position(const seesaw_handle_t seesaw_handle) {
  esp_err_t ret = ESP_OK;
  uint8_t buf[4];

  ret = seesaw_read(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE,
                    SEESAW_ENCODER_POSITION, buf, 4);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error reading encoder position");
    return ret;
  } else {
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                  ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
  }
}

/**
 * @brief Get the delta value of the encoder.
 *
 * This function reads the delta value of the encoder from the seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return The delta value of the encoder.
 * @note This function returns an error code if there was an error reading the
 * encoder delta.
 */
int32_t seesaw_encoder_get_delta(const seesaw_handle_t seesaw_handle) {
  esp_err_t ret = ESP_OK;
  uint8_t buf[4];

  ret = seesaw_read(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE,
                    SEESAW_ENCODER_DELTA, buf, 4);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error reading encoder delta");
    return ret;
  } else {
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                  ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
  }
}

/**
 * @brief Sets the position of the encoder.
 *
 * This function sets the position of the encoder using the specified handle and
 * position value.
 *
 * @param seesaw_handle The handle to the I2C master device.
 * @param pos The position value to set for the encoder.
 * @return `ESP_OK` if the position was set successfully, or an error code if an
 * error occurred.
 */
esp_err_t seesaw_encoder_set_position(const seesaw_handle_t seesaw_handle,
                                      int32_t pos) {
  uint8_t buf[] = {(uint8_t)(pos >> 24), (uint8_t)(pos >> 16),
                   (uint8_t)(pos >> 8), (uint8_t)(pos & 0xFF)};

  return seesaw_write(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE,
                      SEESAW_ENCODER_POSITION, buf, 4);
}

/**
 * @brief Enables the interrupt for the encoder on the seesaw device.
 *
 * This function enables the interrupt for the encoder on the seesaw device
 * specified by the given `seesaw_handle`.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return `ESP_OK` if the interrupt was successfully enabled, or an error code
 * if it failed.
 */
esp_err_t seesaw_encoder_enable_interrupt(const seesaw_handle_t seesaw_handle) {
  return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE,
                       SEESAW_ENCODER_INTENSET, 0x01);
}

/**
 * @brief Disable the interrupt for the encoder on the seesaw device.
 *
 * This function disables the interrupt for the encoder on the seesaw device
 * specified by the given I2C master device handle.
 *
 * @param seesaw_handle The I2C master device handle for the seesaw device.
 * @return `ESP_OK` if the interrupt was successfully disabled, or an error code
 * if not.
 */
esp_err_t
seesaw_encoder_disable_interrupt(const seesaw_handle_t seesaw_handle) {
  return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE,
                       SEESAW_ENCODER_INTENCLR, 0x01);
}

/**
 * @brief Writes an 8-bit value to the specified register address of the seesaw
 * device.
 *
 * This function writes an 8-bit value to the specified register address of the
 * seesaw device using the provided I2C master device handle.
 *
 * @param i2c_dev The I2C master device handle.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param value The 8-bit value to be written.
 * @return `ESP_OK` if the write operation is successful, otherwise an error
 * code.
 */
esp_err_t seesaw_write8(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                        uint8_t regLow, uint8_t value) {
  return seesaw_write(i2c_dev, regHigh, regLow, &value, 1);
}

/**
 * @brief Writes data to the seesaw device via I2C.
 *
 * This function writes data to the seesaw device using the specified I2C master
 * device handle. The data to be written is provided in the `buf` array, with
 * the number of bytes specified by `num`. The register address to write to is
 * specified by `regHigh` and `regLow`.
 *
 * @param i2c_dev The I2C master device handle.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param buf Pointer to the data buffer.
 * @param num The number of bytes to write.
 *
 * @return `ESP_OK` if the write operation is successful, otherwise an error
 * code.
 */
esp_err_t seesaw_write(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                       uint8_t regLow, uint8_t *buf, uint8_t num) {

  uint8_t buffer[2 + num];
  buffer[0] = (uint8_t)regHigh;
  buffer[1] = (uint8_t)regLow;
  for (int i = 0; i < num; i++) {
    buffer[i + 2] = buf[i];
  }

  return i2c_master_transmit(i2c_dev, buffer, sizeof(buffer), -1);
}

/**
 * @brief Reads data from the seesaw device.
 *
 * This function reads data from the seesaw device using the specified I2C
 * master device handle.
 *
 * @param i2c_dev The I2C master device handle.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param buf Pointer to the buffer to store the read data.
 * @param num The number of bytes to read.
 * @return `ESP_OK` on success, or an error code if an error occurred.
 */
esp_err_t seesaw_read(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                      uint8_t regLow, uint8_t *buf, uint8_t num) {

  uint8_t prefix[2];
  prefix[0] = (uint8_t)regHigh;
  prefix[1] = (uint8_t)regLow;

  return i2c_master_transmit_receive(i2c_dev, prefix, sizeof(prefix), buf,
                                     sizeof(buf), -1);
}

/**
 * @brief Initializes the NeoPixel module of the SeeSaw component.
 *
 * This function initializes the NeoPixel module of the SeeSaw component with
 * the provided configuration.
 *
 * @param seesaw_handle The handle to the SeeSaw component.
 * @param neopixel_config The configuration for the NeoPixel module.
 * @return
 *     - ESP_OK if the NeoPixel module is successfully initialized.
 *     - Error code if initialization fails.
 */
esp_err_t seesaw_neopixel_init(const seesaw_handle_t seesaw_handle,
                               const neopixel_config_t *neopixel_config) {
  esp_err_t ret = ESP_OK;

  seesaw_handle->neopixel_handle->brightness = 0;
  seesaw_handle->neopixel_handle->pixels = NULL;
  seesaw_handle->neopixel_handle->endTime = 0;
  seesaw_handle->neopixel_handle->numLEDs = neopixel_config->numLEDs;
  seesaw_handle->neopixel_handle->pin = neopixel_config->pin;
  seesaw_handle->neopixel_handle->type = neopixel_config->type;

  ret = seesaw_neopixel_update_type(seesaw_handle,
                                    seesaw_handle->neopixel_handle->type);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update type");
    return ret;
  }

  ret = seesaw_neopixel_update_length(seesaw_handle,
                                      seesaw_handle->neopixel_handle->numLEDs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update length");
    return ret;
  }

  ret = seesaw_neopixel_set_pin(seesaw_handle,
                                seesaw_handle->neopixel_handle->pin);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set pin");
    return ret;
  } else {
    return ESP_OK;
  }
}

/**
 * @brief Updates the type of the neopixel connected to the seesaw device.
 *
 * This function updates the neopixel type based on the provided value.
 * The neopixel type determines the color order and data format of the neopixel.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param t The value representing the neopixel type.
 *
 * @return `ESP_OK` if the neopixel type is updated successfully, otherwise an
 * error code.
 */
esp_err_t seesaw_neopixel_update_type(const seesaw_handle_t seesaw_handle,
                                      uint16_t t) {
  bool oldTreeBytesPerPixel =
      (seesaw_handle->neopixel_handle->wOffset ==
       seesaw_handle->neopixel_handle->rOffset); // false if RGBW

  seesaw_handle->neopixel_handle->wOffset =
      (t >> 6) & 0b11; // See notes in header file
  seesaw_handle->neopixel_handle->rOffset =
      (t >> 4) & 0b11; // regarding R/G/B/W offsets
  seesaw_handle->neopixel_handle->gOffset = (t >> 2) & 0b11;
  seesaw_handle->neopixel_handle->bOffset = t & 0b11;
  seesaw_handle->neopixel_handle->is800KHz = (t < 256); // 400 KHz flag is 1<<8

  // If bytes-per-pixel has changed (and pixel data was previously
  // allocated), re-allocate to new size.  Will clear any data.
  if (seesaw_handle->neopixel_handle->pixels) {
    bool newThreeBytesPerPixel = (seesaw_handle->neopixel_handle->wOffset ==
                                  seesaw_handle->neopixel_handle->rOffset);
    if (newThreeBytesPerPixel != oldTreeBytesPerPixel)
      seesaw_neopixel_update_length(seesaw_handle,
                                    seesaw_handle->neopixel_handle->numLEDs);
  }

  return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE,
                       SEESAW_NEOPIXEL_SPEED,
                       seesaw_handle->neopixel_handle->is800KHz);
}

/**
 * @brief Updates the length of the NeoPixel strip.
 *
 * This function updates the length of the NeoPixel strip by allocating new
 * memory for the pixels. If there is existing data, it will be freed before
 * allocating new memory. All pixels are cleared after allocating new memory.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param n The new length of the NeoPixel strip.
 * @return esp_err_t Returns ESP_OK if the length update is successful,
 * otherwise an error code.
 */
esp_err_t seesaw_neopixel_update_length(const seesaw_handle_t seesaw_handle,
                                        uint16_t n) {
  if (seesaw_handle->neopixel_handle->pixels)
    free(seesaw_handle->neopixel_handle->pixels); // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  seesaw_handle->neopixel_handle->numBytes =
      n * ((seesaw_handle->neopixel_handle->wOffset ==
            seesaw_handle->neopixel_handle->rOffset)
               ? 3
               : 4);
  if ((seesaw_handle->neopixel_handle->pixels =
           (uint8_t *)malloc(seesaw_handle->neopixel_handle->numBytes))) {
    memset(seesaw_handle->neopixel_handle->pixels, 0,
           seesaw_handle->neopixel_handle->numBytes);
    seesaw_handle->neopixel_handle->numLEDs = n;
  } else {
    seesaw_handle->neopixel_handle->numLEDs =
        seesaw_handle->neopixel_handle->numBytes = 0;
  }

  uint8_t buf[] = {(uint8_t)(seesaw_handle->neopixel_handle->numBytes >> 8),
                   (uint8_t)(seesaw_handle->neopixel_handle->numBytes & 0xFF)};
  return seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE,
                      SEESAW_NEOPIXEL_BUF_LENGTH, buf, 2);
}

/**
 * @brief Sets the pin for the NeoPixel on the seesaw device.
 *
 * This function sets the pin for the NeoPixel on the seesaw device specified by
 * `seesaw_handle`.
 *
 * @param[in] seesaw_handle The handle to the seesaw device.
 * @param[in] p The pin number to set for the NeoPixel.
 *
 * @return
 *     - ESP_OK if the pin is set successfully.
 *     - Error code if the pin setting fails.
 */
esp_err_t seesaw_neopixel_set_pin(const seesaw_handle_t seesaw_handle,
                                  uint8_t p) {
  esp_err_t ret = ESP_OK;

  ret = seesaw_write8(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE,
                      SEESAW_NEOPIXEL_PIN, p);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set pin");
    return ret;
  } else {
    seesaw_handle->neopixel_handle->pin = p;
    return ESP_OK;
  }
}

/**
 * @brief Checks if the neopixel can be shown.
 *
 * This function checks if the neopixel can be shown based on the time elapsed
 * since the last show.
 *
 * @param seesaw_handle The handle to the seesaw component.
 * @return `true` if the neopixel can be shown, `false` otherwise.
 */
bool seesaw_neopixel_can_show(const seesaw_handle_t seesaw_handle) {
  return (esp_timer_get_time() - seesaw_handle->neopixel_handle->endTime) >=
         300L;
}

/**
 * @brief Shows the neopixel data on the seesaw device.
 *
 * This function sends the neopixel data stored in the seesaw handle to the
 * seesaw device for display. If there is no pixel data to send, the function
 * returns without performing any action. The function waits until the seesaw
 * device is ready to receive the data before sending it. After sending the
 * data, the function saves the end time for the latch on the next call.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return `ESP_OK` if the neopixel data is successfully sent, or an error code
 * if an error occurred.
 */
esp_err_t seesaw_neopixel_show(const seesaw_handle_t seesaw_handle) {
  esp_err_t ret = ESP_OK;

  if (!seesaw_handle->neopixel_handle->pixels)
    return ret; // No pixel data to send

  // Data latch = 300+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while (!seesaw_neopixel_can_show(seesaw_handle))
    ;

  ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE,
                     SEESAW_NEOPIXEL_SHOW, NULL, 0);

  seesaw_handle->neopixel_handle->endTime =
      esp_timer_get_time(); // Save EOD time for latch on next call

  return ret;
}

/**
 * @brief Sets the brightness of the NeoPixel on the seesaw device.
 *
 * This function sets the brightness of the NeoPixel on the seesaw device
 * specified by the given seesaw handle.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param b The brightness value to set (0-255).
 */
void seesaw_neopixel_set_brightness(const seesaw_handle_t seesaw_handle,
                                    uint8_t b) {
  seesaw_handle->neopixel_handle->brightness = b;
}

/**
 * @brief Creates a 32-bit color value from the given RGB components.
 *
 * This function takes the red, green, and blue components as input and combines
 * them into a single 32-bit color value.
 *
 * @param r The red component of the color (0-255).
 * @param g The green component of the color (0-255).
 * @param b The blue component of the color (0-255).
 *
 * @return The 32-bit color value created from the RGB components.
 */
uint32_t seesaw_neopixel_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * @brief Returns the number of pixels in the neopixel strip connected to the
 * seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return The number of pixels in the neopixel strip.
 */
uint16_t seesaw_neopixel_num_pixels(const seesaw_handle_t seesaw_handle) {
  return seesaw_handle->neopixel_handle->numLEDs;
}

/**
 * @brief Sets the color of a specific pixel in the NeoPixel strip connected to
 * the seesaw device.
 *
 * This function sets the color of the specified pixel in the NeoPixel strip
 * connected to the seesaw device. The pixel index `n` should be within the
 * range of available pixels on the strip. The color `c` should be a 32-bit RGB
 * color value, where the most significant byte represents the red component,
 * the middle byte represents the green component, and the least significant
 * byte represents the blue component. If the brightness of the NeoPixel strip
 * is non-zero, the color values will be scaled accordingly. The function
 * calculates the appropriate offset based on the configuration of the NeoPixel
 * strip and sets the corresponding color values in the pixel buffer. Finally,
 * it writes the updated pixel buffer to the seesaw device via I2C
 * communication.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @param n The index of the pixel to set the color for.
 * @param c The 32-bit RGB color value to set for the pixel.
 * @return `ESP_OK` if the operation is successful, otherwise an error code
 * indicating the cause of failure.
 */
esp_err_t seesaw_neopixel_set_pixel_color(const seesaw_handle_t seesaw_handle,
                                          uint16_t n, uint32_t c) {
  esp_err_t ret = ESP_OK;

  if (n < seesaw_neopixel_num_pixels(seesaw_handle)) {
    uint8_t *p, r = (uint8_t)(c >> 16), g = (uint8_t)(c >> 8), b = (uint8_t)c;

    if (seesaw_handle->neopixel_handle->brightness) {
      r = (r * seesaw_handle->neopixel_handle->brightness) >> 8;
      g = (g * seesaw_handle->neopixel_handle->brightness) >> 8;
      b = (b * seesaw_handle->neopixel_handle->brightness) >> 8;
    }

    if (seesaw_handle->neopixel_handle->wOffset ==
        seesaw_handle->neopixel_handle->rOffset) {
      p = &seesaw_handle->neopixel_handle->pixels[n * 3];
    } else {
      p = &seesaw_handle->neopixel_handle->pixels[n * 4];
      uint8_t w = (uint8_t)(c >> 24);
      p[seesaw_handle->neopixel_handle->wOffset] =
          seesaw_handle->neopixel_handle->brightness
              ? ((w * seesaw_handle->neopixel_handle->brightness) >> 8)
              : w;
    }

    p[seesaw_handle->neopixel_handle->rOffset] = r;
    p[seesaw_handle->neopixel_handle->gOffset] = g;
    p[seesaw_handle->neopixel_handle->bOffset] = b;

    uint8_t len = (seesaw_handle->neopixel_handle->wOffset ==
                           seesaw_handle->neopixel_handle->rOffset
                       ? 3
                       : 4);
    uint16_t offset = n * len;

    uint8_t writeBuf[6];
    writeBuf[0] = (offset >> 8);
    writeBuf[1] = offset;
    memcpy(&writeBuf[2], p, len);

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE,
                       SEESAW_NEOPIXEL_BUF, writeBuf, len + 2);
  }

  return ret;
}

uint32_t seesaw_neopixel_get_pixel_color(const seesaw_handle_t seesaw_handle,
                                         uint16_t n) {
  if (n >= seesaw_neopixel_num_pixels(seesaw_handle)) {
    return 0;
  }

  uint8_t *p;

  if (seesaw_handle->neopixel_handle->wOffset ==
      seesaw_handle->neopixel_handle->rOffset) // Is RGB-type device
  {
    p = &seesaw_handle->neopixel_handle->pixels[n * 3];

    if (seesaw_handle->neopixel_handle->brightness) {
      // Stored color was decimated by setBrightness().  Returned value
      // attempts to scale back to an approximation of the original 24-bit
      // value used when setting the pixel color, but there will always be
      // some error -- those bits are simply gone.  Issue is most
      // pronounced at low brightness levels.
      return (((uint32_t)(p[seesaw_handle->neopixel_handle->rOffset] << 8) /
               seesaw_handle->neopixel_handle->brightness)
              << 16) |
             (((uint32_t)(p[seesaw_handle->neopixel_handle->gOffset] << 8) /
               seesaw_handle->neopixel_handle->brightness)
              << 8) |
             ((uint32_t)(p[seesaw_handle->neopixel_handle->bOffset] << 8) /
              seesaw_handle->neopixel_handle->brightness);
    } else {
      // No brightness adjustment has been made -- return 'raw' color
      return ((uint32_t)p[seesaw_handle->neopixel_handle->rOffset] << 16) |
             ((uint32_t)p[seesaw_handle->neopixel_handle->gOffset] << 8) |
             (uint32_t)p[seesaw_handle->neopixel_handle->bOffset];
    }
  } else // Is RGBW-type device
  {
    p = &seesaw_handle->neopixel_handle->pixels[n * 4];

    if (seesaw_handle->neopixel_handle->brightness) {
      // Return scaled color
      return (((uint32_t)(p[seesaw_handle->neopixel_handle->wOffset] << 8) /
               seesaw_handle->neopixel_handle->brightness)
              << 24) |
             (((uint32_t)(p[seesaw_handle->neopixel_handle->rOffset] << 8) /
               seesaw_handle->neopixel_handle->brightness)
              << 16) |
             (((uint32_t)(p[seesaw_handle->neopixel_handle->gOffset] << 8) /
               seesaw_handle->neopixel_handle->brightness)
              << 8) |
             ((uint32_t)(p[seesaw_handle->neopixel_handle->bOffset] << 8) /
              seesaw_handle->neopixel_handle->brightness);
    } else {
      // Return raw color
      return ((uint32_t)p[seesaw_handle->neopixel_handle->wOffset] << 24) |
             ((uint32_t)p[seesaw_handle->neopixel_handle->rOffset] << 16) |
             ((uint32_t)p[seesaw_handle->neopixel_handle->gOffset] << 8) |
             (uint32_t)p[seesaw_handle->neopixel_handle->bOffset];
    }
  }
}