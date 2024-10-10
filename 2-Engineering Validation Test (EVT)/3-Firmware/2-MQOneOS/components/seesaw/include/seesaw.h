/**
 * @file seesaw.h
 *
 */

#ifndef SEESAW_H
#define SEESAW_H

#include "driver/i2c_master.h"
#include "esp_err.h"

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

/** Module Base Addreses
 *  The module base addresses for different seesaw modules.
 */
enum {
  SEESAW_STATUS_BASE = 0x00,
  SEESAW_GPIO_BASE = 0x01,

  SEESAW_TIMER_BASE = 0x08,
  SEESAW_ADC_BASE = 0x09,
  SEESAW_DAC_BASE = 0x0A,
  SEESAW_INTERRUPT_BASE = 0x0B,
  SEESAW_EEPROM_BASE = 0x0D,
  SEESAW_NEOPIXEL_BASE = 0x0E,
  SEESAW_ENCODER_BASE = 0x11,
};

/** GPIO module function address registers
 */
enum {
  SEESAW_GPIO_DIRSET_BULK = 0x02,
  SEESAW_GPIO_DIRCLR_BULK = 0x03,
  SEESAW_GPIO_BULK = 0x04,
  SEESAW_GPIO_BULK_SET = 0x05,
  SEESAW_GPIO_BULK_CLR = 0x06,
  SEESAW_GPIO_BULK_TOGGLE = 0x07,
  SEESAW_GPIO_INTENSET = 0x08,
  SEESAW_GPIO_INTENCLR = 0x09,
  SEESAW_GPIO_INTFLAG = 0x0A,
  SEESAW_GPIO_PULLENSET = 0x0B,
  SEESAW_GPIO_PULLENCLR = 0x0C,
};

/** status module function address registers
 */
enum {
  SEESAW_STATUS_HW_ID = 0x01,
  SEESAW_STATUS_VERSION = 0x02,
  SEESAW_STATUS_OPTIONS = 0x03,
  SEESAW_STATUS_TEMP = 0x04,
  SEESAW_STATUS_SWRST = 0x7F,
};

/** timer module function address registers
 */
enum {
  SEESAW_TIMER_STATUS = 0x00,
  SEESAW_TIMER_PWM = 0x01,
  SEESAW_TIMER_FREQ = 0x02,
};

/** ADC module function address registers
 */
enum {
  SEESAW_ADC_STATUS = 0x00,
  SEESAW_ADC_INTEN = 0x02,
  SEESAW_ADC_INTENCLR = 0x03,
  SEESAW_ADC_WINMODE = 0x04,
  SEESAW_ADC_WINTHRESH = 0x05,
  SEESAW_ADC_CHANNEL_OFFSET = 0x07,
};

/** neopixel module function address registers
 */
enum {
  SEESAW_NEOPIXEL_STATUS = 0x00,
  SEESAW_NEOPIXEL_PIN = 0x01,
  SEESAW_NEOPIXEL_SPEED = 0x02,
  SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
  SEESAW_NEOPIXEL_BUF = 0x04,
  SEESAW_NEOPIXEL_SHOW = 0x05,
};

/** encoder module edge definitions
 */
enum {
  SEESAW_ENCODER_STATUS = 0x00,
  SEESAW_ENCODER_INTENSET = 0x10,
  SEESAW_ENCODER_INTENCLR = 0x20,
  SEESAW_ENCODER_POSITION = 0x30,
  SEESAW_ENCODER_DELTA = 0x40,
};

enum {
  INPUT = 0x00,
  OUTPUT = 0x01,
  INPUT_PULLUP = 0x02,
  INPUT_PULLDOWN = 0x03,
};

#define ADC_INPUT_0_PIN 2 ///< default ADC input pin
#define ADC_INPUT_1_PIN 3 ///< default ADC input pin
#define ADC_INPUT_2_PIN 4 ///< default ADC input pin
#define ADC_INPUT_3_PIN 5 ///< default ADC input pin

#define PWM_0_PIN 4 ///< default PWM output pin
#define PWM_1_PIN 5 ///< default PWM output pin
#define PWM_2_PIN 6 ///< default PWM output pin
#define PWM_3_PIN 7 ///< default PWM output pin

// #ifndef INPUT_PULLDOWN
// #define INPUT_PULLDOWN
//   0x03 ///< for compatibility with platforms that do not already define
//        ///< INPUT_PULLDOWN
// #endif

// The order of primary colors in the NeoPixel data stream can vary
// among device types, manufacturers and even different revisions of
// the same item.  The third parameter to the seesaw_NeoPixel
// constructor encodes the per-pixel byte offsets of the red, green
// and blue primaries (plus white, if present) in the data stream --
// the following #defines provide an easier-to-use named version for
// each permutation.  e.g. NEO_GRB indicates a NeoPixel-compatible
// device expecting three bytes per pixel, with the first byte
// containing the green value, second containing red and third
// containing blue.  The in-memory representation of a chain of
// NeoPixels is the same as the data-stream order; no re-ordering of
// bytes is required when issuing data to the chain.

// Bits 5,4 of this value are the offset (0-3) from the first byte of
// a pixel to the location of the red color byte.  Bits 3,2 are the
// green offset and 1,0 are the blue offset.  If it is an RGBW-type
// device (supporting a white primary in addition to R,G,B), bits 7,6
// are the offset to the white byte...otherwise, bits 7,6 are set to
// the same value as 5,4 (red) to indicate an RGB (not RGBW) device.
// i.e. binary representation:
// 0bWWRRGGBB for RGBW devices
// 0bRRRRGGBB for RGB

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0))

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#define NEO_KHZ400 0x0100 // 400 KHz datastream

typedef struct {
  uint16_t numLEDs;
  int8_t pin;
  uint16_t type;
} neopixel_config_t;

struct neopixel_t {
  uint8_t brightness,
      *pixels, // Holds LED color values (3 or 4 bytes each)
      wOffset, rOffset, gOffset,
      bOffset; // Pixel offsets for RGB(W) order
  int8_t pin;
  uint16_t numLEDs, // Number of RGB LEDs in strip
      numBytes,     // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
      type;
  uint32_t endTime; // Latch timing reference
  bool is800KHz;    // 800 KHz datastream
};

/* handle of Neopixel device */
typedef struct neopixel_t *neopixel_handle_t;

struct seesaw_t {
  i2c_master_dev_handle_t i2c_dev;
  neopixel_handle_t neopixel_handle;
};

/* handle of Seesaw device */
typedef struct seesaw_t *seesaw_handle_t;

// Define the main seesaw handle
extern seesaw_handle_t seesaw_handle;

esp_err_t seesaw_init(const i2c_master_bus_handle_t bus_handle,
                      const i2c_device_config_t *seesaw_conf,
                      seesaw_handle_t *seesaw_handle);
esp_err_t seesaw_SW_reset(const seesaw_handle_t seesaw_handle);

esp_err_t seesaw_turn_on(const seesaw_handle_t seesaw_handle);
esp_err_t seesaw_turn_off(const seesaw_handle_t seesaw_handle);

esp_err_t seesaw_pin_mode(const seesaw_handle_t seesaw_handle, uint8_t pin,
                          uint8_t mode);
esp_err_t seesaw_pin_mode_bulk(const seesaw_handle_t seesaw_handle,
                               uint32_t pins, uint8_t mode);
esp_err_t seesaw_digital_write(const seesaw_handle_t seesaw_handle, uint8_t pin,
                               uint8_t value);
esp_err_t seesaw_digital_write_bulk(const seesaw_handle_t seesaw_handle,
                                    uint32_t pins, uint8_t value);
bool seesaw_digital_read(const seesaw_handle_t seesaw_handle, uint8_t pin);
uint32_t seesaw_digital_read_bulk(const seesaw_handle_t seesaw_handle,
                                  uint32_t pins);

int32_t seesaw_encoder_get_position(const seesaw_handle_t seesaw_handle);
int32_t seesaw_encoder_get_delta(const seesaw_handle_t seesaw_handle);
esp_err_t seesaw_encoder_set_position(const seesaw_handle_t seesaw_handle,
                                      int32_t pos);
esp_err_t seesaw_encoder_enable_interrupt(const seesaw_handle_t seesaw_handle);
esp_err_t seesaw_encoder_disable_interrupt(const seesaw_handle_t seesaw_handle);

esp_err_t seesaw_write8(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                        uint8_t regLow, uint8_t value);
esp_err_t seesaw_write(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                       uint8_t regLow, uint8_t *buf, uint8_t num);
esp_err_t seesaw_read(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh,
                      uint8_t regLow, uint8_t *buf, uint8_t num);

esp_err_t seesaw_neopixel_init(const seesaw_handle_t seesaw_handle,
                               const neopixel_config_t *neopixel_config);
esp_err_t seesaw_neopixel_update_type(const seesaw_handle_t seesaw_handle,
                                      uint16_t t);
esp_err_t seesaw_neopixel_update_length(const seesaw_handle_t seesaw_handle,
                                        uint16_t n);
esp_err_t seesaw_neopixel_set_pin(const seesaw_handle_t seesaw_handle,
                                  uint8_t p);
bool seesaw_neopixel_can_show(const seesaw_handle_t seesaw_handle);
esp_err_t seesaw_neopixel_show(const seesaw_handle_t seesaw_handle);
void seesaw_neopixel_set_brightness(const seesaw_handle_t seesaw_handle,
                                    uint8_t b);
uint32_t seesaw_neopixel_color(uint8_t r, uint8_t g, uint8_t b);
uint16_t seesaw_neopixel_num_pixels(const seesaw_handle_t seesaw_handle);
esp_err_t seesaw_neopixel_set_pixel_color(const seesaw_handle_t seesaw_handle,
                                          uint16_t n, uint32_t c);
uint32_t seesaw_neopixel_get_pixel_color(const seesaw_handle_t seesaw_handle,
                                         uint16_t n);

#endif /*SEESAW_H*/