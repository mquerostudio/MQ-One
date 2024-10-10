#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// TFT Board Pins
#define BOARD_LCD_MOSI (39)
#define BOARD_LCD_SCK (38)
#define BOARD_LCD_CS (40)
#define BOARD_LCD_RST (41)
#define BOARD_LCD_DC (42)
#define BOARD_LCD_BL (-1)
#define BOARD_DISP_TE (-1)

#define BOARD_TOUCH_IRQ (-1)
// #define BOARD_TOUCH_RST      (41)
#define BOARD_TOUCH_RST (-1)

#define DISPLAY_BUFFER_SIZE (TFT_WIDTH * TFT_HEIGHT)
#define DISPLAY_FULLRESH true

// Main I2C Bus
#define BOARD_I2C_SDA (47)
#define BOARD_I2C_SCL (21)

// Module I2C Bus
#define MOD_I2C_SDA (9)
#define MOD_I2C_SCL (10)

/* Encoder */
#define PIN_ENCODER_SW (48)

/* EEPROM */
#define PIN_EEPROM (45)

// // MicroSD Slot
// #define PIN_SDCARD_CLK      43
// #define PIN_SDCARD_MISO     44
// #define PIN_SDCARD_MOSI     2
// #define PIN_SDCARD_CS       1
// #define SAMD21_SDCARD_DET   9

// // USB-C
// #define PIN_USB_DPLUS       20
// #define PIN_USB_DMINUS      19

// // Power Control
// #define PIN_POWER_CTRL      45

// // I2S Audio
// #define PIN_I2S_LRCLK       14
// #define PIN_I2S_BLCK        12
// #define PIN_I2S_MIC         13
// #define PIN_I2S_SPEAKER     11

// // Battery Management
// #define SAM21_BATM_INT      10
// #define SAM21_BATM_STAT1    4
// #define SAM21_BATM_STAT2    5

// // IMU
// #define SAM21_IMU_INT       10
// #define SAM21_IMU_RST       18

// //NeoPixel
#define SAM21_PIN_NEOPIXEL 20

// // Encoder
// #define SAM21_ENC_A         6
// #define SAM21_ENC_B         7

// // External Module
#define MOD_PIN_0           4
#define MOD_PIN_1           5
#define MOD_PIN_2           6
#define MOD_PIN_3           7
#define MOD_PIN_4           15
#define MOD_PIN_5           16
#define MOD_PIN_6           17
#define MOD_PIN_7           18
#define MOD_PIN_8           3
#define MOD_PIN_9           46
#define MOD_PIN_10          8
// #define SAM21_MOD_11        14
// #define SAM21_MOD_12        13
// #define SAM21_MOD_13        8
// #define SAM21_MOD_14        2
// #define SAM21_MOD_15        3
// #define SAM21_MOD_RESET     15
// #define SAM21_MOD_PWDN      16
// #define SAM21_MOD_DET       9
// #define SAM21_MOD_EEPROM    17

#endif // PIN_CONFIG_H