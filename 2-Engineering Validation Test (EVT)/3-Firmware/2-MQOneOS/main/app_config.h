#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Define the I2C address */
#define SEESAW_I2C_ADDR (0x49)  ///< Default Seesaw I2C address
#define BNO085_I2C_ADDR (0x4A)  ///< Default BNO085 I2C address
#define PA1010D_I2C_ADDR (0x10) ///< Default PA1010D I2C address
#define BME280_I2C_ADDR (0x77)  ///< Default BME280 I2C address 0x77

/* Define the LCD configuration */
#define CONFIG_LCD_H_RES 240
#define CONFIG_LCD_V_RES 320
#define CONFIG_LCD_HOST SPI3_HOST
#define CONFIG_LCD_FREQ (80 * 1000 * 1000)
#define CONFIF_LCD_CMD_BITS 8
#define CONFIG_LCD_PARAM_BITS 8
#define DISP_BUF_SIZE (CONFIG_LCD_H_RES * CONFIG_LCD_V_RES)
#define SPI_BUS_MAX_TRANSFER_SZ (DISP_BUF_SIZE * 2)

/* Define LVGL configuration */
#define LVGL_TASK_STACK_SIZE (1024 * 18)
#define LVGL_TASK_PRIORITY 5

/* Define the I2C port for the board */
#define CONFIG_BOARD_I2C_PORT I2C_NUM_0

/* Define the I2C port for the module */
#define CONFIG_MOD_I2C_PORT I2C_NUM_1

/* Define a threshold for the second of button presses */
#define PRESS_THRESHOLD 2000000

#endif // APP_CONFIG_H