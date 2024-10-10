/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable
 * content*/
#include "esp_lcd_types.h"
#include <stdint.h>
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
esp_lcd_panel_handle_t panel_handle = NULL;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_display_t *disp, const lv_area_t *area,
                       uint8_t *px_map);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void) {
  /*-------------------------
   * Initialize your display
   * -----------------------*/
  disp_init();

  /*------------------------------------
   * Create a display and set a flush_cb
   * -----------------------------------*/
  lv_display_t *disp = lv_display_create(CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
  lv_display_set_flush_cb(disp, disp_flush);

  /*
   * Two buffers for partial rendering
   * In flush_cb DMA or similar hardware should be used to update the display in
   * the background.
   */
  static lv_color_t buf1[DISP_BUF_SIZE / 10];
  static lv_color_t buf2[DISP_BUF_SIZE / 10];
  lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void) {
  spi_bus_config_t buscfg = {
      .sclk_io_num = BOARD_LCD_SCK,
      .mosi_io_num = BOARD_LCD_MOSI,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = CONFIG_LCD_H_RES * 80 * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)CONFIG_LCD_HOST,
                                     &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = BOARD_LCD_DC,
      .cs_gpio_num = BOARD_LCD_CS,
      .pclk_hz = CONFIG_LCD_FREQ,
      .lcd_cmd_bits = CONFIF_LCD_CMD_BITS,
      .lcd_param_bits = CONFIG_LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
      (esp_lcd_spi_bus_handle_t)CONFIG_LCD_HOST, &io_config, &io_handle));

  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = BOARD_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = 16,
      .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
  esp_lcd_panel_swap_xy(panel_handle, false);
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called
 * by LVGL
 */
void disp_enable_update(void) { disp_flush_enabled = true; }

/* Disable updating the screen (the flushing process) when disp_flush() is
 * called by LVGL
 */
void disp_disable_update(void) { disp_flush_enabled = false; }

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied
 *to `area` on the display. You can use DMA or any hardware acceleration to do
 *this operation in the background but 'lv_display_flush_ready()' has to be
 *called when it's finished.*/
static void disp_flush(lv_display_t *disp_drv, const lv_area_t *area,
                       uint8_t *px_map) {
  if (disp_flush_enabled) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
                              offsety2 + 1, (void *)px_map);
  }

  /*IMPORTANT!!!
   *Inform the graphics library that you are ready with the flushing*/
  lv_display_flush_ready(disp_drv);
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif