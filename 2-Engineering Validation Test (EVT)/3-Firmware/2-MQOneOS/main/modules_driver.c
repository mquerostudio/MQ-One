/**
 * @file one_wire_driver.c
 * @brief One Wire Driver
 */

/*********************
 *      INCLUDES
 *********************/
#include "modules_driver.h"
#include "app_config.h"
#include "bme280_support.h"
#include "bno085.h"
#include "board_pins.h"
#include "core/lv_obj.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "gpgga.h"
#include "gprmc.h"
#include "i2c_driver.h"
#include "misc/lv_color.h"
#include "mqone_pa1010d.h"
#include "onewire_bus.h"
#include "seesaw.h"
#include "ui/ui.h"
#include "widgets/label/lv_label.h"
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
onewire_bus_handle_t one_wire_handle;
onewire_device_t onewire_device;
module_handle_t module_handle;

// Global variable to store the task handle
TaskHandle_t task_gps_module_handle;
TaskHandle_t task_camera_module_handle;
TaskHandle_t task_ambient_sensor_module_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t neopixel_task_handle;
TaskHandle_t microsd_task_handle;
TaskHandle_t app_start_task_handle;

/**********************
 *  STATIC PROTOTYPES
 **********************/
void swap_bytes(uint8_t *buf, size_t len);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "Modules Driver";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t one_wire_init() {
  esp_err_t ret = ESP_OK;

  onewire_bus_config_t bus_config = {
      .bus_gpio_num = PIN_EEPROM,
  };

  onewire_bus_rmt_config_t rmt_config = {
      .max_rx_bytes = 32, /* 32 bytes, which is the size of one page */
  };

  ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &one_wire_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install 1-Wire bus on GPIO%d",
             bus_config.bus_gpio_num);
    return ret;
  }

  ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", bus_config.bus_gpio_num);

  return ESP_OK;
}

esp_err_t module_search() {
  esp_err_t ret = ESP_OK;
  onewire_device_iter_handle_t iter = NULL;

  ESP_ERROR_CHECK(onewire_new_device_iter(one_wire_handle, &iter));
  ESP_LOGI(TAG, "Check if any module is connected to the device...");

  ret = onewire_device_iter_get_next(iter, &onewire_device);
  if (ret == ESP_OK) {
    for (uint16_t i = 0; i < sizeof(module_model) / sizeof(module_model[0]);
         i++) {
      if (onewire_device.address == module_model[i].id) {
        ESP_LOGI(TAG, "Module %s found with address: %016llX",
                 module_model[i].name, module_model[i].id);
        onewire_del_device_iter(iter);
        return ret;
      }
    }
    onewire_del_device_iter(iter);
    return ESP_ERR_NOT_SUPPORTED;
  }

  return ret;
}

esp_err_t module_init() {
  module_handle_t out_handle =
      (module_handle_t)calloc(1, sizeof(module_handle_t));
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_NO_MEM, TAG,
                      "Failed to allocate memory for module handle");

  ds28e07_handle_t eeprom_handle = NULL;

  ds28e07_init(onewire_device, &eeprom_handle);

  for (uint16_t i = 0; i < sizeof(module_model) / sizeof(module_model[0]);
       i++) {
    if (onewire_device.address == module_model[i].id) {
      out_handle->id = module_model[i].id;
      out_handle->name = module_model[i].name;
      out_handle->need_i2c = module_model[i].need_i2c;
      out_handle->eeprom_handle = eeprom_handle;
      break;
    }
  }

  module_handle = out_handle;
  if (module_handle == NULL) {
    ESP_LOGE(TAG, "Failed to initialize module handle");
    return ESP_FAIL;
  }

  if (module_handle->need_i2c) {
    ESP_LOGI(TAG, "Module requires I2C bus");
    if (i2c_module_init() != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize I2C module");
      return ESP_FAIL;
    }
  }

  return ESP_OK;
}

esp_err_t module_deinit() {
  if (module_handle->need_i2c) {
    // ret = i2c_module_rm_device(module_i2c_bus_handle);

    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to remove I2C device");
    //     return ret;
    // }

    if (i2c_module_deinit() == ESP_OK) {
      ESP_LOGI(TAG, "I2C module deinitialized");
    } else {
      ESP_LOGE(TAG, "Failed to deinitialize I2C module");
      return ESP_FAIL;
    }
  }

  module_handle = NULL;
  return ESP_OK;
}

/* Tasks for Mountain Module*/
void task_gps_module(void *param) {

  /*Inicialize and configure PA1010D GPS sensor */
  pa1010d_i2c_config_t pa1010d_i2c_conf = {
      .i2c_bus = module_i2c_bus_handle,
      .i2c_dev_conf.device_address = PA1010D_I2C_ADDR,
      .i2c_dev_conf.scl_speed_hz = 400000,
      .i2c_dev = NULL,
  };

  pa1010d_handle_t pa1010d_handle;

  ESP_LOGI(TAG, "------ Initialize PA1010D");
  ESP_ERROR_CHECK(pa1010d_init(&pa1010d_i2c_conf, &pa1010d_handle));
  ESP_LOGI(TAG, "------ PA1010D Initialized");

  while (true) {
    char buffer[50];

    nmea_gpgga_s *gpgga_data = NULL;
    nmea_gprmc_s *gprmc_data = NULL;

    while (gpgga_data == NULL) {
      gpgga_data =
          (nmea_gpgga_s *)pa1010d_read_parse_nmea(&pa1010d_handle, NMEA_GPGGA);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    while (gprmc_data == NULL) {
      gprmc_data =
          (nmea_gprmc_s *)pa1010d_read_parse_nmea(&pa1010d_handle, NMEA_GPRMC);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (gpgga_data != NULL && gprmc_data != NULL) {
      if (gpgga_data->n_satellites != 0) {

        sprintf(buffer, "%d %c", (int)gpgga_data->altitude,
                gpgga_data->altitude_unit);
        ESP_LOGI(TAG, "%s", buffer);
        lv_label_set_text(ui_Mountain_Module_Label_Label20, buffer);

        sprintf(buffer, "%d", gpgga_data->n_satellites);
        ESP_LOGI(TAG, "%s", buffer);
        lv_label_set_text(ui_Mountain_Module_Label_Label22, buffer);

        sprintf(buffer, "%d°%.1f'%c %d°%.1f'%c", gprmc_data->latitude.degrees,
                gprmc_data->latitude.minutes,
                (char)gprmc_data->latitude.cardinal,
                gprmc_data->longitude.degrees, gprmc_data->longitude.minutes,
                (char)gprmc_data->longitude.cardinal);

        ESP_LOGI(TAG, "%s", buffer);
        lv_label_set_text(ui_Mountain_Module_Label_Label18, buffer);

        free(gpgga_data);
        free(gprmc_data);
      } else if (strcmp(lv_label_get_text(ui_Mountain_Module_Label_Label22),
                        "0") != 0) {
        lv_label_set_text(ui_Mountain_Module_Label_Label22, "0");
        lv_label_set_text(ui_Mountain_Module_Label_Label20, "-");
        lv_label_set_text(ui_Mountain_Module_Label_Label18, "-");
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void task_ambient_sensor_module(void *param) {

  /* Inicialize and configure the BME280 sensor */
  bme280_i2c_config_t bme280_i2c_conf = {
      .i2c_bus = module_i2c_bus_handle,
      .i2c_dev_conf.device_address = BME280_I2C_ADDR,
      .i2c_dev_conf.scl_speed_hz = 400000,
      .i2c_dev = NULL,
  };

  bme280_handle_t bme280_handle;
  bme280_data_t bme280_data;

  ESP_LOGI(TAG, "------ Initialize BME280");
  ESP_ERROR_CHECK(bme280_init_driver(&bme280_i2c_conf, &bme280_handle));
  ESP_LOGI(TAG, "------ BME280 Initialized");

  //  Always read the current settings before writing, especially when all the
  // configuration is not modified
  ESP_ERROR_CHECK(bme280_get_sensor_settings_driver(&bme280_handle));

  /*Configuring the over - sampling rate, filter coefficient and standby time */
  bme280_settings_t bme280_settings = {
      .osr_p = BME280_OVERSAMPLING_1X,
      .osr_t = BME280_OVERSAMPLING_1X,
      .osr_h = BME280_OVERSAMPLING_1X,
      .filter = BME280_FILTER_COEFF_2,
      .standby_time = BME280_STANDBY_TIME_0_5_MS,
  };

  /*   Overwrite the desired settings */
  ESP_ERROR_CHECK(bme280_set_sensor_settings_driver(
      BME280_SEL_ALL_SETTINGS, &bme280_settings, &bme280_handle));

  /* Always set the power mode after setting the configuration */
  ESP_ERROR_CHECK(
      bme280_set_sensor_mode_driver(BME280_POWERMODE_NORMAL, &bme280_handle));

  /* Calculate measurement time in microseconds */
  ESP_ERROR_CHECK(bme280_cal_meas_delay_driver(&bme280_handle));

  while (true) {
    char buffer[32];

    bme280_get_data_sensor_driver(&bme280_data, &bme280_handle);

    if (bme280_data.pressure != 0) {

      sprintf(buffer, "%d *C", bme280_data.temperature);
      lv_label_set_text(ui_Mountain_Module_Label_Label10, buffer);

      sprintf(buffer, "%d %%", bme280_data.humidity);
      lv_label_set_text(ui_Mountain_Module_Label_Label14, buffer);

      sprintf(buffer, "%d Pa", (int)(bme280_data.pressure / 100));
      lv_label_set_text(ui_Mountain_Module_Label_Label15, buffer);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_camera_module(void *param) {
  static camera_config_t camera_config = {
      .pin_pwdn = -1,
      .pin_reset = -1,
      .pin_xclk = -1,
      .pin_sccb_sda = -1,
      .pin_sccb_scl = -1,
      .sccb_i2c_port = CONFIG_MOD_I2C_PORT,

      .pin_d7 = MOD_PIN_7,
      .pin_d6 = MOD_PIN_6,
      .pin_d5 = MOD_PIN_5,
      .pin_d4 = MOD_PIN_4,
      .pin_d3 = MOD_PIN_3,
      .pin_d2 = MOD_PIN_2,
      .pin_d1 = MOD_PIN_1,
      .pin_d0 = MOD_PIN_0,
      .pin_vsync = MOD_PIN_9,
      .pin_href = MOD_PIN_8,
      .pin_pclk = MOD_PIN_10,

      .xclk_freq_hz = 20000000,
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      .pixel_format = PIXFORMAT_RGB565,
      .frame_size = FRAMESIZE_QVGA,

      .jpeg_quality = 12,
      .fb_count = 2,
      .fb_location = CAMERA_FB_IN_PSRAM,
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
  };

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  lv_image_dsc_t my_png;
  lv_obj_t *img1 = lv_image_create(lv_screen_active());

  my_png.header.cf = LV_COLOR_FORMAT_RGB565;

  lv_obj_remove_style_all(img1);
  lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
  lv_image_set_rotation(img1, 1800);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  while (1) {

    camera_fb_t *pic = esp_camera_fb_get();

    if (pic->buf != NULL) {
      swap_bytes(pic->buf, pic->len);

      my_png.data = pic->buf;
      my_png.data_size = pic->len;
      my_png.header.w = pic->width;
      my_png.header.h = pic->height;

      lv_obj_set_size(img1, pic->width, pic->height);

      lv_image_set_src(img1, &my_png);
    }

    esp_camera_fb_return(pic);
  }
}

void imu_task(void *param) {
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void neopixel_task(void *param) {

  ESP_LOGI(TAG, "------ Setup Neopixel");
  neopixel_config_t neopixel_config = {
      .numLEDs = 24,
      .pin = SAM21_PIN_NEOPIXEL,
      .type = (NEO_GRB + NEO_KHZ800),
  };

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK(seesaw_neopixel_init(seesaw_handle, &neopixel_config));
  seesaw_neopixel_set_brightness(seesaw_handle, 5);
  ESP_ERROR_CHECK(seesaw_neopixel_show(seesaw_handle));

  int32_t n_pos = 0;
  bool dir_up = true;
  bool state = false;

  while (1) {

    if (lv_obj_has_state(ui_Screen_Neopixel_Switch_Switch1, LV_STATE_CHECKED)) {
      state = true;

      for (int i = 0; i < 24; i++) {
        seesaw_neopixel_set_pixel_color(seesaw_handle, i, 0);
      }

      if (n_pos >= 0) {
        int upperBound = MIN(n_pos, 24 - 1);
        for (int i = 0; i <= upperBound; i++) {
          seesaw_neopixel_set_pixel_color(seesaw_handle, i,
                                          seesaw_neopixel_color(0, 255, 0));
        }

        if (lv_obj_has_flag(ui_Screen_Neopixel_Arc_Arc3, LV_OBJ_FLAG_HIDDEN)) {
          lv_obj_remove_flag(ui_Screen_Neopixel_Arc_Arc3, LV_OBJ_FLAG_HIDDEN);
        }
        if (!lv_obj_has_flag(ui_Screen_Neopixel_Arc_Arc1, LV_OBJ_FLAG_HIDDEN)) {
          lv_obj_add_flag(ui_Screen_Neopixel_Arc_Arc1, LV_OBJ_FLAG_HIDDEN);
        }
        lv_arc_set_value(ui_Screen_Neopixel_Arc_Arc3, upperBound);
        ESP_LOGI(TAG, "Valor positivo puesto %d", upperBound);
      } else {
        int startLED = 24 + n_pos;
        for (int i = 24 - 1; i >= startLED; i--) {
          seesaw_neopixel_set_pixel_color(seesaw_handle, i,
                                          seesaw_neopixel_color(255, 0, 0));
        }
        if (lv_obj_has_flag(ui_Screen_Neopixel_Arc_Arc1, LV_OBJ_FLAG_HIDDEN)) {
          lv_obj_remove_flag(ui_Screen_Neopixel_Arc_Arc1, LV_OBJ_FLAG_HIDDEN);
        }
        if (!lv_obj_has_flag(ui_Screen_Neopixel_Arc_Arc3, LV_OBJ_FLAG_HIDDEN)) {
          lv_obj_add_flag(ui_Screen_Neopixel_Arc_Arc3, LV_OBJ_FLAG_HIDDEN);
        }
        lv_arc_set_value(ui_Screen_Neopixel_Arc_Arc1, (n_pos * (-1)));
        ESP_LOGI(TAG, "Valor negativo puesto %d", (n_pos * (-1)));
      }

      if (dir_up == true && n_pos == 23) {
        n_pos--;
        dir_up = false;
      } else if (dir_up == true) {
        n_pos++;
      } else if (dir_up == false && n_pos == -23) {
        n_pos++;
        dir_up = true;
      } else if (dir_up == false) {
        n_pos--;
      } else {
        ESP_LOGI(TAG, "eres tonto");
      }

      seesaw_neopixel_show(seesaw_handle);

    } else if (state) {
      n_pos = 0;
      dir_up = true;
      state = false;
      for (int i = 0; i < 24; i++) {
        seesaw_neopixel_set_pixel_color(seesaw_handle, i, 0);
      }
      lv_arc_set_value(ui_Screen_Neopixel_Arc_Arc1, 0);
      lv_arc_set_value(ui_Screen_Neopixel_Arc_Arc3, 0);
      seesaw_neopixel_show(seesaw_handle);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void microsd_task(void *param) {
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_start_task(void *param) {
  /*  ESP_LOGI(TAG, "------ Setup Neopixel");
    neopixel_config_t neopixel_config = {
        .numLEDs = 24,
        .pin = SAM21_PIN_NEOPIXEL,
        .type = (NEO_GRB + NEO_KHZ800),
    };

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(seesaw_neopixel_init(seesaw_handle, &neopixel_config));
    seesaw_neopixel_set_brightness(seesaw_handle, 10);
    ESP_ERROR_CHECK(seesaw_neopixel_show(seesaw_handle));

    ESP_LOGI(TAG, "------ Neopixel On Animation");
    for (uint8_t i = 0; i < 24; i++) {
      seesaw_neopixel_set_pixel_color(seesaw_handle, i, 0); // Turn off all
    pixels vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1) {
      for (uint16_t i = 0; i < 24; i++) {
        seesaw_neopixel_set_pixel_color(seesaw_handle, i,
                                        seesaw_neopixel_color(0, 0, 255));
        seesaw_neopixel_show(seesaw_handle);
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }

      for (uint8_t i = 0; i < 24; i++) {
        seesaw_neopixel_set_pixel_color(seesaw_handle, i,
                                        0); // Turn off all pixels
        seesaw_neopixel_show(seesaw_handle);
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
    }*/

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
// Function to swap the byte order (from big endian to little endian)
void swap_bytes(uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i += 2) {
    uint8_t temp = buf[i];
    buf[i] = buf[i + 1];
    buf[i + 1] = temp;
  }
}