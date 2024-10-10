/**
 * @file main.c
 * @brief Main file for the project
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "driver/gpio.h"

#include "bno085.h"
#include "i2c_driver.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "modules_driver.h"
#include "seesaw.h"
#include "ui/ui.h"

#include "app_config.h"
#include "board_pins.h"

/*********************
 *     DEFINES
 * *******************/

/*********************
 *    TYPEDEFS
 * *******************/
/**
 * @brief MQONE Handle structure
 */
typedef struct {
  bool is_power_on_locked;
  bool is_i2c_board_init;
  bool is_i2c_module_init;
  bool is_seesaw_init;
  bool is_one_wire_init;
  bool is_module_connected;
  bool is_camera_connected;
  bool is_mountain_connected;
  bool is_dev_connected;
} mqone_handle_t;

/* MQ one handle */
mqone_handle_t mqone_handle;

/* Semaphore for GUI */
SemaphoreHandle_t xguiSemaphore;

/*********************
 * STATIC PROTOTYPES
 * *******************/
static uint32_t lv_tick_task(void);
static void task_gui(void *arg);
static void task_enc_button(void *arg);
static void task_check_system(void *arg);
static void task_neopixel(void *arg);
static void task_check_module_connect(void *arg);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "main";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void app_main() {
  ESP_LOGI(TAG, "------ Initialize Main I2C.");
  ESP_ERROR_CHECK(i2c_board_init());
  mqone_handle.is_i2c_board_init = true;
  ESP_LOGI(TAG, "------ Main I2C Initialized.");

  ESP_LOGI(TAG, "------ Initialize Seesaw.");
  i2c_device_config_t i2c_dev_conf = {.scl_speed_hz = (400 * 1000),
                                      .device_address = SEESAW_I2C_ADDR};
  ESP_ERROR_CHECK(
      seesaw_init(board_i2c_bus_handle, &i2c_dev_conf, &seesaw_handle));
  mqone_handle.is_seesaw_init = true;
  ESP_ERROR_CHECK(seesaw_SW_reset(seesaw_handle));
  ESP_LOGI(TAG, "------ Seesaw Initialized.");

  ESP_LOGI(TAG, "------ Lock Power On");
  ESP_ERROR_CHECK(seesaw_turn_on(seesaw_handle));
  mqone_handle.is_power_on_locked = true;
  ESP_LOGI(TAG, "------ Lock Power Done");

  ESP_LOGI(TAG, "------ Initialize OneWire Bus");
  ESP_ERROR_CHECK(one_wire_init());
  mqone_handle.is_one_wire_init = true;

  ESP_LOGI(TAG, "------ Initialize LVGL");
  xguiSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(task_gui, "gui", LVGL_TASK_STACK_SIZE, NULL,
                          LVGL_TASK_PRIORITY, NULL, 0);

  xTaskCreatePinnedToCore(task_enc_button, "task_enc_button", (1024 * 2), NULL,
                          5, NULL, 1);
  /*  xTaskCreatePinnedToCore(task_check_system, "task_check_system", (1024 *
     2), NULL, 4, NULL, 1);*/
  xTaskCreatePinnedToCore(task_check_module_connect,
                          "task_check_module_connect", (1024 * 5), NULL, 4,
                          NULL, 1);

  while (1) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static uint32_t lv_tick_task(void) {
  return (((uint32_t)esp_timer_get_time()) / 1000);
}

static void task_gui(void *arg) {
  lv_init();
  lv_tick_set_cb((lv_tick_get_cb_t)lv_tick_task);
  lv_port_disp_init();
  lv_port_indev_init();

  ui_init();
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  uint32_t time_till_next = 0;
  while (1) {
    if (pdTRUE == xSemaphoreTake(xguiSemaphore, 50 / portTICK_PERIOD_MS)) {
      time_till_next = lv_timer_handler();
      xSemaphoreGive(xguiSemaphore);
    }
    vTaskDelay(time_till_next / portTICK_PERIOD_MS);
  }
}

static void task_enc_button(void *arg) {
  // Configure button input pin
  gpio_set_direction(PIN_ENCODER_SW, GPIO_MODE_INPUT);

  int64_t button_press_time = 0;

  while (1) {
    if (gpio_get_level(PIN_ENCODER_SW) == 0) // Button pressed
    {
      if (button_press_time == 0) {
        button_press_time = esp_timer_get_time(); // Record the press start time
      }

      if ((esp_timer_get_time() - button_press_time) >=
          PRESS_THRESHOLD) // 5 seconds in microseconds
      {
        ESP_LOGI(TAG, "------ Unlock power off");
        ESP_ERROR_CHECK(seesaw_turn_off(seesaw_handle));
        while (1)
          ;
      }
    } else {
      button_press_time = 0; // Reset press time if button is released
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Debouncing delay
  }
}

static void task_check_system(void *arg) {
  while (1) {
    ESP_LOGI(" ", " ");
    ESP_LOGI(TAG, "------ Check system status ------");
    ESP_LOGI(TAG, "------ Power On Locked: %s",
             mqone_handle.is_power_on_locked ? "Yes" : "No");
    ESP_LOGI(TAG, "------ I2C Board Initialized: %s",
             mqone_handle.is_i2c_board_init ? "Yes" : "No");
    ESP_LOGI(TAG, "------ I2C Module Initialized: %s",
             mqone_handle.is_i2c_module_init ? "Yes" : "No");
    ESP_LOGI(TAG, "------ Seesaw Initialized: %s",
             mqone_handle.is_seesaw_init ? "Yes" : "No");
    ESP_LOGI(TAG, "------ OneWire Initialized: %s",
             mqone_handle.is_one_wire_init ? "Yes" : "No");
    ESP_LOGI(TAG, "------ Module Connected: %s",
             mqone_handle.is_module_connected ? "Yes" : "No");
    ESP_LOGI(TAG, "-----------------------------------");
    ESP_LOGI(" ", " ");

    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

static void task_check_module_connect(void *arg) {
  while (1) {
    if (module_search() == ESP_OK) {
      if (!mqone_handle.is_module_connected) {
        module_init();
        mqone_handle.is_module_connected = true;
        switch (module_handle->id) {
        case MODULE_DEV_ROM_ID:
          ESP_LOGI(TAG, "------ Dev Module Inicialized");
          mqone_handle.is_dev_connected = true;
          mqone_handle.is_i2c_module_init = false;
          lv_obj_remove_flag(ui_Screen_Menu_Menubutton_MenuButton8,
                             LV_OBJ_FLAG_HIDDEN); /// Flags
          break;

        case MODULE_CAMERA_ROM_ID:
          ESP_LOGI(TAG, "------ Camera Module Inicialized");
          mqone_handle.is_camera_connected = true;
          mqone_handle.is_i2c_module_init = true;
          lv_obj_remove_flag(ui_Screen_Menu_Menubutton_MenuButton6,
                             LV_OBJ_FLAG_HIDDEN); /// Flags
          break;

        case MODULE_MOUNTAIN_ROM_ID:
          ESP_LOGI(TAG, "------ Mountain Module Inicialized");
          mqone_handle.is_mountain_connected = true;
          mqone_handle.is_i2c_module_init = true;
          lv_obj_remove_flag(ui_Screen_Menu_Menubutton_MenuButton7,
                             LV_OBJ_FLAG_HIDDEN); /// Flags
          break;
        }
      } else {
        ESP_LOGI(TAG, "Module already initialized");
      }
    } else {
      if (mqone_handle.is_module_connected) {
        ESP_LOGI(TAG, "Module disconnected");
        /*        _ui_screen_change(&ui_Screen_Menu, LV_SCR_LOAD_ANIM_FADE_ON,
           500, 0, &ui_Screen_Menu_screen_init);*/
        module_deinit();
        mqone_handle.is_module_connected = false;

        if (mqone_handle.is_camera_connected) {
          mqone_handle.is_camera_connected = false;
          mqone_handle.is_i2c_module_init = false;
          lv_obj_add_flag(ui_Screen_Menu_Menubutton_MenuButton6,
                          LV_OBJ_FLAG_HIDDEN); /// Flags
        } else if (mqone_handle.is_mountain_connected) {
          mqone_handle.is_mountain_connected = false;
          mqone_handle.is_i2c_module_init = false;
          lv_obj_add_flag(ui_Screen_Menu_Menubutton_MenuButton7,
                          LV_OBJ_FLAG_HIDDEN); /// Flags
        } else if (mqone_handle.is_dev_connected) {
          mqone_handle.is_dev_connected = false;
          lv_obj_add_flag(ui_Screen_Menu_Menubutton_MenuButton8,
                          LV_OBJ_FLAG_HIDDEN); /// Flags
        }
      } else {
        ESP_LOGI(TAG, "Module not found");
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}