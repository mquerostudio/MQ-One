// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 9.1.0
// Project name: MQOne

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "modules_driver.h"
#include "seesaw.h"
#include "ui.h"

void lv_shut_down_device(lv_event_t *e) {
  ESP_LOGI("GUI", "Shutting down device");
  seesaw_turn_off(seesaw_handle);
}

void lv_mountain_module_load_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ Mountain module screen loaded");
  // Create the task and store its handle
  xTaskCreatePinnedToCore(task_gps_module, "task_gps_module", (1024 * 4), NULL,
                          5, &task_gps_module_handle, 1);
  xTaskCreatePinnedToCore(task_ambient_sensor_module, "task_ambient_sensor",
                          (1024 * 4), NULL, 5,
                          &task_ambient_sensor_module_handle, 1);
}

void lv_mountain_module_unload_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ Mountain module screen unloaded");
  // Check if the task handle is valid
  if (task_gps_module_handle != NULL) {
    // Delete the task
    vTaskDelete(task_gps_module_handle);
    // Reset the task handle to NULL
    task_gps_module_handle = NULL;
  }
  if (task_ambient_sensor_module_handle != NULL) {
    // Delete the task
    vTaskDelete(task_ambient_sensor_module_handle);
    // Reset the task handle to NULL
    task_ambient_sensor_module_handle = NULL;
  }
}

void lv_imu_load_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ IMU screen loaded");
  // Create the task and store its handle
  xTaskCreatePinnedToCore(imu_task, "imu_task", (1024 * 4), NULL, 5,
                          &imu_task_handle, 1);
}

void lv_imu_unload_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ IMU screen unloaded");
  // Check if the task handle is valid
  if (imu_task_handle != NULL) {
    // Delete the task
    vTaskDelete(imu_task_handle);
    // Reset the task handle to NULL
    imu_task_handle = NULL;
  }
}

void lv_neopixel_load_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ Neopixel screen loaded");

  if (neopixel_task_handle == NULL) {
    // Create the task and store its handle
    xTaskCreatePinnedToCore(neopixel_task, "neopixel_task", (1024 * 10), NULL,
                            5, &neopixel_task_handle, 1);
  }
}

void lv_neopixel_unload_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ Neopixel screen unloaded");
  /*  // Check if the task handle is valid
    if (neopixel_task_handle != NULL) {
      // Delete the task
      vTaskDelete(neopixel_task_handle);
      // Reset the task handle to NULL
      neopixel_task_handle = NULL;
    }*/
}

void lv_microsd_load_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ microSD screen loaded");
  // Create the task and store its handle
  xTaskCreatePinnedToCore(microsd_task, "microsd_task", (1024 * 2), NULL, 5,
                          &microsd_task_handle, 1);
}

void lv_microsd_unload_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ microSD screen unloaded");
  // Check if the task handle is valid
  if (microsd_task_handle != NULL) {
    // Delete the task
    vTaskDelete(microsd_task_handle);
    // Reset the task handle to NULL
    microsd_task_handle = NULL;
  }
}

void lv_app_start_load_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ App Start screen loaded");
  // Create the task and store its handle
  xTaskCreatePinnedToCore(app_start_task, "app_start_task", (1024 * 5), NULL, 5,
                          &app_start_task_handle, 1);
}

void lv_app_start_unload_screen(lv_event_t *e) {
  ESP_LOGI("lv", "------ App Start screen unloaded");
  // Check if the task handle is valid
  if (app_start_task_handle != NULL) {
    // Delete the task
    vTaskDelete(app_start_task_handle);
    // Reset the task handle to NULL
    app_start_task_handle = NULL;
  }
}

void lv_camera_module_load_screen(lv_event_t * e)
{
	  ESP_LOGI("lv", "------ Camera module screen loaded");
  // Create the task and store its handle
  xTaskCreatePinnedToCore(task_camera_module, "task_camera_module", (1024 * 10), NULL,
                          5, &task_camera_module_handle, 1);
}

void lv_camera_module_unload_screen(lv_event_t * e)
{
	  ESP_LOGI("lv", "------ Camera module screen unloaded");
  // Check if the task handle is valid
  if (task_camera_module_handle != NULL) {
    // Delete the task
    vTaskDelete(task_camera_module_handle);
    // Reset the task handle to NULL
    task_camera_module_handle = NULL;
  }
}
