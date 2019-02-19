#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <lvgl/lvgl.h>
#include <drv/disp_spi.h>
#include <drv/ili9341.h>
#include <drv/m5stack_kpad.h>
#include <esp_freertos_hooks.h>
#include "mcp9600.h"
#include "pid.h"
#include "../cmake-build-debug/config/sdkconfig.h"

#define BUFFER_SIZE 128

// GPIO Settings
#define TOP_HEATER_PIN 16
#define BOTTOM_HEATER_PIN 17

// I2C Settings
#define I2C_MASTER_SCL 22
#define I2C_MASTER_SDA 21
#define I2C_MASTER_NUM 1
#define I2C_MASTER_FREQ_HZ 50000
#define I2C_MASTER_SCL_IO I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO I2C_MASTER_SDA
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// Global variables
static xQueueHandle event_queue = NULL;
static pid temp_pid;
static float setpoint = 100.0f;
static float current_temp = 0.0f;
static lv_group_t * g;

typedef enum {
  TEMPERATURE_UPDATE,
  DUTY_CYCLE_UPDATE,
  HEATER_UPDATE
} event_type_t;

typedef struct {
  event_type_t type;
  void* value;
} event_t;

static void IRAM_ATTR lv_tick_task(void) {
  lv_tick_inc(portTICK_RATE_MS);
}

static esp_err_t i2c_setup()
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void gpio_setup() {
  gpio_config_t heater_conf = {
      .intr_type = GPIO_PIN_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT_OUTPUT,
      .pin_bit_mask = (1ULL<<TOP_HEATER_PIN) | (1ULL<<BOTTOM_HEATER_PIN),
      .pull_down_en = 0,
      .pull_up_en = 0
  };
  gpio_config(&heater_conf);
}

void lvgl_setup() {
  lv_disp_drv_t disp;
  lv_disp_drv_init(&disp);
  disp.disp_flush = ili9431_flush;
  disp.disp_fill = ili9431_fill;
  lv_disp_drv_register(&disp);

  lv_indev_drv_t indev;
  lv_indev_drv_init(&indev);
  indev.read = m5stack_enc_read;
  indev.type = LV_INDEV_TYPE_ENCODER;
  lv_indev_t * m5stack_kpad_indev = lv_indev_drv_register(&indev);

  g = lv_group_create();
  lv_indev_set_group(m5stack_kpad_indev, g);

  esp_register_freertos_tick_hook(lv_tick_task);
}

static void temperature_task(void *arg) {
  // Configure thermocouple
  Mcp9600 temp_sensor = {
      .address = 0x60,
      .master_port = I2C_MASTER_NUM,
      .thermocouple_type = THER_TYPE_K,
      .filter_coefficents = FILT_MID,
      .shutdown_modes = NORMAL_OPERATION,
      .burst_mode_samples = BURST_32_SAMPLE,
      .adc_resolution = ADC_14BIT_RESOLUTION,
      .cold_junction_resolution = COLD_JUNC_RESOLUTION_0_25
  };
  Mcp_configure(&temp_sensor);

  event_t temp_event = {
      .type = TEMPERATURE_UPDATE,
      .value = NULL
  };
  float * temp_val = (float *) &temp_event.value;

  while(1) {
    if(ESP_OK == Mcp_get_hot_junc(&temp_sensor, temp_val)) {
      current_temp = *temp_val;

      xQueueSend(event_queue, &temp_event, 0);
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

static void control_task(void *arg) {
  event_t cmd_event = {
      .type = DUTY_CYCLE_UPDATE,
      .value = NULL
  };
  float* command = (float *) &cmd_event.value;

  event_t heat_event = {
      .type = HEATER_UPDATE,
      .value = NULL
  };
  uint32_t* heat_val = (uint32_t *) &heat_event.value;

  TickType_t now_time = xTaskGetTickCount();
  TickType_t last_pid_time = now_time;

  float duty_cycle = 0.0f;
  const float period = 1000.0f;
  float on_time, off_time;
  float error = 0.0f;
  while(1) {
    now_time = xTaskGetTickCount();

    // Calculate PID output
    error = setpoint - current_temp;

    *command = pid_compute_command(&temp_pid, error, (now_time - last_pid_time) * portTICK_RATE_MS);
    *command = clamp(*command, 0.0f, 100.0f);

    xQueueSend(event_queue, &cmd_event, 0);

    last_pid_time = now_time;

    // Perform PWM duty cycle on output
    duty_cycle = *command / 100.0f;
    on_time = duty_cycle * period;
    off_time = period - on_time;

    // Turn on heaters
    if(duty_cycle > 0.05f) {
      *heat_val = 1;
      xQueueSend(event_queue, &heat_event, 0);
      gpio_set_level(TOP_HEATER_PIN, 1);
      gpio_set_level(BOTTOM_HEATER_PIN, 1);
      vTaskDelay((TickType_t) on_time / portTICK_RATE_MS);
    }

    // Turn off heaters
    if(duty_cycle < 0.95f) {
      *heat_val = 0;
      xQueueSend(event_queue, &heat_event, 0);
      gpio_set_level(TOP_HEATER_PIN, 0);
      gpio_set_level(BOTTOM_HEATER_PIN, 0);
      vTaskDelay((TickType_t) off_time / portTICK_RATE_MS);
    }
  }
}

static lv_obj_t * pid_gain_sbox(lv_coord_t x, lv_coord_t y) {
  lv_obj_t *sbox = lv_spinbox_create(lv_scr_act(), NULL);
  lv_spinbox_set_digit_format(sbox, 3, 2);
  lv_spinbox_set_range(sbox, 0, 1000);
  lv_obj_set_size(sbox, 60, 30);
  lv_obj_align(sbox, NULL, LV_ALIGN_IN_TOP_LEFT, 4, 0);
  lv_obj_set_pos(sbox, x, y);
  lv_group_add_obj(g, sbox);
  return sbox;
}

typedef struct {
  lv_obj_t *p_gain_sbox;
  lv_obj_t *i_gain_sbox;
  lv_obj_t *d_gain_sbox;
  lv_obj_t *temp_lbl;
} lv_gui_t;

static void gui_create(lv_gui_t* gui) {
  gui->p_gain_sbox = pid_gain_sbox(20, 100);
  lv_spinbox_set_value(gui->p_gain_sbox, (int32_t) temp_pid.p_gain_ * 10);

  gui->i_gain_sbox = pid_gain_sbox(20, 140);
  lv_spinbox_set_value(gui->i_gain_sbox, (int32_t) temp_pid.i_gain_ * 10);

  gui->d_gain_sbox = pid_gain_sbox(20, 180);
  lv_spinbox_set_value(gui->d_gain_sbox, (int32_t) temp_pid.d_gain_ * 10);

  gui->temp_lbl = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_pos(gui->temp_lbl, 20, 40);
}

static void gui_update_task(void* arg) {
  lv_gui_t* gui = (lv_gui_t*) arg;

  char tmp_buff[BUFFER_SIZE];
  event_t received_event;

  while (1) {
    // Redraw GUI objects when there is an update from the event queue
    if(xQueueReceive(event_queue, &received_event, portMAX_DELAY)) {
      if(TEMPERATURE_UPDATE == received_event.type) {
        const float temperature = *(float *)&received_event.value;
        snprintf(tmp_buff, BUFFER_SIZE, "Temp. = %.01f C\n", temperature);
        lv_label_set_text(gui->temp_lbl, tmp_buff);
      }
      else if(DUTY_CYCLE_UPDATE == received_event.type) {
        const float command = *(float *)&received_event.value;
        snprintf(tmp_buff, BUFFER_SIZE, "Duty Cycle = \r%.01f %% \n", command);
      }
      else if(HEATER_UPDATE == received_event.type) {
        const uint32_t command = *(uint32_t *)&received_event.value;
      }
    }
  }
}

void app_main()
{
  event_queue = xQueueCreate(16, sizeof(event_t) + 4);
  pid_set_gains(&temp_pid, 2.0f, 0.0f, 1.0f, 0.0f, 0.0f, false);

  gpio_setup();
  i2c_setup();

  lv_init();

  disp_spi_init();
  ili9431_init();

  m5stack_kpad_init();

  lvgl_setup();

  lv_gui_t gui;
  gui_create(&gui);

  xTaskCreate(gui_update_task, "gui_update_task", 1024 * 2, (void *) &gui, 5, NULL);
  xTaskCreate(temperature_task, "temperature_task", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(control_task, "control_task", 1024 * 2, NULL, 7, NULL);

  while (1) {
    vTaskDelay(1);
    lv_task_handler();
  }
}