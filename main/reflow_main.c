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
#include <lv_style.h>
#include "mcp9600.h"
#include "pid.h"
#include "lv_screen.h"
#include "temp_profile.h"
#include "../cmake-build-debug/config/sdkconfig.h"

#define MAX_TEMPERATURE 310.0f
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

typedef struct {
  lv_screen_t * tune_pid_scr;
  lv_obj_t *setpoint_sbox;
  lv_obj_t *p_gain_sbox;
  lv_obj_t *i_gain_sbox;
  lv_obj_t *d_gain_sbox;
  lv_obj_t *temp_lbl;
  lv_obj_t *power_lbl;
  lv_obj_t *heat_led;
  lv_obj_t *temp_chart;
  lv_obj_t *home1_btn;
  lv_chart_series_t * temp_curve;
  lv_chart_series_t * setpoint_curve;
  lv_screen_t * main_menu_scr;
  lv_obj_t * menu_lst;
  lv_obj_t * run_profile_btn;
  lv_obj_t * tune_pid_btn;
  lv_obj_t * settings_btn;

} lv_gui_t;

// Global variables
static xQueueHandle event_queue = NULL;
static pid temp_pid;
static float setpoint = 0.0f;
static float current_temp = 0.0f;
static lv_group_t * g;
static lv_gui_t gui;

typedef enum {
  TEMPERATURE_UPDATE,
  DUTY_CYCLE_UPDATE,
  HEATER_UPDATE
} event_type_t;

typedef struct {
  event_type_t type;
  union {
    float float_val;
    int32_t int_val;
  } value ;
} event_t;

static temp_profile_t * mg_4860p;

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

  event_t temp_event;
  temp_event.type = TEMPERATURE_UPDATE;

  while(1) {
    if(ESP_OK == Mcp_get_hot_junc(&temp_sensor, &temp_event.value.float_val)) {
      current_temp = temp_event.value.float_val;

      xQueueSend(event_queue, &temp_event, 0);
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

static void control_task(void *arg) {
  event_t cmd_event;
  cmd_event.type = DUTY_CYCLE_UPDATE;

  event_t heat_event;
  heat_event.type = HEATER_UPDATE;

  TickType_t now_time = xTaskGetTickCount();
  TickType_t last_pid_time = now_time;

  float duty_cycle = 0.0f;
  const float period = 500.0f;
  float on_time, off_time;
  float error = 0.0f;
  while(1) {
    now_time = xTaskGetTickCount();

    // Calculate PID output
    error = setpoint - current_temp;

    cmd_event.value.float_val = pid_compute_command(&temp_pid, error, ((now_time - last_pid_time) * portTICK_RATE_MS) / 1000.0f);
    cmd_event.value.float_val = clamp(cmd_event.value.float_val, 0.0f, 100.0f);

    xQueueSend(event_queue, &cmd_event, 0);

    last_pid_time = now_time;

    // Perform PWM duty cycle on output
    duty_cycle = cmd_event.value.float_val / 100.0f;
    on_time = duty_cycle * period;
    off_time = period - on_time;

    // Turn on heaters
    if(duty_cycle > 0.05f) {
      heat_event.value.int_val = 1;
      xQueueSend(event_queue, &heat_event, 0);
      gpio_set_level(TOP_HEATER_PIN, 1);
      gpio_set_level(BOTTOM_HEATER_PIN, 1);
      vTaskDelay((TickType_t) on_time / portTICK_RATE_MS);
    }

    // Turn off heaters
    if(duty_cycle < 0.95f) {
      heat_event.value.int_val = 0;
      xQueueSend(event_queue, &heat_event, 0);
      gpio_set_level(TOP_HEATER_PIN, 0);
      gpio_set_level(BOTTOM_HEATER_PIN, 0);
      vTaskDelay((TickType_t) off_time / portTICK_RATE_MS);
    }
  }
}

static void spinbox_cb(lv_obj_t *spinbox, int32_t new_value) {
  if(gui.p_gain_sbox == spinbox) {
    temp_pid.p_gain_ = (float) new_value / 10;
  }
  else if(gui.i_gain_sbox == spinbox) {
    temp_pid.i_gain_ = (float) new_value / 10;
  }
  else if(gui.d_gain_sbox == spinbox) {
    temp_pid.d_gain_ = (float) new_value / 10;
  }
  else if(gui.setpoint_sbox == spinbox) {
    setpoint = (float) new_value / 10;
  }
}

static lv_res_t main_menu_cb(lv_obj_t * clicked_btn) {
  if(gui.tune_pid_btn == clicked_btn) {
    lv_screen_show(gui.tune_pid_scr, gui.main_menu_scr);
  }
  else if(gui.home1_btn == clicked_btn) {
    lv_screen_show(gui.main_menu_scr, gui.tune_pid_scr);
    lv_group_set_editing(g, true);
  }
  return LV_RES_OK;
}

static lv_obj_t * pid_gain_sbox(lv_obj_t * par, const char *label, lv_coord_t x, lv_coord_t y) {
  lv_obj_t *sbox = lv_spinbox_create(par, NULL);
  lv_spinbox_set_digit_format(sbox, 3, 2);
  lv_spinbox_set_range(sbox, 0, 1000);
  lv_obj_set_size(sbox, 60, 30);
  lv_obj_align(sbox, NULL, LV_ALIGN_IN_TOP_LEFT, 4, 0);
  lv_obj_set_pos(sbox, x, y);
  lv_spinbox_set_value_changed_cb(sbox, spinbox_cb);

  lv_obj_t * sbox_label = lv_label_create(par, NULL);
  lv_label_set_text(sbox_label, label);
  lv_obj_align(sbox_label, sbox, LV_ALIGN_OUT_LEFT_MID, -10, 0);
  return sbox;
}

static lv_obj_t * indicator_led(lv_obj_t * par, const char * label) {
  lv_obj_t *led = lv_led_create(par, NULL);
  lv_obj_align(led, NULL, LV_ALIGN_IN_TOP_RIGHT, -20, 10);
  lv_led_off(led);

  lv_obj_t * led_label = lv_label_create(par, NULL);
  lv_label_set_text(led_label, label);
  lv_obj_align(led_label, led, LV_ALIGN_OUT_LEFT_MID, -10, 0);
  return led;
}

static lv_obj_t * temperature_chart(lv_obj_t * par) {
  /*Create a chart*/
  lv_obj_t * chart;
  chart = lv_chart_create(par, NULL);
  lv_obj_set_size(chart, 300, 120);
  lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 15);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_series_opa(chart, LV_OPA_70);                            /*Opacity of the data series*/
  lv_chart_set_series_width(chart, 2);                                  /*Line width and point radious*/
  lv_chart_set_range(chart, 0, 300);
  lv_chart_set_point_count(chart, 600);
  return chart;
}

static lv_obj_t * menu_list(lv_obj_t * par) {
  /* Create the list */
  lv_obj_t * list1 = lv_list_create(par, NULL);
  lv_obj_set_size(list1, 250, 170);
  lv_obj_align(list1, NULL, LV_ALIGN_CENTER, 0, 0);

  /*Create a label above the list*/
  lv_obj_t * label;
  label = lv_label_create(par, NULL);
  lv_label_set_text(label, "M5Stack Reflow Oven");
  lv_obj_align(label, list1, LV_ALIGN_OUT_TOP_MID, 0, -10);
  return list1;
}

static void gui_create(lv_gui_t* gui) {
  // Build the main menu screen
  gui->main_menu_scr = lv_screen_create(g);

  gui->menu_lst = menu_list(gui->main_menu_scr->screen);
  lv_screen_add_object(gui->main_menu_scr, gui->menu_lst);

  /*Add list elements*/
  gui->run_profile_btn = lv_list_add(gui->menu_lst, SYMBOL_CHARGE, "Run Profile...", main_menu_cb);
  gui->tune_pid_btn = lv_list_add(gui->menu_lst, SYMBOL_LIST, "Tune PID...", main_menu_cb);
  gui->settings_btn = lv_list_add(gui->menu_lst, SYMBOL_SETTINGS, "Settings", main_menu_cb);

  // Build the PID Tuning screen
  gui->tune_pid_scr = lv_screen_create(g);

  gui->temp_lbl = lv_label_create(gui->tune_pid_scr->screen, NULL);
  lv_obj_set_pos(gui->temp_lbl, 20, 10);
  lv_label_set_text(gui->temp_lbl, "Temp. = N/A");

  gui->setpoint_sbox = lv_spinbox_create(gui->tune_pid_scr->screen, NULL);
  lv_spinbox_set_digit_format(gui->setpoint_sbox, 4, 3);
  lv_spinbox_set_range(gui->setpoint_sbox, 0, 3000);
  lv_obj_set_size(gui->setpoint_sbox, 70, 30);
  lv_obj_align(gui->setpoint_sbox, NULL, LV_ALIGN_IN_TOP_LEFT, 4, 0);
  lv_obj_set_pos(gui->setpoint_sbox, 90, 40);
  lv_spinbox_set_value_changed_cb(gui->setpoint_sbox, spinbox_cb);
  lv_screen_add_object(gui->tune_pid_scr, gui->setpoint_sbox);

  lv_obj_t * setpoint_label = lv_label_create(gui->tune_pid_scr->screen, NULL);
  lv_label_set_text(setpoint_label, "Set T.");
  lv_obj_align(setpoint_label, gui->setpoint_sbox, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  gui->heat_led = indicator_led(gui->tune_pid_scr->screen, "Heat");

  gui->power_lbl = lv_label_create(gui->tune_pid_scr->screen, NULL);
  lv_obj_set_pos(gui->power_lbl, 230, 40);

  gui->temp_chart = temperature_chart(gui->tune_pid_scr->screen);
  gui->temp_curve = lv_chart_add_series(gui->temp_chart, LV_COLOR_RED);
  gui->setpoint_curve = lv_chart_add_series(gui->temp_chart, LV_COLOR_GREEN);

  gui->p_gain_sbox = pid_gain_sbox(gui->tune_pid_scr->screen, "P", 30, 200);
  lv_screen_add_object(gui->tune_pid_scr, gui->p_gain_sbox);

  gui->i_gain_sbox = pid_gain_sbox(gui->tune_pid_scr->screen, "I", 120, 200);
  lv_screen_add_object(gui->tune_pid_scr, gui->i_gain_sbox);

  gui->d_gain_sbox = pid_gain_sbox(gui->tune_pid_scr->screen, "D", 210, 200);
  lv_screen_add_object(gui->tune_pid_scr, gui->d_gain_sbox);

  gui->home1_btn = lv_btn_create(gui->tune_pid_scr->screen, NULL);
  lv_cont_set_fit(gui->home1_btn, true, true);
  lv_obj_set_pos(gui->home1_btn, 270, 200);
  lv_obj_t *home1_lbl = lv_label_create(gui->home1_btn, NULL);
  lv_label_set_text(home1_lbl, SYMBOL_HOME);
  lv_btn_set_action(gui->home1_btn, LV_BTN_ACTION_CLICK, main_menu_cb);
  lv_screen_add_object(gui->tune_pid_scr, gui->home1_btn);

  // Show the initial screen
  lv_screen_show(gui->main_menu_scr, NULL);
  lv_group_set_editing(g, true);
}

static lv_obj_t * max_temp_message() {
  char tmp_buff[BUFFER_SIZE];

  static lv_style_t style_bg;
  lv_style_copy(&style_bg, &lv_style_pretty);
  style_bg.body.main_color = LV_COLOR_MAKE(0xf5, 0x45, 0x2e);
  style_bg.body.grad_color = LV_COLOR_MAKE(0xb9, 0x1d, 0x09);
  style_bg.body.border.color = LV_COLOR_MAKE(0x3f, 0x0a, 0x03);
  style_bg.text.color = LV_COLOR_WHITE;
  style_bg.body.padding.hor = 12;
  style_bg.body.padding.ver = 8;
  style_bg.body.shadow.width = 8;

  lv_obj_t * mbox1 = lv_mbox_create(lv_scr_act(), NULL);
  lv_obj_set_style(mbox1, &style_bg);
  lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, -40);
  snprintf(tmp_buff, BUFFER_SIZE, "Maximum temperature limit of %.01f C reached.", MAX_TEMPERATURE);
  lv_mbox_set_text(mbox1, tmp_buff);
  return mbox1;
}

static void gui_update_task(void* arg) {
  lv_gui_t* gui = (lv_gui_t*) arg;

  TickType_t now_time = xTaskGetTickCount();
  TickType_t start_time = now_time;

  lv_spinbox_set_value(gui->setpoint_sbox, (int32_t) setpoint * 10);
  lv_spinbox_set_value(gui->p_gain_sbox, (int32_t) temp_pid.p_gain_ * 10);
  lv_spinbox_set_value(gui->i_gain_sbox, (int32_t) temp_pid.i_gain_ * 10);
  lv_spinbox_set_value(gui->d_gain_sbox, (int32_t) temp_pid.d_gain_ * 10);

  char tmp_buff[BUFFER_SIZE];
  event_t received_event;
  bool update_gui = true;
  while (1) {
    // Redraw GUI objects when there is an update from the event queue
    if(xQueueReceive(event_queue, &received_event, portMAX_DELAY) && update_gui) {
      now_time = xTaskGetTickCount();
      if(TEMPERATURE_UPDATE == received_event.type) {
        const float temperature = received_event.value.float_val;

        if(temperature >= MAX_TEMPERATURE) {
          max_temp_message();
          setpoint = 0.0f;
          update_gui = false;
        }

        temp_profile_get_point(mg_4860p, ((now_time - start_time) * portTICK_RATE_MS) / 1000.0f, &setpoint);

        snprintf(tmp_buff, BUFFER_SIZE, "Temp. = %.01f C", temperature);
        lv_label_set_text(gui->temp_lbl, tmp_buff);

        lv_chart_set_next(gui->temp_chart, gui->temp_curve, (lv_coord_t) temperature);
        lv_chart_set_next(gui->temp_chart, gui->setpoint_curve, (lv_coord_t) setpoint);
        lv_chart_refresh(gui->temp_chart);
      }
      else if(DUTY_CYCLE_UPDATE == received_event.type) {
        const float command = received_event.value.float_val;
        snprintf(tmp_buff, BUFFER_SIZE, "%.01f %%", command);
        lv_label_set_text(gui->power_lbl, tmp_buff);
      }
      else if(HEATER_UPDATE == received_event.type) {
        const int32_t command = received_event.value.int_val;
        if(command) {
          lv_led_on(gui->heat_led);
        }
        else {
          lv_led_off(gui->heat_led);
        }
      }
    }
  }
}

void app_main()
{
  event_queue = xQueueCreate(16, sizeof(event_t));
  pid_set_gains(&temp_pid, 16.0f, 0.0f, 90.0f, -50.0f, 50.0f, false);

  gpio_setup();
  i2c_setup();

  lv_init();

  disp_spi_init();
  ili9431_init();

  m5stack_kpad_init();

  lvgl_setup();

  lv_theme_set_current(lv_theme_night_init(NULL, NULL));
  gui_create(&gui);

  mg_4860p = temp_profile_create();
  temp_profile_add_point(mg_4860p, 0.0f, 25.0f);
  temp_profile_add_point(mg_4860p, 90.0f, 125.0f);
  temp_profile_add_point(mg_4860p, 150.0f, 140.0f);
  temp_profile_add_point(mg_4860p, 210.0f, 150.0f);
  temp_profile_add_point(mg_4860p, 240.0f, 160.0f);
  temp_profile_add_point(mg_4860p, 300.0f, 215.0f);
  temp_profile_add_point(mg_4860p, 330.0f, 183.0f);
  temp_profile_add_point(mg_4860p, 360.0f, 140.0f);

  xTaskCreate(gui_update_task, "gui_update_task", 1024 * 2, (void *) &gui, 5, NULL);
  xTaskCreate(temperature_task, "temperature_task", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(control_task, "control_task", 1024 * 2, NULL, 7, NULL);

  while (1) {
    vTaskDelay(1);
    lv_task_handler();
  }
}