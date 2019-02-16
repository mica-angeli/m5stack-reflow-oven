#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "tftspi.h"
#include "tft.h"
#include "mcp9600.h"
#include "pid.h"

// ==========================================================
// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_HSPI_HOST
// ==========================================================

#define BUFFER_SIZE 128

// GPIO Settings
#define TOP_HEATER_PIN 16
#define BOTTOM_HEATER_PIN 17
#define BUTTON_A_PIN 39
#define BUTTON_B_PIN 38
#define BUTTON_C_PIN 37

#define BUTTON_DEBOUNCING_TICKS 200
#define ESP_INTR_FLAG_DEFAULT 0

// I2C Settings
#define I2C_MASTER_SCL 22
#define I2C_MASTER_SDA 21
#define I2C_MASTER_NUM 1
#define I2C_MASTER_FREQ_HZ 50000
#define I2C_MASTER_SCL_IO I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO I2C_MASTER_SDA
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

//typedef enum {
//  BUTTON_A_PRESSED,
//  BUTTON_B_PRESSED,
//  BUTTON_C_PRESSED,
//  BUTTON_LEN
//} button_t;

// Global variables
static xQueueHandle event_queue = NULL;

static float setpoint = 100.0f;
static float pid_output = 0.0f;

typedef enum {
  BUTTON_PRESSED,
  TEMPERATURE_UPDATE,
  DUTY_CYCLE_UPDATE
} event_type_t;

typedef struct {
  event_type_t type;
  void* value;
} event_t;

static void IRAM_ATTR button_handler(void* arg) {
  static TickType_t last_time = 0;
  const TickType_t now_time = xTaskGetTickCountFromISR();

  event_t event = {
      .type = BUTTON_PRESSED,
      .value = arg
  };

  // Button debouncing
  if(!gpio_get_level((uint32_t) arg) &&
      (now_time - last_time) > BUTTON_DEBOUNCING_TICKS) {
    xQueueSendFromISR(event_queue, &event, NULL);
  }
  last_time = now_time;
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

void tft_setup() {
  esp_err_t ret;

  tft_disp_type = DEFAULT_DISP_TYPE;
  _width = DEFAULT_TFT_DISPLAY_WIDTH;  // smaller dimension
  _height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
  max_rdclock = 8000000;

  TFT_PinsInit();

  spi_lobo_device_handle_t spi;

  spi_lobo_bus_config_t buscfg = {
      .miso_io_num=PIN_NUM_MISO,				// set SPI MISO pin
      .mosi_io_num=PIN_NUM_MOSI,				// set SPI MOSI pin
      .sclk_io_num=PIN_NUM_CLK,				// set SPI CLK pin
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
      .max_transfer_sz = 6*1024,
  };

  spi_lobo_device_interface_config_t devcfg = {
      .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
      .mode=0,                                // SPI mode 0
      .spics_io_num=-1,                       // we will use external CS pin
      .spics_ext_io_num=PIN_NUM_CS,           // external CS pin
      .flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
  };

  vTaskDelay(500 / portTICK_RATE_MS);

  // ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

  ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
  assert(ret == ESP_OK);

  disp_spi = spi;

  // ==== Test select/deselect ====
  ret = spi_lobo_device_select(spi, 1);
  assert(ret == ESP_OK);

  ret = spi_lobo_device_deselect(spi);
  assert(ret == ESP_OK);

  // ==== Initialize the Display ====
  TFT_display_init();

  spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
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

  gpio_config_t button_conf = {
      .intr_type = GPIO_PIN_INTR_NEGEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL<<BUTTON_A_PIN) | (1ULL<<BUTTON_B_PIN) | (1ULL<<BUTTON_C_PIN),
      .pull_down_en = 0,
      .pull_up_en = 0
  };
  gpio_config(&button_conf);

  event_queue = xQueueCreate(10, sizeof(event_t) + 4);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(BUTTON_A_PIN, button_handler, (void*) BUTTON_A_PIN);
  gpio_isr_handler_add(BUTTON_B_PIN, button_handler, (void*) BUTTON_B_PIN);
  gpio_isr_handler_add(BUTTON_C_PIN, button_handler, (void*) BUTTON_C_PIN);
}

static void control_task(void* arg) {
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

  event_t cmd_event = {
      .type = DUTY_CYCLE_UPDATE,
      .value = NULL
  };

  float temperature = 0.0f;

  pid temp_pid;
  pid_set_gains(&temp_pid, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);

  TickType_t last_pid_time = 0;

  while(1) {
    const TickType_t now_time = xTaskGetTickCount();

    if(ESP_OK == Mcp_get_hot_junc(&temp_sensor, (float *) &temp_event.value)) {
      temperature = *(float *)&temp_event.value;

      xQueueSend(event_queue, &temp_event, 0);
    }

    if((now_time - last_pid_time) >= 1000 / portTICK_RATE_MS) {
      const float error = setpoint - temperature;

      float* command = (float *) &cmd_event.value;
      *command = pid_compute_command(&temp_pid, error, (now_time - last_pid_time) * portTICK_RATE_MS);
      *command = clamp(*command, 0.0f, 100.0f);
      pid_output = *command;

      xQueueSend(event_queue, &cmd_event, 0);

      last_pid_time = now_time;
    }
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

static void heater_task(void* arg) {
  float duty_cycle = 0.0f;
  const float period = 1000.0f;
  float on_time, off_time;

  while(1) {
    duty_cycle = pid_output / 100.0f;
    on_time = duty_cycle * period;
    off_time = period - on_time;

    // Turn on heaters
    gpio_set_level(TOP_HEATER_PIN, 1);
    gpio_set_level(BOTTOM_HEATER_PIN, 1);
    vTaskDelay ((TickType_t) on_time / portTICK_RATE_MS);

    // Turn off heaters
    gpio_set_level(TOP_HEATER_PIN, 0);
    gpio_set_level(BOTTOM_HEATER_PIN, 0);
    vTaskDelay ((TickType_t) off_time / portTICK_RATE_MS);
  }
}

static void display_task(void* arg) {


  char tmp_buff[BUFFER_SIZE];

  font_rotate = 0;
  text_wrap = 0;
  font_transparent = 0;
  font_forceFixed = 0;
  TFT_resetclipwin();

  image_debug = 0;

  gray_scale = 0;

  TFT_setRotation(LANDSCAPE);
  const int tempy = TFT_getfontheight() + 8;

  TFT_setFont(DEJAVU18_FONT, NULL);

  _fg = TFT_WHITE;

  uint32_t top_level = 0;
  uint32_t bottom_level = 1;
  event_t received_event;

  while (1) {
  // Old heater code
//    gpio_set_level(TOP_HEATER_PIN, top_level);
//    gpio_set_level(BOTTOM_HEATER_PIN, bottom_level);
//
//    _fg = TFT_WHITE;
//    TFT_print("TOP H. = ", 10, 5);
//
//    _fg = gpio_get_level(TOP_HEATER_PIN) ? TFT_RED : TFT_WHITE;
//    snprintf(tmp_buff, BUFFER_SIZE, "\r%s", gpio_get_level(TOP_HEATER_PIN) ? "ON " : "OFF");
//    TFT_print(tmp_buff, LASTX, LASTY);
//
//    _fg = TFT_WHITE;
//    TFT_print("   BOT. H. = ", LASTX, LASTY);
//
//    _fg = gpio_get_level(BOTTOM_HEATER_PIN) ? TFT_RED : TFT_WHITE;
//    snprintf(tmp_buff, BUFFER_SIZE, "\r%s", gpio_get_level(BOTTOM_HEATER_PIN) ? "ON " : "OFF");
//    TFT_print(tmp_buff, LASTX, LASTY);
//
//    top_level = top_level ? 0 : 1;
//    bottom_level = bottom_level ? 0 : 1;
    _fg = TFT_WHITE;
    // Redraw GUI objects when there is an update from the event queue
    if(xQueueReceive(event_queue, &received_event, portMAX_DELAY)) {
      if(BUTTON_PRESSED == received_event.type &&
         !gpio_get_level((uint32_t) received_event.value)) {
        snprintf(tmp_buff, BUFFER_SIZE, "Button #%d Pressed\n", (uint32_t) received_event.value);
        TFT_print(tmp_buff, 10, 60);
      }
      else if(TEMPERATURE_UPDATE == received_event.type) {
        const float temperature = *(float *)&received_event.value;
        snprintf(tmp_buff, BUFFER_SIZE, "Temp. = \r%.01f C\n", temperature);
        TFT_print(tmp_buff, 10, 80);
      }
      else if(DUTY_CYCLE_UPDATE == received_event.type) {
        const float command = *(float *)&received_event.value;
        snprintf(tmp_buff, BUFFER_SIZE, "Duty Cycle = \r%.01f %% \n", command);
        TFT_print(tmp_buff, 10, 100);
      }
    }


  }
}

void app_main()
{
  tft_setup();
  gpio_setup();
  i2c_setup();

  xTaskCreate(display_task, "display_task", 1024 * 2, NULL, 5, NULL);
  xTaskCreate(control_task, "control_task", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(heater_task, "heater_task", 1024 * 2, NULL, 7, NULL);
}