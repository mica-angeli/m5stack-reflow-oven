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
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SCL_IO I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO I2C_MASTER_SDA
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define ESP_SLAVE_ADDR 0x50 //0x60 for thermocouple

//typedef enum {
//  BUTTON_A_PRESSED,
//  BUTTON_B_PRESSED,
//  BUTTON_C_PRESSED,
//  BUTTON_LEN
//} button_t;

static xQueueHandle button_evt_queue = NULL;

static void IRAM_ATTR button_handler(void* arg) {
  static TickType_t last_time = 0;
  const TickType_t now_time = xTaskGetTickCountFromISR();

  uint32_t button = (uint32_t) arg;

  // Button debouncing
  if(!gpio_get_level(button) &&
      (now_time - last_time) > BUTTON_DEBOUNCING_TICKS) {
    xQueueSendFromISR(button_evt_queue, &button, NULL);
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

  button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(BUTTON_A_PIN, button_handler, (void*) BUTTON_A_PIN);
  gpio_isr_handler_add(BUTTON_B_PIN, button_handler, (void*) BUTTON_B_PIN);
  gpio_isr_handler_add(BUTTON_C_PIN, button_handler, (void*) BUTTON_C_PIN);
}

//=============
void app_main()
{
  tft_setup();
  gpio_setup();
  i2c_setup();

  // Configure thermocouple
  Mcp9600 temp_sensor = {
      .address = 0x50,
      .master_port = I2C_MASTER_NUM,
      .thermocouple_type = THER_TYPE_K,
      .filter_coefficents = FILT_MID,
      .shutdown_modes = NORMAL_OPERATION,
      .burst_mode_samples = BURST_32_SAMPLE,
      .adc_resolution = ADC_14BIT_RESOLUTION,
      .cold_junction_resolution = COLD_JUNC_RESOLUTION_0_25
  };
  Mcp_configure(&temp_sensor);

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
  float temperature = 0.0f;
  uint32_t button = 0;
  while (1) {
    Mcp_get_hot_junc(&temp_sensor, &temperature);

    gpio_set_level(TOP_HEATER_PIN, top_level);
    gpio_set_level(BOTTOM_HEATER_PIN, bottom_level);

    _fg = TFT_WHITE;
    TFT_print("TOP H. = ", 10, 5);

    _fg = gpio_get_level(TOP_HEATER_PIN) ? TFT_RED : TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "\r%s", gpio_get_level(TOP_HEATER_PIN) ? "ON " : "OFF");
    TFT_print(tmp_buff, LASTX, LASTY);

    _fg = TFT_WHITE;
    TFT_print("   BOT. H. = ", LASTX, LASTY);

    _fg = gpio_get_level(BOTTOM_HEATER_PIN) ? TFT_RED : TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "\r%s", gpio_get_level(BOTTOM_HEATER_PIN) ? "ON " : "OFF");
    TFT_print(tmp_buff, LASTX, LASTY);

    top_level = top_level ? 0 : 1;
    bottom_level = bottom_level ? 0 : 1;

    _fg = TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "Temp. = \r%.03f C\n", temperature);
    TFT_print(tmp_buff, 60, (dispWin.y2-dispWin.y1)/2 - tempy);

    if(xQueueReceive(button_evt_queue, &button, portMAX_DELAY) && !gpio_get_level(button)) {
      snprintf(tmp_buff, BUFFER_SIZE, "Button #%d Pressed\n", button);
      TFT_print(tmp_buff, CENTER, LASTY);
    }

  }
}