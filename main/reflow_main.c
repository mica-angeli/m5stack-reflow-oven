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

#define TOP_HEATER_PIN 16
#define BOTTOM_HEATER_PIN 17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<TOP_HEATER_PIN) | (1ULL<<BOTTOM_HEATER_PIN))

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


static unsigned int rand_interval(unsigned int min, unsigned int max)
{
  const unsigned int range = 1 + max - min;
  return min + (rand() % range);
}

static color_t random_color() {

  color_t color;
  color.r  = (uint8_t)rand_interval(20,252);
  color.g  = (uint8_t)rand_interval(20,252);
  color.b  = (uint8_t)rand_interval(20,252);
  return color;
}

static esp_err_t i2c_master_init()
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
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

//=============
void app_main()
{
  Mcp9600 temp_sensor;

  tft_setup();
  gpio_setup();
  i2c_master_init();
  Mcp_init(&temp_sensor, 0x50, I2C_MASTER_NUM);

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
  while (1) {
    Mcp_read_hot_junc(&temp_sensor, &temperature);

    TFT_fillWindow(TFT_BLACK);

    gpio_set_level(TOP_HEATER_PIN, top_level);
    gpio_set_level(BOTTOM_HEATER_PIN, bottom_level);

    _fg = TFT_WHITE;
    TFT_print("TOP H. = ", 10, 5);

    _fg = gpio_get_level(TOP_HEATER_PIN) ? TFT_RED : TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "%s", gpio_get_level(TOP_HEATER_PIN) ? "ON " : "OFF");
    TFT_print(tmp_buff, LASTX, LASTY);

    _fg = TFT_WHITE;
    TFT_print("   BOT. H. = ", LASTX, LASTY);

    _fg = gpio_get_level(BOTTOM_HEATER_PIN) ? TFT_RED : TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "%s", gpio_get_level(BOTTOM_HEATER_PIN) ? "ON " : "OFF");
    TFT_print(tmp_buff, LASTX, LASTY);

    top_level = top_level ? 0 : 1;
    bottom_level = bottom_level ? 0 : 1;

    _fg = TFT_WHITE;
    snprintf(tmp_buff, BUFFER_SIZE, "Temp. = %f C", temperature);
    TFT_print(tmp_buff, CENTER, (dispWin.y2-dispWin.y1)/2 - tempy);

    vTaskDelay(2000 / portTICK_RATE_MS);
  }
}