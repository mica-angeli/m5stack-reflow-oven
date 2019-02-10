//
// Created by ricardo on 2/9/19.
//

#include "mcp9600.h"

static esp_err_t Mcp_read_byte(const Mcp9600 * this, uint8_t reg, uint8_t *output) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->address << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, output, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(this->master_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t Mcp_read_16bit(const Mcp9600 * this, uint8_t reg, uint16_t *output) {
  uint8_t raw_data[2];
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->address << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, raw_data, 2, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(this->master_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  *output = raw_data [0] << 8 | raw_data[1];
  return ret;
}

esp_err_t Mcp_init(Mcp9600 *this, uint8_t sensor_address, i2c_port_t master_port) {
  this->address = sensor_address;
  this->master_port = master_port;
  return ESP_OK;
}

esp_err_t Mcp_read_version(const Mcp9600 *this, uint16_t *version) {
  return Mcp_read_16bit(this, VERSION_ID_REG_ADDR, version);
}

esp_err_t Mcp_read_hot_junc(const Mcp9600 *this, float *temperature) {
  *temperature = 0.0f;
  uint16_t read_value = 0;
  if(ESP_OK != Mcp_read_16bit(this, 0x0, &read_value)) {
    return ESP_FAIL;
  }

  *temperature = read_value / 1000.0f;

  return ESP_OK;
}