//
// Created by ricardo on 2/9/19.
//

#ifndef _MCP9600_H
#define _MCP9600_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// I2C Defines
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ               /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// MCP9600 Defines
#define HOT_JUNCTION_REG_ADDR               0X0
#define JUNCTION_TEMP_DELTA_REG_ADDR        0X1
#define COLD_JUNCTION_TEMP_REG_ADDR         0X2
#define RAW_ADC_DATA_REG_ADDR               0X3
#define STAT_REG_ADDR                       0X4
#define THERM_SENS_CFG_REG_ADDR             0X5
#define DEVICE_CFG_REG_ADDR                 0X6

#define VERSION_ID_REG_ADDR                 0x20

#define UPDATE_FLAG                         1<<6

typedef enum {
  COLD_JUNC_RESOLUTION_0_625 = 0,
  COLD_JUNC_RESOLUTION_0_25
} mcp_cold_junc_res_t;

typedef enum {
  ADC_18BIT_RESOLUTION = 0,
  ADC_16BIT_RESOLUTION,
  ADC_14BIT_RESOLUTION,
  ADC_12BIT_RESOLUTION
} mcp_adc_res_t;

typedef enum {
  BURST_1_SAMPLE = 0,
  BURST_2_SAMPLE,
  BURST_4_SAMPLE,
  BURST_8_SAMPLE,
  BURST_16_SAMPLE,
  BURST_32_SAMPLE,
  BURST_64_SAMPLE,
  BURST_128_SAMPLE
} mcp_burst_t;

typedef enum {
  NORMAL_OPERATION = 0,
  SHUTDOWN_MODE,
  BURST_MODE
} mcp_shutdown_t;

typedef enum {
  THER_TYPE_K = 0,
  THER_TYPE_J,
  THER_TYPE_T,
  THER_TYPE_N,
  THER_TYPE_S,
  THER_TYPE_E,
  THER_TYPE_B,
  THER_TYPE_R
} mcp_ther_t;

typedef enum {
  FILT_OFF = 0,
  FILT_MIN = 1,
  FILT_MID = 4,
  FILT_MAX = 7,
} mcp_filter_coeff_t;

typedef struct {
  uint8_t address;
  i2c_port_t master_port;

  mcp_ther_t thermocouple_type;
  mcp_filter_coeff_t filter_coefficents;
  mcp_shutdown_t shutdown_modes;
  mcp_burst_t burst_mode_samples;
  mcp_adc_res_t adc_resolution;
  mcp_cold_junc_res_t cold_junction_resolution;

} Mcp9600;

extern esp_err_t Mcp_configure(const Mcp9600 *this);

extern esp_err_t Mcp_read_version(const Mcp9600 *this, uint16_t *version);

extern esp_err_t Mcp_read_hot_junc(const Mcp9600 *this, float *temperature);
#endif //_MCP9600_H
