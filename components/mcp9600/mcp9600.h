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


#define ALERT1_CFG_REG_ADDR                 0X8
#define ALERT2_CFG_REG_ADDR                 0X9
#define ALERT3_CFG_REG_ADDR                 0XA
#define ALERT4_CFG_REG_ADDR                 0XB
#define ALERT1_HYS_REG_ADDR                 0XC
#define ALERT2_HYS_REG_ADDR                 0XD
#define ALERT3_HYS_REG_ADDR                 0XE
#define ALERT4_HYS_REG_ADDR                 0XF

#define TEMP_ALERT1_LIMIT_REG_ADDR          0X10
#define TEMP_ALERT2_LIMIT_REG_ADDR          0X11
#define TEMP_ALERT3_LIMIT_REG_ADDR          0X12
#define TEMP_ALERT4_LIMIT_REG_ADDR          0X13

#define VERSION_ID_REG_ADDR                 0x20


#define DEFAULT_IIC_ADDR  0X60

#define RESOLUTION_0_5_DEGREE               0
#define RESOLUTION_0_25_DEGREE              0X01
#define RESOLUTION_0_125_DEGREE             0X02
#define RESOLUTION_0_0625_DEGREE            0X03

#define THER_TYPE_K                         0X0<<4
#define THER_TYPE_J                         0X1<<4
#define THER_TYPE_T                         0X2<<4
#define THER_TYPE_N                         0X3<<4
#define THER_TYPE_S                         0X4<<4
#define THER_TYPE_E                         0X5<<4
#define THER_TYPE_B                         0X6<<4
#define THER_TYPE_R                         0X7<<4


#define ALERT_NUN_1                         1
#define ALERT_NUN_2                         2
#define ALERT_NUN_3                         3
#define ALERT_NUN_4                         4


#define FILT_OFF                            0
#define FILT_MIN                            1
#define FILT_MID                            4
#define FILT_MAX                            7


#define ENABLE   true
#define DISABLE  false


#define COLD_JUNC_RESOLUTION_0_625          0<<7
#define COLD_JUNC_RESOLUTION_0_25           1<<7

#define ADC_18BIT_RESOLUTION                0<<5
#define ADC_16BIT_RESOLUTION                1<<5
#define ADC_14BIT_RESOLUTION                2<<5
#define ADC_12BIT_RESOLUTION                3<<5

#define BURST_1_SAMPLE                      0<<2
#define BURST_2_SAMPLE                      1<<2
#define BURST_4_SAMPLE                      2<<2
#define BURST_8_SAMPLE                      3<<2
#define BURST_16_SAMPLE                     4<<2
#define BURST_32_SAMPLE                     5<<2
#define BURST_64_SAMPLE                     6<<2
#define BURST_128_SAMPLE                    7<<2

#define NORMAL_OPERATION                    0
#define SHUTDOWN_MODE                       1
#define BURST_MODE                          2

#define ACTIVE_LOW                          0<<2
#define ACTIVE_HIGH                         1<<2

#define INT_MODE                            1<<1
#define COMPARE_MODE                        0<<1

#define UPDATE_FLAG                         1<<6

typedef struct {
  uint8_t address;
  i2c_port_t master_port;

} Mcp9600;

extern esp_err_t Mcp_init(Mcp9600 *this, uint8_t sensor_address, i2c_port_t master_port);

extern esp_err_t Mcp_read_version(const Mcp9600 *this, uint16_t *version);

extern esp_err_t Mcp_read_hot_junc(const Mcp9600 *this, float *temperature);

#endif //_MCP9600_H
