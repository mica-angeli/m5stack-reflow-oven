/**
 * @file m5stack_kpad.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <lv_group.h>
#include "m5stack_kpad.h"
#include "esp_system.h"
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void m5stack_kpad_init() {
  gpio_config_t button_conf = {
      .intr_type = GPIO_PIN_INTR_DISABLE, //GPIO_PIN_INTR_NEGEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << KPAD_A_PIN) | (1ULL << KPAD_B_PIN) | (1ULL << KPAD_C_PIN),
      .pull_down_en = 0,
      .pull_up_en = 0
  };
  gpio_config(&button_conf);

//    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//    gpio_isr_handler_add(BUTTON_A_PIN, button_handler, (void*) BUTTON_A_PIN);
//    gpio_isr_handler_add(BUTTON_B_PIN, button_handler, (void*) BUTTON_B_PIN);
//    gpio_isr_handler_add(BUTTON_C_PIN, button_handler, (void*) BUTTON_C_PIN);
}

bool m5stack_kpad_read(lv_indev_data_t *data) {
  static uint32_t last_key = 0;

  if(!gpio_get_level(KPAD_A_PIN)) {
    data->state = LV_INDEV_STATE_PR;
    last_key = LV_GROUP_KEY_NEXT;
  }
  else if(!gpio_get_level(KPAD_B_PIN)) {
    data->state = LV_INDEV_STATE_PR;
    last_key = LV_GROUP_KEY_PREV;
  }
  else if(!gpio_get_level(KPAD_C_PIN)) {
    data->state = LV_INDEV_STATE_PR;
    last_key = LV_GROUP_KEY_ENTER;
  }
  else {
    data->state = LV_INDEV_STATE_REL;
  }

  data->key = last_key;
  return false;
}

bool m5stack_enc_read(lv_indev_data_t *data) {
  static uint32_t last_key = 0;
  if(!gpio_get_level(KPAD_B_PIN) && KPAD_B_PIN != last_key) {
    data->enc_diff = 1;
    last_key = KPAD_B_PIN;
  }
  else if(!gpio_get_level(KPAD_C_PIN) && KPAD_C_PIN != last_key) {
    data->enc_diff = -1;
    last_key = KPAD_C_PIN;
  }
  else if(KPAD_B_PIN == last_key && gpio_get_level(KPAD_B_PIN)) {
    last_key = 0;
  }
  else if(KPAD_C_PIN == last_key && gpio_get_level(KPAD_C_PIN)) {
    last_key = 0;
  }
  else {
    data->enc_diff = 0;
  }
  data->state = gpio_get_level(KPAD_A_PIN) ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;
  return false;
}
/**********************
 *   STATIC FUNCTIONS
 **********************/
