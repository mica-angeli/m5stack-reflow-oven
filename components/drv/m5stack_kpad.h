/**
 * @file m5stack_kpad.h
 *
 */

#ifndef M5STACK_KPAD_H
#define M5STACK_KPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include <stdbool.h>
#include "lvgl/lv_hal/lv_hal_indev.h"

/*********************
 *      DEFINES
 *********************/

#define KPAD_A_PIN  39
#define KPAD_B_PIN  38
#define KPAD_C_PIN  37


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
extern void m5stack_kpad_init();

extern bool m5stack_kpad_read(lv_indev_data_t *data);

extern bool m5stack_enc_read(lv_indev_data_t *data);
/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*M5STACK_KPAD_H*/
