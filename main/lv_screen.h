/**
 * @file lv_screen.c
 *
 */

#ifndef LV_SCREEN_H
#define LV_SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <lvgl/lv_core/lv_obj.h>
#include <lvgl/lv_core/lv_group.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
  lv_obj_t* screen;
  lv_group_t* group;
  lv_obj_t** objects;
  size_t objects_len;
  size_t size;
} lv_screen_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_screen_t * lv_screen_create(lv_group_t* group);

void lv_screen_add_object(lv_screen_t * scr, lv_obj_t* obj);

void lv_screen_show(lv_screen_t * scr, lv_screen_t * prev_scr);

void lv_screen_delete(lv_screen_t * scr);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif



#endif //LV_SCREEN_H
