/**
 * @file lv_screen.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdlib.h>
#include "lv_screen.h"

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

lv_screen_t * lv_screen_create(lv_group_t* group) {
  lv_screen_t *scr = malloc(sizeof(lv_screen_t));
  scr->objects = malloc(sizeof(lv_obj_t*) * 2);
  scr->objects_len = 0;
  scr->size = 2;
  scr->screen = lv_obj_create(NULL, NULL);
  scr->group = group;
  return scr;
}

void lv_screen_add_object(lv_screen_t * scr, lv_obj_t* obj) {
  if(scr->objects_len == scr->size) {
    scr->size *= 2;
    scr->objects = realloc(scr->objects, sizeof(lv_obj_t*) * scr->size);
  }
  scr->objects[scr->objects_len++] = obj;
}

void lv_screen_show(lv_screen_t * scr, lv_screen_t * prev_scr) {
  size_t i;
  // Remove all group objects from previous screen
  if(prev_scr != NULL) {
    for(i = 0; i < prev_scr->objects_len; i++) {
      lv_group_remove_obj(prev_scr->objects[i]);
    }
  }

  // Add all group objects from new screen
  for(i = 0; i < scr->objects_len; i++) {
    lv_group_add_obj(scr->group, scr->objects[i]);
  }

  lv_group_focus_obj(scr->objects[0]);
  if(scr->objects_len == 1) {
    lv_group_set_editing(scr->group, true);
  }

  lv_scr_load(scr->screen);
}

void lv_screen_delete(lv_screen_t * scr) {
  free(scr->objects);
  free(scr);
}