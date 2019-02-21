//
// Created by ricardo on 2/20/19.
//

#ifndef TEMP_PROFILE_H
#define TEMP_PROFILE_H

#include <stddef.h>
#include <esp_err.h>

typedef struct {
  float time;
  float temp;
} temp_point_t;

typedef struct {
  temp_point_t * points;
  size_t points_len;
  size_t size;
} temp_profile_t;

temp_profile_t* temp_profile_create();

void temp_profile_delete(temp_profile_t* profile);

void temp_profile_add_point(temp_profile_t* profile, float time, float temp);

esp_err_t temp_profile_get_point(const temp_profile_t *profile, float time, float *temperature);

#endif //TEMP_PROFILE_H
