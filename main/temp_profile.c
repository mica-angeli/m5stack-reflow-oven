//
// Created by ricardo on 2/20/19.
//

#include <stdlib.h>
#include "temp_profile.h"

temp_profile_t* temp_profile_create() {
  temp_profile_t* profile = malloc(sizeof(temp_profile_t));
  profile->size = 2;
  profile->points_len = 0;
  profile->points = malloc(sizeof(temp_point_t) * profile->size);
  return profile;
}

void temp_profile_delete(temp_profile_t* profile) {
  free(profile->points);
  free(profile);
}

void temp_profile_add_point(temp_profile_t* profile, float time, float temp) {
  if(profile->points_len == profile->size) {
    profile->size *= 2;
    profile->points = realloc(profile->points, sizeof(temp_point_t) * profile->size);
  }
  profile->points[profile->points_len].time = time;
  profile->points[profile->points_len].temp = temp;
  profile->points_len++;
}

esp_err_t temp_profile_get_point(const temp_profile_t *profile, float time, float *temperature) {
  // Check that the requested time value is within bounds
  if(time < profile->points[0].time) {
    return ESP_FAIL;
  }
  else if(time > profile->points[profile->points_len - 1].time) {
    *temperature = 0.0f;
    return ESP_FAIL;
  }

  size_t i;
  for(i = 0; i < profile->points_len; i++) {
    if(time == profile->points[i].time) {
      *temperature = profile->points[i].temp;
      return ESP_OK;
    }
    else if(time < profile->points[i].time) {
      const temp_point_t a = profile->points[i-1];
      const temp_point_t b = profile->points[i];

      *temperature =  a.temp + (b.temp - a.temp) * (time - a.time) / (b.time - a.time);
      return ESP_OK;
    }

  }

  return ESP_FAIL;
}