/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Ricardo Angeli
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Ported from a BSD-licensed C++ ROS library ( https://github.com/ros-controls/control_toolbox )
//  licensed by Willow Garage, Inc.

#ifndef PID_LIBRARY_PID_H
#define PID_LIBRARY_PID_H

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

//typedef enum { false, true } bool;

struct pid {
  float p_gain_;
  float i_gain_;
  float d_gain_;
  float i_max_;
  float i_min_;
  bool antiwindup_;

  float p_error_last_;
  float p_error_;
  float i_error_;
  float d_error_;
  float cmd_;
};

typedef struct pid pid;

/** @brief Clamp a value to a specified range.
 *
 * @param val Value to clamp.
 * @param lo Low end of the range to clamp the value to.
 * @param hi High end of the range to clamp the value to.
 * @return Clamped result.
 */
extern float clamp(float val, float lo, float hi);

/** @brief Set PID gains for the controller.
 *
 * @param pid Pointer to PID object.
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * @param i_max The max integral windup.
 * @param i_min The min integral windup.
 * @param antiwindup Whether the integral antiwindup feature is enabled
 *                   (Default: false).
 * @return Void.
 */
extern void pid_set_gains(pid* pid, float p, float i, float d, float i_max, float i_min, bool antiwindup);

/** @brief Get PID gains for the controller.
 *
 * @param pid Pointer to PID object.
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * @param i_max The max integral windup.
 * @param i_min The min integral windup.
 * @param antiwindup Whether the integral antiwindup feature is enabled.
 * @return Void.
 */
extern void pid_get_gains(const pid* pid, float* p, float* i, float* d, float* i_max, float* i_min, bool* antiwindup);

/** @brief Set the PID error and compute the PID command with nonuniform time
 * step size. The derivative error is computed from the change in the error
 * and the timestep \c dt.
 *
 * @param pid Pointer to PID object.
 * @param error Error since last call (error = target - state)
 * @param dt Change in time since last call (seconds)
 * @return PID command
 */
extern float pid_compute_command(pid* pid, float error, float dt);

/** @brief Set the PID error and compute the PID command with nonuniform
 * time step size. This also allows the user to pass in a precomputed
 * derivative error.
 *
 * @param pid Pointer to PID object.
 * @param error Error since last call (error = target - state)
 * @param error_dot d(Error)/dt since last call
 * @param dt Change in time since last call (seconds)
 * @return PID command
 */
extern float pid_compute_command_with_error_dot(pid* pid, float error, float error_dot, float dt);

/** @brief Reset the state of this PID controller.
 *
 * @param pid Pointer to PID object.
 * @return Void.
 */
extern void pid_reset(pid* pid);

/** @brief Print to console the current parameters.
 *
 * @param pid Pointer to PID object.
 * @return Void.
 */
extern void pid_print_values(const pid* pid);

#endif //PID_LIBRARY_PID_H
