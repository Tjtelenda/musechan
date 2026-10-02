/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* StackChan head base: servo power and the two head servos (MuseChan). */
#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Power the base through its M5IOE1 expander and start the servo bus.
 * Returns ESP_ERR_NOT_FOUND when no base answers (a bare CoreS3). */
esp_err_t stackchan_head_init(i2c_master_bus_handle_t bus);

/* Whether a StackChan head base answered at boot. */
bool stackchan_head_present(void);

/* Aim the head: yaw -128..128 degrees, pitch 0..90 (clamped). */
esp_err_t stackchan_head_look(int yaw_deg, int pitch_deg);

/* Back to yaw 0, pitch 0. */
esp_err_t stackchan_head_center(void);

/* The servos' motor supply, through the expander. Safe without a base. */
esp_err_t stackchan_head_set_power(bool on);

#ifdef __cplusplus
}
#endif
