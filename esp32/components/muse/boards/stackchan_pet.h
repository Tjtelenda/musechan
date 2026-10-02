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

/* CoreS3 motion and head-touch sensing, turned into face reactions (MuseChan). */
#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the pet task (BMI270 IMU plus the Si12T head-touch sensor; either
 * may be absent). Reactions go through muse_state, like the SDK's own
 * touch-to-pet: happy face, a poke to keep Muse awake, sleep when the
 * robot is put face-down. */
esp_err_t stackchan_pet_start(i2c_master_bus_handle_t bus);

/* True while a hand is on the head (the Si12T reports a touch). */
bool stackchan_pet_head_touched(void);

/* The stackchan.face device command: "idle", "listening", "thinking",
 * "speaking", "error", "boot", "off", or "happy". */
esp_err_t stackchan_face_set(const char *face);

#ifdef __cplusplus
}
#endif
