/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Captures one frame for the Muse camera.capture command on StackChan. */
bool stackchan_camera_capture(char **jpeg_base64, const char **error);
esp_err_t stackchan_camera_prepare(void);

#ifdef __cplusplus
}
#endif
