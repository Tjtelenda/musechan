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

/*
 * StackChan's camera.capture, on the board's camera (components/camera;
 * board_m5stack_stackchan.c registers the CoreS3's GC0308). One still at a
 * time, base64 JPEG for the Muse that asked; no preview, the paired Muse
 * aims the head with stackchan.look between captures instead.
 */
#include "stackchan_camera.h"

#include <stdatomic.h>
#include <stdlib.h>

#include "camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mbedtls/base64.h"

static const char *TAG = "stackchan.camera";
static SemaphoreHandle_t s_lock;
static atomic_bool s_busy;

esp_err_t stackchan_camera_prepare(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    return s_lock ? ESP_OK : ESP_ERR_NO_MEM;
}

/* `jpeg` as base64, for camera.capture's result. */
static char *to_base64(const uint8_t *jpeg, size_t len)
{
    size_t cap = (len + 2) / 3 * 4 + 1, out = 0;
    char *b64 = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (b64 && mbedtls_base64_encode((unsigned char *)b64, cap, &out, jpeg, len) != 0) {
        free(b64);
        return NULL;
    }
    return b64;
}

bool stackchan_camera_capture(char **jpeg_base64, const char **error)
{
    if (!jpeg_base64 || !error) return false;
    *jpeg_base64 = NULL;
    if (stackchan_camera_prepare() != ESP_OK) {
        *error = "camera memory allocation failed";
        return false;
    }
    if (atomic_exchange(&s_busy, true)) {
        *error = "camera busy";
        return false;
    }
    camera_frame_t frame;
    esp_err_t err = camera_capture(&frame);
    if (err != ESP_OK) {
        *error = err == ESP_ERR_TIMEOUT ? "Muse camera did not respond"
                 : err == ESP_ERR_NO_MEM ? "camera memory allocation failed"
                 : err == ESP_ERR_NOT_SUPPORTED ? "camera unavailable"
                                                 : "camera capture failed";
        ESP_LOGW(TAG, "Muse camera capture failed: %s", esp_err_to_name(err));
        atomic_store(&s_busy, false);
        return false;
    }
    *jpeg_base64 = to_base64(frame.jpeg, frame.len);
    camera_release(&frame);
    *error = *jpeg_base64 ? NULL : "camera memory allocation failed";
    atomic_store(&s_busy, false);
    return *jpeg_base64 != NULL;
}
