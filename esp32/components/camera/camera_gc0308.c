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
 * The CoreS3's GC0308 over espressif/esp32-camera. The pin map, the 20 MHz
 * onboard oscillator and the PSRAM frame buffers are M5's own (M5CoreS3's
 * GC0308.cpp; the sensor has its own clock, so XCLK is -1). Two departures
 * from their config: SCCB joins the board's existing I2C_NUM_0 bus instead
 * of taking pins of its own (their pin_sscb_sda/scl are 12/11, the bus the
 * board file already runs), and stills are encoded to JPEG on the way out
 * (esp32-camera's frame2jpg): the GC0308 has no JPEG encoder of its own,
 * so the sensor runs YUV422, its native output. jpge, the encoder under frame2jpg, keeps
 * ~8 KB of static tables that internal RAM cannot spare (the Link
 * session's transmit reserve lives there), so the camera component's
 * linker fragment places them in PSRAM instead (linker.lf alongside).
 * The camera powers down between captures for the same reason: its DMA
 * buffers and descriptors cost internal RAM the Link session needs, so
 * each capture starts the camera, takes its frame and stops it again.
 * The camera's reset is the AW9523's P1_0, released in the board file;
 * esp32-camera never sees it.
 */
#include "camera_gc0308.h"

#include <stdint.h>
#include <stdlib.h>

#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "camera_gc0308";

static const camera_config_t s_cam_cfg = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = -1,
    .pin_sccb_sda = -1,   /* the board's bus, by number below */
    .pin_sccb_scl = -1,
    .pin_d7 = 47,
    .pin_d6 = 48,
    .pin_d5 = 16,
    .pin_d4 = 15,
    .pin_d3 = 42,
    .pin_d2 = 41,
    .pin_d1 = 40,
    .pin_d0 = 39,
    .pin_vsync = 46,
    .pin_href = 38,
    .pin_pclk = 45,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_YUV422,   /* the sensor's native output; frame2jpg converts */
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0,   /* I2C_NUM_0: SDA GPIO12, SCL GPIO11, the board's bus */
    .jpeg_buffer_size = 0,
};

static bool s_ready;

static esp_err_t gc0308_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    esp_err_t err = esp_camera_init(&s_cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Muse camera didn't start: %s", esp_err_to_name(err));
        return err;
    }
    s_ready = true;
    ESP_LOGI(TAG, "GC0308 camera ready (VGA YUV422, JPEG on capture)");
    return ESP_OK;
}

static esp_err_t gc0308_capture(camera_frame_t *out)
{
    /* A previous capture powered the camera down (below); start it again. */
    esp_err_t err = gc0308_init();
    if (err != ESP_OK) {
        return err;
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "Muse camera capture timed out");
        esp_camera_deinit();
        s_ready = false;
        return ESP_ERR_TIMEOUT;
    }
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    esp_err_t result = ESP_OK;
    if (!frame2jpg(fb, 80, &jpg, &jpg_len) || jpg_len < 4 ||
        jpg[0] != 0xFF || jpg[1] != 0xD8) {
        ESP_LOGW(TAG, "Muse camera frame didn't encode to JPEG");
        free(jpg);
        result = ESP_FAIL;
    } else {
        out->jpeg = jpg;
        out->len = jpg_len;
        out->width = (int)fb->width;
        out->height = (int)fb->height;
        out->priv = jpg;   /* frame2jpg's malloc: release frees it */
        ESP_LOGI(TAG, "Muse camera capture: %dx%d JPEG, %u bytes",
                 out->width, out->height, (unsigned)out->len);
    }
    esp_camera_fb_return(fb);
    /* The camera's frame buffers and DMA descriptors hold internal RAM the
     * Link session's transmit reserve needs, so the camera powers down
     * between captures; the next capture starts it again. */
    esp_camera_deinit();
    s_ready = false;
    return result;
}

static void gc0308_release(camera_frame_t *frame)
{
    free(frame->priv);
}

static const camera_driver_t s_driver = {
    .name = "GC0308 (M5Stack CoreS3)",
    .max_width = 640,
    .max_height = 480,
    .init = gc0308_init,
    .capture = gc0308_capture,
    .release = gc0308_release,
};

const camera_driver_t *camera_gc0308(void)
{
    return &s_driver;
}
