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
 * MuseChan pet behaviors. A task polls the CoreS3's BMI270 (I2C 0x69) and
 * the StackChan head's Si12T touch sensor (0x68), and turns what it feels
 * into the SDK's own pet vocabulary: muse_state_make_happy() for the face
 * (the same call the UI makes when the face is tapped), muse_state_poke()
 * to keep Muse awake, muse_state_set_asleep() when the robot is put
 * face-down. With the head base attached, reactions also move the head.
 * Thresholds are the first guesses; the task logs what it felt so they can
 * be tuned from a hardware run.
 */
#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi270.h"
#include "muse_state.h"
#include "stackchan_head.h"
#include "stackchan_pet.h"

static const char *TAG = "stackchan_pet";

#define SI12T_ADDR 0x68
#define SI12T_CTRL1 0x08           /* CFIG; 0x22 is the BSP's default */
#define SI12T_CTRL2 0x09
#define SI12T_OUTPUT1 0x10         /* three channels, two bits each */

#define POLL_MS 20
#define SHAKE_JERK 0.5f            /* g per sample that counts as a jolt */
#define SHAKE_HITS 6               /* jolts inside the window = a shake */
#define SHAKE_WINDOW 25            /* samples (500 ms) */
#define TAP_JERK 1.7f              /* a single hard jolt = a tap */
/* The BMI270 sits flat on the CoreS3 PCB, so its Z is the screen normal.
 * Which sign is face-down is a first-hardware-run check: the task logs the
 * gravity vector whenever the orientation class changes. */
#define FACE_DOWN_Z (-0.85f)
#define FACE_UP_Z 0.2f             /* clearly no longer face-down */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_si12t;
static bmi270_handle_t *s_imu;
static volatile bool s_head_touched;
static bool s_face_down;
static int64_t s_last_react_ms;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t si12t_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_si12t, buf, sizeof(buf), 50);
}

/* The BSP's Si12T begin(): channels enabled, the control words it ships,
 * sensitivity level 3 (low range) on all five sensitivity registers. */
static esp_err_t si12t_init(void)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI12T_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_si12t), TAG, "si12t");
    ESP_RETURN_ON_ERROR(si12t_write(0x0A, 0x00), TAG, "si12t ref rst 1");
    ESP_RETURN_ON_ERROR(si12t_write(0x0B, 0x00), TAG, "si12t ref rst 2");
    ESP_RETURN_ON_ERROR(si12t_write(0x0C, 0x00), TAG, "si12t ch hold 1");
    ESP_RETURN_ON_ERROR(si12t_write(0x0D, 0x00), TAG, "si12t ch hold 2");
    ESP_RETURN_ON_ERROR(si12t_write(0x0E, 0x00), TAG, "si12t cal hold 1");
    ESP_RETURN_ON_ERROR(si12t_write(0x0F, 0x00), TAG, "si12t cal hold 2");
    ESP_RETURN_ON_ERROR(si12t_write(SI12T_CTRL2, 0x0F), TAG, "si12t ctrl2 reset");
    ESP_RETURN_ON_ERROR(si12t_write(SI12T_CTRL2, 0x07), TAG, "si12t ctrl2");
    ESP_RETURN_ON_ERROR(si12t_write(SI12T_CTRL1, 0x22), TAG, "si12t cfig");
    for (uint8_t reg = 0x02; reg <= 0x06; reg++) {
        ESP_RETURN_ON_ERROR(si12t_write(reg, 0x33), TAG, "si12t sens %02x", reg);
    }
    uint8_t out;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_si12t, (uint8_t[]){ SI12T_OUTPUT1 }, 1,
                                                    &out, 1, 50), TAG, "si12t probe");
    return ESP_OK;
}

/* A hand on the head, or any happy event: the face reacts the way the SDK's
 * own tap-on-the-face does. While asleep, a pat also wakes. */
static void react_pet(bool wake)
{
    if (muse_state_asleep()) {
        if (!wake) {
            return;
        }
        muse_state_set_asleep(false);
    }
    muse_state_poke();
    muse_state_make_happy();
    s_last_react_ms = now_ms();
}

static void react_shake(void)
{
    if (muse_state_asleep()) {
        return;
    }
    react_pet(false);
    if (stackchan_head_present()) {
        stackchan_head_look(-14, 0);
        vTaskDelay(pdMS_TO_TICKS(140));
        stackchan_head_look(14, 0);
        vTaskDelay(pdMS_TO_TICKS(140));
        stackchan_head_center();
    }
}

static void react_pickup(void)
{
    if (muse_state_asleep()) {
        return;
    }
    react_pet(false);
    if (stackchan_head_present()) {
        stackchan_head_look(0, 18);   /* look up at whoever picked it up */
        vTaskDelay(pdMS_TO_TICKS(450));
        stackchan_head_center();
    }
}

static void pet_task(void *arg)
{
    (void)arg;
    bmi270_driver_config_t imu_cfg = {
        .addr = BMI270_I2C_ADDRESS_H,
        .interface = BMI270_USE_I2C,
        .i2c_bus = s_bus,
    };
    bool imu_ok = bmi270_create(&imu_cfg, &s_imu) == ESP_OK;
    if (imu_ok) {
        const bmi270_config_t start = {
            .acce_odr = BMI270_ACC_ODR_100_HZ,
            .acce_range = BMI270_ACC_RANGE_8_G,
            .gyro_odr = BMI270_GYR_ODR_100_HZ,
            .gyro_range = BMI270_GYR_RANGE_2000_DPS,
        };
        imu_ok = bmi270_start(s_imu, &start) == ESP_OK;
    }
    ESP_LOGI(TAG, "BMI270 %s", imu_ok ? "ready" : "not found");
    bool touch_ok = si12t_init() == ESP_OK;
    ESP_LOGI(TAG, "Si12T head touch %s", touch_ok ? "ready" : "not found");
    if (!imu_ok && !touch_ok) {
        vTaskDelete(NULL);
    }

    float gx = 0, gy = 0, gz = 0;
    bool g_init = false;
    float px = 0, py = 0, pz = 0;
    bool p_init = false;
    uint8_t shake_ring[SHAKE_WINDOW] = { 0 };
    int shake_at = 0, shake_hits = 0;
    float g_hist[50][3];
    int g_at = 0;
    bool g_full = false;
    int face_down_ms = 0;
    bool was_touched = false;

    while (true) {
        if (imu_ok) {
            float ax, ay, az;
            if (bmi270_get_acce_data(s_imu, &ax, &ay, &az) == ESP_OK) {
                if (!g_init) {
                    gx = ax;
                    gy = ay;
                    gz = az;
                    g_init = true;
                } else {
                    gx += (ax - gx) * 0.06f;
                    gy += (ay - gy) * 0.06f;
                    gz += (az - gz) * 0.06f;
                }

                float jerk = 0;
                if (p_init) {
                    float dx = ax - px, dy = ay - py, dz = az - pz;
                    jerk = sqrtf(dx * dx + dy * dy + dz * dz);
                }
                px = ax;
                py = ay;
                pz = az;
                p_init = true;

                /* Shake: repeated jolts inside a sliding half-second. A
                 * single harder jolt is a tap. Cooldown keeps a reaction
                 * from retriggering itself. */
                shake_hits -= shake_ring[shake_at];
                shake_ring[shake_at] = jerk > SHAKE_JERK ? 1 : 0;
                shake_hits += shake_ring[shake_at];
                shake_at = (shake_at + 1) % SHAKE_WINDOW;
                int64_t now = now_ms();
                if (shake_hits >= SHAKE_HITS) {
                    memset(shake_ring, 0, sizeof(shake_ring));
                    shake_hits = 0;
                    if (now - s_last_react_ms > 900) {
                        ESP_LOGI(TAG, "shake (g %.2f %.2f %.2f)", gx, gy, gz);
                        react_shake();
                    }
                } else if (jerk > TAP_JERK && now - s_last_react_ms > 900) {
                    ESP_LOGI(TAG, "tap (jerk %.2f g)", jerk);
                    react_pet(false);
                }

                /* Picked up: the gravity direction swings away from where
                 * it sat a second ago. */
                g_hist[g_at][0] = gx;
                g_hist[g_at][1] = gy;
                g_hist[g_at][2] = gz;
                g_at = (g_at + 1) % 50;
                if (g_at == 0) {
                    g_full = true;
                }
                if (g_full) {
                    const float *old = g_hist[g_at];   /* the oldest sample */
                    float dot = gx * old[0] + gy * old[1] + gz * old[2];
                    float mag = sqrtf(gx * gx + gy * gy + gz * gz) *
                                sqrtf(old[0] * old[0] + old[1] * old[1] + old[2] * old[2]);
                    if (mag > 0.5f && dot / mag < 0.75f && now - s_last_react_ms > 2000) {
                        ESP_LOGI(TAG, "picked up (g %.2f %.2f %.2f)", gx, gy, gz);
                        react_pickup();
                    }
                }

                /* Face-down for a moment: tuck in. Face-up again: wake.
                 * Sleeping only happens idle, so a talk is never cut off. */
                if (gz < FACE_DOWN_Z) {
                    face_down_ms += POLL_MS;
                    if (face_down_ms >= 1200 && !s_face_down) {
                        s_face_down = true;
                        ESP_LOGI(TAG, "face down (g %.2f %.2f %.2f)", gx, gy, gz);
                        if (!muse_state_asleep() &&
                            muse_state_mode(NULL) == MUSE_MODE_IDLE &&
                            muse_state_idle_secs() > 3.0f) {
                            muse_state_set_asleep(true);
                        }
                    }
                } else {
                    face_down_ms = 0;
                    if (s_face_down && gz > FACE_UP_Z) {
                        s_face_down = false;
                        ESP_LOGI(TAG, "face up (g %.2f %.2f %.2f)", gx, gy, gz);
                        react_pet(true);
                    }
                }
            }
        }

        if (touch_ok) {
            uint8_t out = 0;
            if (i2c_master_transmit_receive(s_si12t, (uint8_t[]){ SI12T_OUTPUT1 }, 1,
                                            &out, 1, 50) == ESP_OK) {
                bool touched = (out & 0x3F) != 0;
                s_head_touched = touched;
                if (touched && !was_touched) {
                    ESP_LOGI(TAG, "head pat (channels %02x)", out);
                    react_pet(true);
                }
                was_touched = touched;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

esp_err_t stackchan_pet_start(i2c_master_bus_handle_t bus)
{
    s_bus = bus;
    return xTaskCreate(pet_task, "stackchan_pet", 4096, NULL, 3, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

bool stackchan_pet_head_touched(void)
{
    return s_head_touched;
}

esp_err_t stackchan_face_set(const char *face)
{
    if (!face) {
        return ESP_ERR_INVALID_ARG;
    }
    muse_mode_t mode;
    if (!strcmp(face, "happy")) {
        muse_state_poke();
        muse_state_make_happy();
        return ESP_OK;
    } else if (!strcmp(face, "idle")) {
        mode = MUSE_MODE_IDLE;
    } else if (!strcmp(face, "listening")) {
        mode = MUSE_MODE_LISTENING;
    } else if (!strcmp(face, "thinking")) {
        mode = MUSE_MODE_THINKING;
    } else if (!strcmp(face, "speaking")) {
        mode = MUSE_MODE_SPEAKING;
    } else if (!strcmp(face, "error")) {
        mode = MUSE_MODE_ERROR;
    } else if (!strcmp(face, "boot")) {
        mode = MUSE_MODE_BOOT;
    } else if (!strcmp(face, "off")) {
        mode = MUSE_MODE_OFF;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    muse_state_poke();
    muse_state_set_mode(mode);
    return ESP_OK;
}
