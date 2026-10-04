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
 * MuseChan: StackChan's head base. An M5IOE1 (PY32) I2C expander at 0x6F
 * switches the servo motor supply on its pin 0; two Feetech SCSCL servos
 * (yaw ID 1, pitch ID 2) share a 1 Mbaud UART. Only the two SCSCL writes the
 * head needs are implemented (torque enable, goal position); the packet
 * layout, the big-endian goal registers and the angle-to-raw scale are the
 * StackChan BSP's. Everything here is optional hardware: without a base the
 * calls report ESP_ERR_NOT_FOUND and the CoreS3 runs headless.
 */
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "muse_state.h"
#include "stackchan_head.h"

static const char *TAG = "stackchan_head";

#define IOE_ADDR 0x6F
#define IOE_VERSION 0x02
#define IOE_GPIO_M_L 0x03          /* 1 = output */
#define IOE_GPIO_O_L 0x05
#define IOE_GPIO_PU_L 0x09
#define IOE_SERVO_POWER BIT(0)
#define IOE_GPIO_M_H 0x04          /* high bank: pins 8..13, same layout as low */
#define IOE_GPIO_PU_H 0x0A
#define IOE_GPIO_PD_H 0x0C
#define IOE_GPIO_DRV_H 0x14        /* 1 = open-drain, so 0 is push-pull */
#define IOE_LED_CFG 0x24           /* base LEDs: count in low 6 bits, bit 6 refreshes */
#define IOE_LED_RAM 0x30           /* base LEDs: two RGB565 bytes per LED from here */
#define IOE_LED_DATA_BIT 5         /* LED data out is expander pin 13, high-bank bit 5 */
#define BASE_LED_COUNT 12          /* 0-5 on the left of the base, 6-11 on the right */

#define SERVO_UART UART_NUM_1
#define SERVO_TX GPIO_NUM_6
#define SERVO_RX GPIO_NUM_7
#define SERVO_YAW_ID 1
#define SERVO_PITCH_ID 2
#define SERVO_YAW_ZERO 460         /* raw at 0 degrees, the BSP's defaults */
#define SERVO_PITCH_ZERO 620
#define SERVO_REG_TORQUE 40
#define SERVO_REG_GOAL_POS 42
#define SCS_INST_WRITE 0x03

static i2c_master_dev_handle_t s_ioe;
static bool s_present;
static bool s_uart_ready;

static esp_err_t ioe_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_ioe, &reg, 1, val, 1, 100);
}

static esp_err_t ioe_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_ioe, buf, sizeof(buf), 100);
}

static esp_err_t ioe_update(uint8_t reg, uint8_t mask, bool on)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(ioe_read(reg, &v), TAG, "ioe read %02x", reg);
    uint8_t want = on ? v | mask : v & ~mask;
    return want == v ? ESP_OK : ioe_write(reg, want);
}

#if CONFIG_MUSE_STACKCHAN_PENDING_LED
/*
 * The twelve base LEDs, through the expander's LED engine. M5's BSP init
 * order (stackchan-bsp's M5StackChan.cpp): the LED data pin out with a
 * pull-up, push-pull, count 12, then an all-off frame. A frame is 24 bytes
 * of little-endian RGB565 from IOE_LED_RAM, pushed out by setting bit 6 of
 * the count register.
 */
static esp_err_t base_leds_show(uint8_t r, uint8_t g, uint8_t b)
{
    const uint16_t c = ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (uint16_t)(b >> 3);
    uint8_t buf[1 + BASE_LED_COUNT * 2];
    buf[0] = IOE_LED_RAM;
    for (int i = 0; i < BASE_LED_COUNT; i++) {
        buf[1 + i * 2] = (uint8_t)(c & 0xFF);
        buf[2 + i * 2] = (uint8_t)(c >> 8);
    }
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_ioe, buf, sizeof(buf), 100), TAG, "led frame");
    uint8_t cfg;
    ESP_RETURN_ON_ERROR(ioe_read(IOE_LED_CFG, &cfg), TAG, "led cfg read");
    return ioe_write(IOE_LED_CFG, cfg | (1 << 6));
}

static esp_err_t base_leds_init(void)
{
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_M_H, BIT(IOE_LED_DATA_BIT), true), TAG, "led pin output");
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_PU_H, BIT(IOE_LED_DATA_BIT), true), TAG, "led pin pull-up");
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_PD_H, BIT(IOE_LED_DATA_BIT), false), TAG, "led pin pull-down");
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_DRV_H, BIT(IOE_LED_DATA_BIT), false), TAG, "led pin push-pull");
    ESP_RETURN_ON_ERROR(ioe_write(IOE_LED_CFG, BASE_LED_COUNT), TAG, "led count");
    return base_leds_show(0, 0, 0);
}

/* Amber, half a second on and half off, while a reply from Muse waits. */
static void pending_led_task(void *arg)
{
    (void)arg;
    bool was_pending = false, lit = false;
    int64_t phase_ms = 0;
    for (;;) {
        const bool pending = muse_state_reply_pending();
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (pending && !was_pending) {
            ESP_LOGI(TAG, "base LEDs: a reply from Muse is waiting, flashing");
            was_pending = true;
            phase_ms = now_ms - 600;  /* first frame right away */
        }
        if (!pending && was_pending) {
            base_leds_show(0, 0, 0);
            ESP_LOGI(TAG, "base LEDs: the reply from Muse was read, LEDs off");
            was_pending = false;
            lit = false;
        }
        if (pending && now_ms - phase_ms >= 500) {
            phase_ms = now_ms;
            lit = !lit;
            base_leds_show(lit ? 255 : 0, lit ? 140 : 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif /* CONFIG_MUSE_STACKCHAN_PENDING_LED */

/* FF FF ID LEN INST ADDR DATA... CHK, with CHK the inverted low byte of the
 * sum from ID through DATA (the SCSCL packet layout). */
static esp_err_t scs_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t n)
{
    if (!s_uart_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t pkt[16];
    size_t len = n + 3;              /* LEN counts INST, ADDR, DATA and CHK */
    if (len + 3 > sizeof(pkt)) {
        return ESP_ERR_INVALID_SIZE;
    }
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = (uint8_t)len;
    pkt[4] = SCS_INST_WRITE;
    pkt[5] = addr;
    unsigned sum = id + len + SCS_INST_WRITE + addr;
    for (size_t i = 0; i < n; i++) {
        pkt[6 + i] = data[i];
        sum += data[i];
    }
    pkt[6 + n] = (uint8_t)~sum;
    uart_write_bytes(SERVO_UART, pkt, (int)(n + 7));   /* header, data, CHK */
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(50));
    return ESP_OK;
}

/* Goal position, time and speed are big-endian at SCSCL_GOAL_POSITION_L:
 * the BSP's Host2SCS with End=1 stores the high byte first. */
static esp_err_t servo_write_pos(uint8_t id, int raw, uint16_t time)
{
    raw = raw < 0 ? 0 : raw > 1000 ? 1000 : raw;
    const uint8_t data[6] = {
        (uint8_t)(raw >> 8), (uint8_t)raw,
        (uint8_t)(time >> 8), (uint8_t)time,
        0, 0,                        /* speed 0: as fast as the time allows */
    };
    return scs_write(id, SERVO_REG_GOAL_POS, data, sizeof(data));
}

static esp_err_t servo_torque(uint8_t id, bool on)
{
    const uint8_t data[1] = { on ? 1 : 0 };
    return scs_write(id, SERVO_REG_TORQUE, data, sizeof(data));
}

/* Degrees to raw counts: the BSP's zero + angle_tenths * 16 / 5 / 10. */
static int angle_to_raw(int deg, int zero, int min_deg, int max_deg)
{
    deg = deg < min_deg ? min_deg : deg > max_deg ? max_deg : deg;
    return zero + deg * 10 * 16 / 50;
}

esp_err_t stackchan_head_init(i2c_master_bus_handle_t bus)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IOE_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_ioe), TAG, "ioe");

    /* The expander runs its own firmware and takes a moment to boot; the
     * BSP waits up to about 1.2 s for its version register to answer. */
    uint8_t version = 0;
    for (int waited = 0; waited < 1200; waited += 50) {
        if (ioe_read(IOE_VERSION, &version) == ESP_OK && version != 0x00 && version != 0xFF) {
            break;
        }
        version = 0;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!version) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "M5IOE1 expander, version %u", version);

    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_M_L, IOE_SERVO_POWER, true), TAG, "servo pin output");
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_PU_L, IOE_SERVO_POWER, true), TAG, "servo pin pull-up");
    ESP_RETURN_ON_ERROR(ioe_update(IOE_GPIO_O_L, IOE_SERVO_POWER, true), TAG, "servo power on");
    s_present = true;

#if CONFIG_MUSE_STACKCHAN_PENDING_LED
    if (base_leds_init() == ESP_OK) {
        xTaskCreate(pending_led_task, "stackchan_led", 3072, NULL, 2, NULL);
    } else {
        ESP_LOGW(TAG, "base LEDs unavailable");
    }
#endif

    const uart_config_t uart_cfg = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(SERVO_UART, &uart_cfg), TAG, "servo uart cfg");
    ESP_RETURN_ON_ERROR(uart_set_pin(SERVO_UART, SERVO_TX, SERVO_RX,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "servo uart pins");
    ESP_RETURN_ON_ERROR(uart_driver_install(SERVO_UART, 256, 0, 0, NULL, 0), TAG, "servo uart");
    s_uart_ready = true;

    vTaskDelay(pdMS_TO_TICKS(100));
    servo_torque(SERVO_YAW_ID, true);
    servo_torque(SERVO_PITCH_ID, true);
    return stackchan_head_center();
}

bool stackchan_head_present(void)
{
    return s_present;
}

esp_err_t stackchan_head_look(int yaw_deg, int pitch_deg)
{
    if (!s_uart_ready) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(servo_write_pos(SERVO_YAW_ID,
                                        angle_to_raw(yaw_deg, SERVO_YAW_ZERO, -128, 128), 20),
                        TAG, "yaw servo");
    return servo_write_pos(SERVO_PITCH_ID,
                           angle_to_raw(pitch_deg, SERVO_PITCH_ZERO, 0, 90), 20);
}

esp_err_t stackchan_head_center(void)
{
    return stackchan_head_look(0, 0);
}

esp_err_t stackchan_head_set_power(bool on)
{
    if (!s_present) {
        return ESP_OK;   /* nothing to switch; the board's power-off goes on */
    }
    if (!on) {
        servo_torque(SERVO_YAW_ID, false);
        servo_torque(SERVO_PITCH_ID, false);
    }
    return ioe_update(IOE_GPIO_O_L, IOE_SERVO_POWER, on);
}
