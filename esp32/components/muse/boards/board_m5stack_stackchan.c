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
 * M5Stack StackChan, on its CoreS3 controller: ESP32-S3 (16 MB flash, 8 MB
 * octal PSRAM), 2.0" 320x240 ILI9342C/E touch LCD, AW88298 speaker amp and
 * ES7210 mic ADC on I2S, AXP2101 power (I2C 0x34), AW9523B expander (0x58)
 * holding the LCD reset, the backlight rail and the amp enable, FT5x06-family
 * touch (0x38) and a BMI270 IMU (0x69). The StackChan head base adds two
 * Feetech SCSCL servos on UART1 and an Si12T head-touch sensor (0x68); both
 * belong to the StackChan build only. The CoreS3-only build compiles this
 * same CoreS3 hardware driver without the head base, pet reactions or
 * StackChan device commands. Every rail, pin and register below was taken
 * from M5's own sources (M5GFX, M5Unified, M5CoreS3) and the StackChan BSP;
 * the CoreS3's GC0308 camera pins are M5CoreS3's GC0308.cpp, wired up under
 * CONFIG_MUSE_STACKCHAN_CAMERA.
 */
#include <math.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aw88298_dac.h"
#include "es7210_adc.h"
#include "muse_audio.h"
#include "muse_board.h"
#include "muse_mem.h"
#include "muse_pmu.h"
#if CONFIG_MUSE_BOARD_M5STACK_STACKCHAN
#include "stackchan_head.h"
#include "stackchan_pet.h"
#if CONFIG_MUSE_STACKCHAN_CAMERA
#include "camera.h"
#include "camera_gc0308.h"
#include "boards/stackchan_camera.h"
#endif
#endif

static const char *TAG = "board";

/* FT6336U touch controller: TOUCH_RST is AW9523 port 0 bit 0, TOUCH_INT
 * is port 1 bit 2 (Mouser CoreS3 datasheet, "CAP.TOUCH" block). The LCD's
 * reset is port 1 bit 1. Keep RST released once the display is up. */
#define AW9523_TOUCH_RST BIT(0)

#define LCD_W 320
#define LCD_H 240
#define LCD_HOST SPI2_HOST
#define LCD_SCLK GPIO_NUM_36
#define LCD_MOSI GPIO_NUM_37
#define LCD_DC GPIO_NUM_35         /* also the panel's SDO; see the variant probe */
#define LCD_CS GPIO_NUM_3
#define DRAW_BUF_LINES 16

#define I2C_SDA GPIO_NUM_12
#define I2C_SCL GPIO_NUM_11
#define I2S_MCLK GPIO_NUM_0
#define I2S_BCLK GPIO_NUM_34
#define I2S_WS GPIO_NUM_33
#define I2S_DOUT GPIO_NUM_13
#define I2S_DIN GPIO_NUM_14

#define AXP_ADDR 0x34
#define AXP_LDO_CTRL 0x90          /* bit 7: DLDO1, the backlight rail */
#define AXP_DLDO1_VOLT 0x99        /* 100 mV steps from 2.5 V (value 20) */

#define AW9523_ADDR 0x58
#define AW9523_OUT_P0 0x02
#define AW9523_OUT_P1 0x03
#define AW9523_CFG_P0 0x04
#define AW9523_CFG_P1 0x05
#define AW9523_GCR 0x11
#define AW9523_BOOST_EN BIT(7)     /* P1_7: the SY7088 boost for the base */
#define AW9523_LCD_RST BIT(1)      /* P1_1 */
#define AW9523_CAM_RST BIT(0)      /* P1_0: the GC0308's reset (M5's CAM_RST) */
#define AW9523_SPK_EN BIT(2)       /* P0_2: the AW88298's enable */

static i2c_master_bus_handle_t s_i2c;
static i2c_master_dev_handle_t s_axp;
static i2c_master_dev_handle_t s_aw9523;
static esp_lcd_panel_handle_t s_panel;

/* The expander and the PMU both sit behind one shared bus; retry a transfer
 * the way the StickS3 board does, since a busy expander can miss one. */
static esp_err_t axp_read(uint8_t reg, uint8_t *buf, size_t n)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++) {
        err = i2c_master_transmit_receive(s_axp, &reg, 1, buf, n, 50);
    }
    return err;
}

static esp_err_t axp_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++) {
        err = i2c_master_transmit(s_axp, buf, sizeof(buf), 50);
    }
    return err;
}

static esp_err_t axp_update(uint8_t reg, uint8_t mask, bool on)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(axp_read(reg, &v, 1), TAG, "axp read %02x", reg);
    uint8_t want = on ? v | mask : v & ~mask;
    return want == v ? ESP_OK : axp_write(reg, want);
}

static esp_err_t aw9523_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++) {
        err = i2c_master_transmit(s_aw9523, buf, sizeof(buf), 50);
    }
    return err;
}

static esp_err_t aw9523_update(uint8_t reg, uint8_t mask, bool on)
{
    uint8_t v = 0;
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++) {
        err = i2c_master_transmit_receive(s_aw9523, &reg, 1, &v, 1, 50);
    }
    ESP_RETURN_ON_ERROR(err, TAG, "aw9523 read %02x", reg);
    uint8_t want = on ? v | mask : v & ~mask;
    return want == v ? ESP_OK : aw9523_write(reg, want);
}

/* The codec driver enables the amp through this as the speaker opens and
 * closes; the pin number is ignored, our one output is the enable. */
static int aw9523_gpio_setup(int16_t gpio, audio_gpio_dir_t dir, audio_gpio_mode_t mode)
{
    (void)gpio;
    (void)dir;
    (void)mode;
    return ESP_CODEC_DEV_OK;
}

static int aw9523_gpio_set(int16_t gpio, bool high)
{
    (void)gpio;
    return aw9523_update(AW9523_OUT_P0, AW9523_SPK_EN, high) == ESP_OK
        ? ESP_CODEC_DEV_OK : ESP_CODEC_DEV_WRITE_FAIL;
}

static bool aw9523_gpio_get(int16_t gpio)
{
    (void)gpio;
    uint8_t v = 0;
    i2c_master_transmit_receive(s_aw9523, (uint8_t[]){ AW9523_OUT_P0 }, 1, &v, 1, 50);
    return v & AW9523_SPK_EN;
}

static const audio_codec_gpio_if_t s_aw9523_gpio = {
    .setup = aw9523_gpio_setup,
    .set = aw9523_gpio_set,
    .get = aw9523_gpio_get,
};

/* M5GFX's ILI9342C init list (Panel_ILI9342.hpp), as esp_lcd_ili9341 vendor
 * commands. The driver first sends its own SLPOUT, MADCTL and COLMOD; the
 * list ends in DISPON/SLPOUT again, which is harmless. */
static const uint8_t s_ili9342c_gamma_p[] = { 0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89,
                                              0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34, 0x0F };
static const uint8_t s_ili9342c_gamma_n[] = { 0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67,
                                              0x48, 0x07, 0x0E, 0x0B, 0x2E, 0x33, 0x0F };
static const ili9341_lcd_init_cmd_t s_ili9342c_cmds[] = {
    { 0xC8, (const uint8_t[]){ 0xFF, 0x93, 0x42 }, 3, 0 },   /* SETEXTC */
    { 0xC0, (const uint8_t[]){ 0x12, 0x12 }, 2, 0 },         /* PWCTR1 */
    { 0xC1, (const uint8_t[]){ 0x03 }, 1, 0 },               /* PWCTR2 */
    { 0xC5, (const uint8_t[]){ 0xF2 }, 1, 0 },               /* VMCTR1 */
    { 0xB0, (const uint8_t[]){ 0xE0 }, 1, 0 },
    { 0xF6, (const uint8_t[]){ 0x01, 0x00, 0x00 }, 3, 0 },
    { 0xE0, s_ili9342c_gamma_p, sizeof(s_ili9342c_gamma_p), 0 },
    { 0xE1, s_ili9342c_gamma_n, sizeof(s_ili9342c_gamma_n), 0 },
    { 0xB6, (const uint8_t[]){ 0x08, 0x82, 0x1D, 0x04 }, 4, 0 }, /* DFUNCTR */
    { 0x38, NULL, 0, 0 },                                    /* IDMOFF */
    { 0x29, NULL, 0, 0 },                                    /* DISPON */
    { 0x11, NULL, 0, 120 },                                  /* SLPOUT */
};

/* The ILI9342E list (Panel_ILI9342E). The C list must not be sent to an E. */
static const uint8_t s_ili9342e_gamma_p[] = { 0x04, 0x08, 0x11, 0x06, 0x12, 0x07, 0x3A, 0x76,
                                              0x47, 0x07, 0x0F, 0x0A, 0x11, 0x19, 0x05 };
static const uint8_t s_ili9342e_gamma_n[] = { 0x02, 0x03, 0x07, 0x06, 0x12, 0x07, 0x36, 0x5F,
                                              0x48, 0x06, 0x10, 0x0C, 0x16, 0x14, 0x09 };
static const ili9341_lcd_init_cmd_t s_ili9342e_cmds[] = {
    { 0xDD, (const uint8_t[]){ 0x01 }, 1, 0 },               /* SETEXTC */
    { 0xD5, (const uint8_t[]){ 0x00 }, 1, 0 },
    { 0xB1, (const uint8_t[]){ 0x22 }, 1, 0 },
    { 0xC8, (const uint8_t[]){ 0x38 }, 1, 0 },
    { 0xCB, (const uint8_t[]){ 0x1C }, 1, 0 },
    { 0xC9, (const uint8_t[]){ 0x1A }, 1, 0 },
    { 0xCA, (const uint8_t[]){ 0x1A }, 1, 0 },
    { 0xB7, (const uint8_t[]){ 0x5A, 0x41, 0x11, 0x19 }, 4, 0 },
    { 0xE4, s_ili9342e_gamma_p, sizeof(s_ili9342e_gamma_p), 0 },
    { 0xE5, s_ili9342e_gamma_n, sizeof(s_ili9342e_gamma_n), 0 },
    { 0x38, NULL, 0, 0 },
    { 0x29, NULL, 0, 0 },
    { 0x11, NULL, 0, 120 },
};

/*
 * Two panel revisions ship in CoreS3s, told apart on the bus before the panel
 * exists (M5GFX's _probe_ili9342_variant). GPIO35 is both the bus's MISO and
 * the panel's D/C, so the probe bit-bangs: D/C out for the command, then the
 * pin flips to an input to clock the answer in. The raw read-back is logged
 * so a unit that answers in an unexpected alignment is diagnosable from the
 * boot log; the C is the fallback, as in M5GFX.
 */
static void probe_write(uint8_t cmd, const uint8_t *data, size_t n)
{
    gpio_set_level(LCD_CS, 0);
    gpio_set_level(LCD_DC, 0);
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(LCD_MOSI, (cmd >> i) & 1);
        gpio_set_level(LCD_SCLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(LCD_SCLK, 0);
        esp_rom_delay_us(1);
    }
    gpio_set_level(LCD_DC, 1);
    for (size_t b = 0; b < n; b++) {
        for (int i = 7; i >= 0; i--) {
            gpio_set_level(LCD_MOSI, (data[b] >> i) & 1);
            gpio_set_level(LCD_SCLK, 1);
            esp_rom_delay_us(1);
            gpio_set_level(LCD_SCLK, 0);
            esp_rom_delay_us(1);
        }
    }
    gpio_set_level(LCD_CS, 1);
}

static uint16_t probe_read_param(uint8_t cmd, uint8_t index)
{
    probe_write(0xD9, (const uint8_t[]){ 0x10 | index }, 1);
    uint16_t raw = 0;
    gpio_set_level(LCD_CS, 0);
    gpio_set_level(LCD_DC, 0);
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(LCD_MOSI, (cmd >> i) & 1);
        gpio_set_level(LCD_SCLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(LCD_SCLK, 0);
        esp_rom_delay_us(1);
    }
    gpio_set_direction(LCD_DC, GPIO_MODE_INPUT);
    for (int i = 0; i < 16; i++) {
        gpio_set_level(LCD_SCLK, 1);
        esp_rom_delay_us(1);
        raw = raw << 1 | gpio_get_level(LCD_DC);
        gpio_set_level(LCD_SCLK, 0);
        esp_rom_delay_us(1);
    }
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_CS, 1);
    return raw;
}

/* The answer at each plausible alignment, M5GFX-normalised ((v >> 1) & 0x7F)
 * and plain. Both probe keys must match at the same alignment. */
static bool probe_keys_match(uint16_t raw0, uint16_t raw1, uint8_t want0, uint8_t want1)
{
    for (int shift = 8; shift >= 6; shift--) {
        uint8_t a0 = (raw0 >> shift) & 0xFF, a1 = (raw1 >> shift) & 0xFF;
        if (((a0 >> 1) & 0x7F) == want0 && ((a1 >> 1) & 0x7F) == want1) {
            return true;
        }
        if ((a0 & 0x7F) == want0 && (a1 & 0x7F) == want1) {
            return true;
        }
    }
    return false;
}

static bool probe_panel_is_e(void)
{
    const gpio_config_t pins = {
        .pin_bit_mask = BIT64(LCD_SCLK) | BIT64(LCD_MOSI) | BIT64(LCD_DC) | BIT64(LCD_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pins);
    gpio_set_level(LCD_CS, 1);
    gpio_set_level(LCD_SCLK, 0);

    probe_write(0xD9, (const uint8_t[]){ 0x00 }, 1);
    probe_write(0xDD, (const uint8_t[]){ 0x01 }, 1);
    probe_write(0xCB, (const uint8_t[]){ 0x1C }, 1);
    uint16_t dd = probe_read_param(0xDD, 1);
    uint16_t cb = probe_read_param(0xCB, 1);
    probe_write(0xD9, (const uint8_t[]){ 0x00 }, 1);
    bool is_e = probe_keys_match(dd, cb, 0x01, 0x1C);
    if (!is_e) {
        /* Not an E: confirm the C by its Read ID4, reachable once its EXTC
         * key is sent. The normalisation keeps 7 bits: 0x93 reads as 0x13. */
        probe_write(0xC8, (const uint8_t[]){ 0xFF, 0x93, 0x42 }, 3);
        uint16_t id2 = probe_read_param(0xD3, 2);
        uint16_t id3 = probe_read_param(0xD3, 3);
        probe_write(0xD9, (const uint8_t[]){ 0x00 }, 1);
        bool is_c = probe_keys_match(id2, id3, 0x93 & 0x7F, 0x42);
        ESP_LOGI(TAG, "panel probe DD=%04x CB=%04x ID4=%04x%04x -> ILI9342%s",
                 dd, cb, id2, id3, is_c ? "C" : "? (C assumed)");
    } else {
        ESP_LOGI(TAG, "panel probe DD=%04x CB=%04x -> ILI9342E", dd, cb);
    }
    return is_e;
}

static esp_err_t init(void)
{
    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");
    const i2c_device_config_t axp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &axp_cfg, &s_axp), TAG, "axp2101");
    const i2c_device_config_t aw_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AW9523_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &aw_cfg, &s_aw9523), TAG, "aw9523");

    /* M5GFX's expander setup, so the touch interrupt stays an input and the
     * port-0 pins the audio side drives push-pull. */
    ESP_RETURN_ON_ERROR(aw9523_write(AW9523_CFG_P0, 0x18), TAG, "aw9523 cfg p0");
    ESP_RETURN_ON_ERROR(aw9523_write(AW9523_CFG_P1, 0x0C), TAG, "aw9523 cfg p1");
    ESP_RETURN_ON_ERROR(aw9523_write(AW9523_GCR, 0x10), TAG, "aw9523 gcr");
    ESP_RETURN_ON_ERROR(aw9523_write(0x12, 0xFF), TAG, "aw9523 led p0");
    ESP_RETURN_ON_ERROR(aw9523_write(0x13, 0xFF), TAG, "aw9523 led p1");
    /* The SY7088 boost feeds the StackChan base (servos, LEDs); on with the
     * board, mirroring M5Unified. The LCD starts out of reset; display_start
     * pulses it properly, and the amp stays off until the speaker opens. */
    ESP_RETURN_ON_ERROR(aw9523_update(AW9523_OUT_P0, AW9523_SPK_EN, false), TAG, "amp off");
    ESP_RETURN_ON_ERROR(aw9523_update(AW9523_OUT_P1, AW9523_LCD_RST, true), TAG, "lcd rst idle");
#if CONFIG_MUSE_STACKCHAN_CAMERA
    /* The GC0308's reset is this P1_0, not an ESP32 pin (M5Unified's begin
     * releases it; esp32-camera never sees it). Held low, the camera never
     * answers on SCCB. */
    ESP_RETURN_ON_ERROR(aw9523_update(AW9523_OUT_P1, AW9523_CAM_RST, true), TAG, "cam rst release");
#endif
    ESP_RETURN_ON_ERROR(aw9523_update(AW9523_OUT_P1, AW9523_BOOST_EN, true), TAG, "boost on");

    /* M5's CoreS3 power table (M5Unified's Power_Class), so the codec rails
     * match the vendor firmware: ALDO1 1.8 V for the AW88298, ALDO2..4 at
     * 3.3 V for the ES7210, the camera and the TF slot. No keep-rails call:
     * this table is the rail setup and must not be trimmed. */
    uint8_t v;
    ESP_RETURN_ON_ERROR(axp_read(0x23, &v, 1), TAG, "axp 0x23");
    ESP_RETURN_ON_ERROR(axp_write(0x23, v & ~0x1F), TAG, "axp uvp off");
    ESP_RETURN_ON_ERROR(axp_write(0x90, 0xBF), TAG, "axp ldo enables");
    ESP_RETURN_ON_ERROR(axp_write(0x92, 13), TAG, "axp aldo1 1.8 V");
    ESP_RETURN_ON_ERROR(axp_write(0x93, 28), TAG, "axp aldo2 3.3 V");
    ESP_RETURN_ON_ERROR(axp_write(0x94, 28), TAG, "axp aldo3 3.3 V");
    ESP_RETURN_ON_ERROR(axp_write(0x95, 28), TAG, "axp aldo4 3.3 V");
    ESP_RETURN_ON_ERROR(axp_write(0x27, 0x00), TAG, "axp pwr key timing");
    ESP_RETURN_ON_ERROR(axp_write(0x69, 0x11), TAG, "axp chgled");
    ESP_RETURN_ON_ERROR(axp_write(0x10, 0x30), TAG, "axp common cfg");
    ESP_RETURN_ON_ERROR(axp_write(0x30, 0x0F), TAG, "axp adc enable");

    /* Latched power-key edges for the talk button, plus battery metering and
     * soft power-off, all from the SDK's AXP2101 support. */
    esp_err_t err = muse_pmu_init(s_i2c, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 key/power support unavailable: %s", esp_err_to_name(err));
    }

#if CONFIG_MUSE_BOARD_M5STACK_STACKCHAN
    /* Optional StackChan hardware. Neither may fail the board: the same
     * CoreS3 hardware driver also runs in the CoreS3-only build, where the
     * head base and pet sensors are not compiled in at all. */
    err = stackchan_head_init(s_i2c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no StackChan head base: %s", esp_err_to_name(err));
    }
    err = stackchan_pet_start(s_i2c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "pet sensors unavailable: %s", esp_err_to_name(err));
    }
#if CONFIG_MUSE_STACKCHAN_CAMERA
    if (stackchan_camera_prepare() != ESP_OK) {
        ESP_LOGW(TAG, "Muse camera unavailable: no memory");
    } else {
        camera_register(camera_gc0308());
    }
#endif
#endif
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    bool is_e = probe_panel_is_e();

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,   /* reads went through the probe; none after */
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_W * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }

    /* Panel reset is not a pin: it is AW9523 P1_1. */
    aw9523_update(AW9523_OUT_P1, AW9523_LCD_RST, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    aw9523_update(AW9523_OUT_P1, AW9523_LCD_RST, true);
    vTaskDelay(pdMS_TO_TICKS(120));

    const ili9341_vendor_config_t vendor = {
        .init_cmds = is_e ? s_ili9342e_cmds : s_ili9342c_cmds,
        .init_cmds_size = is_e ? sizeof(s_ili9342e_cmds) / sizeof(s_ili9342e_cmds[0])
                               : sizeof(s_ili9342c_cmds) / sizeof(s_ili9342c_cmds[0]),
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,   /* reset went through the AW9523 above */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
        .vendor_config = (void *)&vendor,
    };
    if (esp_lcd_new_panel_ili9341(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_W,
            .ver_res = LCD_H,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        return NULL;
    }

    /* Release the touch controller: pulse TOUCH_RST (AW9523 P0_0) and give
     * it its boot time, the same way the LCD reset was handled. Without
     * this the FT6336U never leaves reset and NACKs at 0x38. */
    aw9523_update(AW9523_OUT_P0, AW9523_TOUCH_RST, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    aw9523_update(AW9523_OUT_P0, AW9523_TOUCH_RST, true);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* FT5x06-family touch on the same bus. The interrupt line reaches the
     * ESP only through the AW9523, so poll it (LVGL reads every tick). */
    esp_lcd_panel_io_handle_t tp_io;
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    if (esp_lcd_new_panel_io_i2c(s_i2c, &tp_io_cfg, &tp_io) != ESP_OK) {
        return NULL;
    }
    esp_lcd_touch_handle_t tp;
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_W - 1,
        .y_max = LCD_H - 1,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
    };
    if (esp_lcd_touch_new_i2c_ft5x06(tp_io, &tp_cfg, &tp) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_touch_config_t adapter_tp = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);
    *touch = esp_lv_adapter_register_touch(&adapter_tp);
    if (!*touch || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

/* The backlight is not a PWM pin: it is the AXP2101's DLDO1 rail, and M5GFX
 * maps brightness onto its voltage (2.5 V at the bottom, 3.3 V at the top). */
static void set_brightness(int pct)
{
    if (!s_axp) {
        return;
    }
    if (pct <= 0) {
        axp_update(AXP_LDO_CTRL, 0x80, false);
        return;
    }
    uint8_t b = pct * 255 / 100;
    axp_write(AXP_DLDO1_VOLT, (b + 641) >> 5);
    axp_update(AXP_LDO_CTRL, 0x80, true);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);   /* SLPIN/SLPOUT; GRAM is kept */
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

/* The AW88298 drives the speaker, the ES7210 the two mics, both over one
 * duplex I2S bus at Muse's 16 kHz. */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");
    const i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .rx_handle = rx, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_ERR_NO_MEM, TAG, "i2s data interface");

    audio_codec_i2c_cfg_t spk_i2c = {
        .port = I2C_NUM_0, .addr = AW88298_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c,
    };
    const audio_codec_ctrl_if_t *spk_ctrl = audio_codec_new_i2c_ctrl(&spk_i2c);
    ESP_RETURN_ON_FALSE(spk_ctrl, ESP_ERR_NO_MEM, TAG, "amp control interface");
    aw88298_codec_cfg_t aw_cfg = {
        .ctrl_if = spk_ctrl,
        .gpio_if = &s_aw9523_gpio,
        /* Not a real pin: the driver's "reset" line is the amp enable, and
         * our gpio_if drives AW9523 P0_2 whatever number it is handed. */
        .reset_pin = 2,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *spk_codec = aw88298_codec_new(&aw_cfg);
    ESP_RETURN_ON_FALSE(spk_codec, ESP_FAIL, TAG, "AW88298 not responding");

    audio_codec_i2c_cfg_t mic_i2c = {
        .port = I2C_NUM_0, .addr = ES7210_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c,
    };
    const audio_codec_ctrl_if_t *mic_ctrl = audio_codec_new_i2c_ctrl(&mic_i2c);
    ESP_RETURN_ON_FALSE(mic_ctrl, ESP_ERR_NO_MEM, TAG, "mic control interface");
    es7210_codec_cfg_t es_cfg = {
        .ctrl_if = mic_ctrl,
        .master_mode = false,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2,
        .mclk_src = ES7210_MCLK_FROM_PAD,
    };
    const audio_codec_if_t *mic_codec = es7210_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(mic_codec, ESP_FAIL, TAG, "ES7210 not responding");

    esp_codec_dev_cfg_t out_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = spk_codec, .data_if = data_if,
    };
    esp_codec_dev_cfg_t in_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN, .codec_if = mic_codec, .data_if = data_if,
    };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}

/* The ES7210's gain register steps 3 dB at a time; snap like the Waveshare
 * ES7210 board does. */
static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    if (db == 33) {
        db = 34.5;
    } else {
        db = db / 3 * 3;
    }
    esp_codec_dev_set_in_gain(mic, (float)db);
}

/* The physical power key reaches only the AXP2101, never an ESP pin, so the
 * talk button is the PMU's latched key edge, polled. There is no hardware
 * aux button; the head pat is the pet task's business, not the input map. */
static unsigned poll_buttons(void)
{
    unsigned ev = 0;
    unsigned key = muse_pmu_poll_key();
    if (key & MUSE_PMU_KEY_PRESS) {
        ev |= MUSE_BTN_TALK_PRESS;
    }
    if (key & MUSE_PMU_KEY_RELEASE) {
        ev |= MUSE_BTN_TALK_RELEASE;
    }
    return ev;
}

static esp_err_t power_off(void)
{
    set_brightness(0);
#if CONFIG_MUSE_BOARD_M5STACK_STACKCHAN
    stackchan_head_set_power(false);   /* no-op without the head base */
#endif
    return muse_pmu_power_off();
}

#if CONFIG_MUSE_BOARD_M5STACK_STACKCHAN
#define BOARD_NAME "M5Stack StackChan"
#define BOARD_AUX_BUTTON "top"
#define BOARD_AUX_HINT { LV_ALIGN_TOP_MID, 0, 4 }
#else
#define BOARD_NAME "M5Stack CoreS3"
#define BOARD_AUX_BUTTON "side"
#define BOARD_AUX_HINT { LV_ALIGN_DEFAULT, 0, 0 }
#endif

static const muse_board_t s_board = {
    .name = BOARD_NAME,
    .width = LCD_W,
    .height = LCD_H,
    .round = false,
    .touch = true,
    .diagonal_in = 2.0f,
    .talk_button = "side",
    .aux_button = BOARD_AUX_BUTTON,
    /* The power key is on the left edge; the aux hint points at the head,
     * where the touch sensor lives. */
    .talk_hint = { LV_ALIGN_LEFT_MID, 4, 0 },
    .aux_hint = BOARD_AUX_HINT,
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .audio_init = audio_init,
    .mic_slot = -1,              /* two mics on the ES7210; muse_audio mixes them */
    .set_mic_gain = set_mic_gain,
    .poll_buttons = poll_buttons,
    .read_power = muse_pmu_read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
