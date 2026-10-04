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
 * Sunton ESP32-2432S028 "Cheap Yellow Display" (CYD), 2-USB variant:
 * ESP32-D0WD-V3 (classic ESP32, 4 MB flash, NO PSRAM), 2.8" 240x320 ILI9341
 * SPI LCD, XPT2046 resistive touch on its own SPI bus, single BOOT button on
 * GPIO0, discrete active-LOW RGB LED, CH340 USB-UART bridge. No microphone,
 * no speaker, no PMU. Pins from the board silkscreen and the community
 * pinout (witnessmenow/ESP32-Cheap-Yellow-Display), verified on hardware.
 *
 * Two things make this board unusual for the SDK:
 *
 * 1. NO PSRAM, 4 MB flash. Every other full-UI board has at least 8 MB of
 *    PSRAM and 8 MB of flash. There is no room for the home-network tunnel
 *    and no PSRAM for the voice task's stack, so voice is compiled out (see
 *    CONFIG_HOMEHUB_VOICE below) and the UI runs on internal RAM alone:
 *    single-buffered, few draw lines, no image path beyond the avatar.
 *
 * 2. The touch controller is on its OWN SPI bus (VSPI: CLK 25 / MOSI 32 /
 *    MISO 39 / CS 33), not the panel's (HSPI: SCLK 14 / MOSI 13 / MISO 12 /
 *    CS 15 / DC 2). cyd_touch.c owns that bus; this file only registers its
 *    coordinates as an LVGL input device.
 *
 * Touch is the only input besides BOOT, so push-to-talk follows the Core2's
 * pattern: a touch zone at the bottom of the screen is the talk button, and
 * BOOT is the aux button.
 *
 * There is no microphone or codec on the board, so audio_init() reports
 * failure and Muse runs text-only. See the note on voice in init().
 */
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cyd_touch.h"
#include "muse_board.h"
#include "muse_input.h"
#include "muse_lcd_bands.h"
#include "muse_mem.h"

static const char *TAG = "board.cyd";

/* Panel pins — the CYD's HSPI (SPI2) bus. */
#define LCD_HOST        SPI2_HOST
#define LCD_PIN_SCLK    14
#define LCD_PIN_MOSI    13
#define LCD_PIN_MISO    12
#define LCD_PIN_CS      15
#define LCD_PIN_DC      2
/* RST is tied to the board's EN line, so there is no reset GPIO. */
#define LCD_PIN_RST     -1
#define LCD_PIN_BL      21
#define LCD_H_RES       240
#define LCD_V_RES       320
#define LCD_PCLK_HZ     (40 * 1000 * 1000)

/* No PSRAM, and by the time the UI starts Wi-Fi has already taken the large
 * internal blocks: the largest free DMA-capable run is a few KB. 8 lines at
 * 240 px RGB565 is 3.8 KB per buffer, which still fits after Wi-Fi joins.
 * The bands path sends each buffer as it fills, so more, smaller buffers cost
 * a little overhead but no tearing. */
#define DRAW_BUF_LINES  8
#define LCD_CHUNK_BYTES (LCD_H_RES * DRAW_BUF_LINES * 2)

/* BOOT button. */
#define BTN_BOOT        0

/* Backlight is plain PWM on the CYD, not a PMU rail. */
#define BL_LEDC_CH      LEDC_CHANNEL_0
#define BL_LEDC_TIMER   LEDC_TIMER_0

static esp_lcd_panel_io_handle_t s_io;
static lv_display_t *s_disp;
static lv_indev_t *s_touch;
static int s_brightness = 100;

static void set_brightness(int pct);

static esp_err_t backlight_setup(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "backlight timer");

    ledc_channel_config_t ch = {
        .gpio_num = LCD_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_LEDC_CH,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 1023,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "backlight channel");
    return ESP_OK;
}

static esp_err_t panel_new(esp_lcd_panel_handle_t *out)
{
    spi_bus_config_t bus = {
        .sclk_io_num = LCD_PIN_SCLK,
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = LCD_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_CHUNK_BYTES,
    };
    /* The touch controller owns VSPI, so the panel gets its own bus (SPI2). */
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "panel bus");

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                                 &io_cfg, &s_io),
                        TAG, "panel io");

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(s_io, &panel_cfg, out),
                        TAG, "ili9341");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*out), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*out), TAG, "panel init");
    /* No colour inversion: this panel renders correctly without it, verified
     * on hardware with TFT_eSPI. */
    esp_lcd_panel_set_gap(*out, 0, 0);
    esp_lcd_panel_mirror(*out, false, false);
    esp_lcd_panel_disp_on_off(*out, true);
    return ESP_OK;
}

/* LVGL input read callback: XPT2046 coordinates, converted to LVGL's pointer
 * state. cyd_touch_read() reports screen-space pixels already. */
static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    int x, y;
    if (cyd_touch_read(&x, &y)) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    esp_lcd_panel_handle_t panel;
    if (panel_new(&panel) != ESP_OK) {
        ESP_LOGE(TAG, "panel init failed");
        return NULL;
    }

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }

    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = panel,
        .panel_io = s_io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_H_RES,
            .ver_res = LCD_V_RES,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    s_disp = muse_lcd_bands_register(disp_cfg, DRAW_BUF_LINES, LCD_CHUNK_BYTES);
    if (!s_disp) {
        ESP_LOGE(TAG, "lvgl display registration failed");
        return NULL;
    }

    /* Touch: the driver brings up VSPI, then LVGL polls it. */
    if (cyd_touch_init() != ESP_OK) {
        ESP_LOGW(TAG, "no touch; BOOT only");
        *touch = NULL;
    } else {
        s_touch = lv_indev_create();
        lv_indev_set_type(s_touch, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(s_touch, touch_read);
        lv_indev_set_display(s_touch, s_disp);
        *touch = s_touch;
    }

    if (esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return s_disp;
}

static bool display_lock(int timeout_ms)
{
    /* Muse uses -1 for forever. */
    return esp_lv_adapter_lock(timeout_ms < 0 ? -1 : timeout_ms);
}

static esp_err_t init(void)
{
    ESP_RETURN_ON_ERROR(backlight_setup(), TAG, "backlight");
    set_brightness(100);

    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BTN_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    /* No microphone and no speaker amp populated on this board, so audio_init
     * is left NULL. Muse runs text-only: push-to-talk submits nothing and the
     * UI hides the level meter (muse_ui checks the board's mic support). */
    ESP_LOGI(TAG, "CYD ready: 240x320 ILI9341, XPT2046 touch, 4 MB, no PSRAM");
    return ESP_OK;
}

static void set_brightness(int pct)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    s_brightness = pct;
    /* 10-bit duty; 0 is fully dark, which is legal here because the backlight
     * is a plain GPIO PWM rather than a PMU rail. */
    uint32_t duty = (uint32_t)((1023 * pct) / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH);
}

/* BOOT on GPIO0 is the aux button: tap cycles brightness, hold 5 s is handled
 * by Home Link's own long-press logic when the UI is down. */
#define BTN_SAMPLE_MS 10
#define AUX_TAP_MS    40
#define AUX_HOLD_MS   1200

static unsigned poll_buttons(void)
{
    static bool last;
    static uint32_t held_ms;
    static bool long_sent;
    unsigned events = 0;

    bool now = gpio_get_level(BTN_BOOT) == 0;   /* active low */
    if (now == last) {
        if (now) {
            held_ms += BTN_SAMPLE_MS;
            if (!long_sent && held_ms >= AUX_HOLD_MS) {
                events |= MUSE_BTN_AUX_PRESS;
                long_sent = true;
            }
        }
        return events;
    }
    last = now;
    if (now) {
        held_ms = 0;
        long_sent = false;
    } else {
        if (!long_sent && held_ms >= AUX_TAP_MS) {
            events |= MUSE_BTN_AUX_PRESS | MUSE_BTN_AUX_RELEASE;
        } else if (long_sent) {
            events |= MUSE_BTN_AUX_RELEASE;
        }
        held_ms = 0;
        long_sent = false;
    }
    return events;
}

static const muse_board_t s_board = {
    .name = "CYD 2432S028",
    .width = LCD_H_RES,
    .height = LCD_V_RES,
    .round = false,
    .touch = true,
    .diagonal_in = 2.8f,
    .talk_button = "touch",     /* touch zone at the bottom of the screen */
    .aux_button = "side",       /* BOOT, on the right edge */
    .talk_hint = { LV_ALIGN_BOTTOM_MID, 0, -30 },
    .aux_hint = { LV_ALIGN_RIGHT_MID, -4, 0 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .audio_init = NULL,         /* no codec on the board */
    .poll_buttons = poll_buttons,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
