// Copyright (c) Meta Platforms, Inc. and affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// XPT2046 resistive touch driver for the Sunton ESP32-2432S028 CYD.
//
// The touch controller is on its OWN SPI bus (VSPI), NOT the display's HSPI
// bus. This is the single most common CYD mistake: bringing the XPT2046 up on
// the display bus leaves every read returning the idle saturation value
// (0x1FFF) and touched() stuck true, because nothing answers on that bus.
//
// The panel exposes two things to the UI:
//   cyd_touch_read()  — polled by the LVGL input device
//   cyd_touch_init()  — starts the bus and the polling task

#include "cyd_touch.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "link.cyd_touch";

// Touch bus pins — fixed in the CYD's PCB copper.
#define TOUCH_HOST       SPI3_HOST
#define TOUCH_PIN_SCLK   25
#define TOUCH_PIN_MOSI   32
#define TOUCH_PIN_MISO   39
#define TOUCH_PIN_CS     33
#define TOUCH_PIN_IRQ    36
#define TOUCH_CLK_HZ     (2 * 1000 * 1000)   // XPT2046 tops out ~2.5 MHz

// Raw range measured on this unit by pressing the four corners, then padded.
// A different CYD will land slightly outside this; the clamp below keeps the
// cursor on screen either way.
#define RAW_X_MIN   500
#define RAW_X_MAX   3550
#define RAW_Y_MIN   500
#define RAW_Y_MAX   3650

#define SCREEN_W    240
#define SCREEN_H    320

// XPT2046 command bytes: start bit | channel | 12-bit | single-ended.
#define CMD_X       0xD0
#define CMD_Y       0x90

static spi_device_handle_t s_dev;
static bool s_ready;

// Last sample, shared with the LVGL read callback.
static volatile int s_x = -1;
static volatile int s_y = -1;
static volatile bool s_pressed;

static inline int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// One 12-bit conversion. The XPT2046 returns 12 bits MSB-first in three bytes
// after the command byte; the useful bits straddle bytes 1 and 2.
static uint16_t xpt_read(uint8_t cmd) {
    uint8_t tx[3] = {cmd, 0, 0};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (spi_device_polling_transmit(s_dev, &t) != ESP_OK) return 0;
    return (uint16_t)(((rx[1] << 8) | rx[2]) >> 3) & 0x0FFF;
}

// Read once. Returns true if a plausible press is on the panel.
//
// A real press pulls the touched rails away from their idle state; reading
// 0x0FFF (4095) or 0 on both axes means nothing is being driven, so treat it
// as released rather than reporting a phantom touch at a corner.
static bool xpt_sample(int *out_x, int *out_y) {
    uint16_t z1 = xpt_read(0xB0);          // pressure channel
    if (z1 < 100) return false;            // no meaningful contact

    uint16_t rx = xpt_read(CMD_X);
    uint16_t ry = xpt_read(CMD_Y);
    if (rx == 0 || ry == 0 || rx == 0x0FFF || ry == 0x0FFF) return false;

    // Map raw -> screen, clamped so a finger at the bezel still lands inside.
    int x = (int)((long)(rx - RAW_X_MIN) * SCREEN_W / (RAW_X_MAX - RAW_X_MIN));
    int y = (int)((long)(ry - RAW_Y_MIN) * SCREEN_H / (RAW_Y_MAX - RAW_Y_MIN));
    *out_x = clamp_int(x, 0, SCREEN_W - 1);
    *out_y = clamp_int(y, 0, SCREEN_H - 1);
    return true;
}

static void touch_task(void *arg) {
    (void)arg;
    for (;;) {
        // Polling the IRQ line first saves a whole SPI transaction when idle.
        // The CYD's IRQ floats on some revisions, so it gates the fast path
        // only; the sample below still decides whether a press is real.
        int x, y;
        if (gpio_get_level(TOUCH_PIN_IRQ) == 0 && xpt_sample(&x, &y)) {
            s_x = x;
            s_y = y;
            s_pressed = true;
        } else {
            s_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));     // 50 Hz is plenty for a UI
    }
}

esp_err_t cyd_touch_init(void) {
    if (s_ready) return ESP_OK;

    spi_bus_config_t bus = {
        .sclk_io_num = TOUCH_PIN_SCLK,
        .mosi_io_num = TOUCH_PIN_MOSI,
        .miso_io_num = TOUCH_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    esp_err_t err = spi_bus_initialize(TOUCH_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "touch bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = TOUCH_CLK_HZ,
        .mode = 0,                          // XPT2046 is SPI mode 0
        .spics_io_num = TOUCH_PIN_CS,
        .queue_size = 1,
    };
    err = spi_bus_add_device(TOUCH_HOST, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "touch device add failed: %s", esp_err_to_name(err));
        spi_bus_free(TOUCH_HOST);
        return err;
    }

    // IRQ is input-only on GPIO36, with no internal pull-up available.
    gpio_config_t irq = {
        .pin_bit_mask = 1ULL << TOUCH_PIN_IRQ,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&irq);

    // Sanity check: with nothing pressed the controller should answer.
    int x, y;
    ESP_LOGI(TAG, "XPT2046 on SPI3 (clk=%d mosi=%d miso=%d cs=%d irq=%d), probe=%s",
             TOUCH_PIN_SCLK, TOUCH_PIN_MOSI, TOUCH_PIN_MISO,
             TOUCH_PIN_CS, TOUCH_PIN_IRQ,
             xpt_sample(&x, &y) ? "pressed" : "idle OK");

    xTaskCreate(touch_task, "cyd_touch", 3072, NULL, 4, NULL);
    s_ready = true;
    ESP_LOGI(TAG, "CYD touch ready (240x320 XPT2046)");
    return ESP_OK;
}

bool cyd_touch_read(int *x, int *y) {
    if (!s_ready || !s_pressed) return false;
    *x = s_x;
    *y = s_y;
    return true;
}
