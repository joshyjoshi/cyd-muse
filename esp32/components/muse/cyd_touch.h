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

// XPT2046 touch for the CYD, on its own SPI bus. See cyd_touch.c.

#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Brings up the touch SPI bus and starts the polling task. Safe to call twice.
esp_err_t cyd_touch_init(void);

// Latest touch sample in screen coordinates (240x320, portrait). Returns false
// when nothing is pressed. Does not block.
bool cyd_touch_read(int *x, int *y);

#ifdef __cplusplus
}
#endif
