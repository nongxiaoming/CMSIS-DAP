/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
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
 
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

// Each board should have a unique ID and secret. For information 
//  about obtaining a secret contact support@mbed.org
#include "device_cfg.h"

typedef struct {
    const uint8_t  id[5];
    const uint8_t  secret[9];
} BOARD;

extern BOARD board;

#endif