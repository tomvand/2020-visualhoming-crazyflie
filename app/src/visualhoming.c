/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "VISUALHOMING"
#include "debug.h"
#include "log.h"
#include "param.h"


static int8_t start;


void appMain() {
  while (!start) {
    DEBUG_PRINT("Waiting for start command...\n");
    vTaskDelay(M2T(1000));
  }
  DEBUG_PRINT("Taking off...\n");
  crtpCommanderHighLevelTakeoff(1.0f, 2.0f);
  vTaskDelay(M2T(2000));
  DEBUG_PRINT("Hovering...\n");
  crtpCommanderHighLevelGoTo(0, 0, 0, 0, 2.0, true);
  vTaskDelay(M2T(2000));
  DEBUG_PRINT("Landing...\n");
  crtpCommanderHighLevelLand(0, 2.0);
  vTaskDelay(M2T(2000));

  while(1) {
    DEBUG_PRINT("Landed!\n");
    vTaskDelay(M2T(2000));
  }
}


PARAM_GROUP_START(visualhoming)
PARAM_ADD(PARAM_INT8, start, &start)
PARAM_GROUP_STOP(visualhoming)
