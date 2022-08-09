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


#include "camera.h"
#include "visualhoming_common.h"


#include "app.h"
#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "VISUALHOMING"
#include "debug.h"
#include "log.h"
#include "param.h"


#ifndef VISUALHOMING_Z
#define VISUALHOMING_Z 1.0
#endif

#ifndef VISUALHOMING_VREF
#define VISUALHOMING_VREF 1.0
#endif


static struct params_t {
  struct {
    // Bool switches
    uint8_t enable;
    uint8_t record_snapshot_single;
    uint8_t record_snapshot_sequence;
    uint8_t record_odometry;
    uint8_t record_both_single;
    uint8_t record_both_sequence;
    uint8_t record_clear;
    uint8_t follow_stay;
    uint8_t follow;
  } sw;
  float z;
  float vref;
} params;

static struct varid_t {
  logVarId_t pos_x;
  logVarId_t pos_y;
  logVarId_t pos_z;
  logVarId_t att_roll;
  logVarId_t att_pitch;
  logVarId_t att_yaw;
} varid;


///////////////////////////////////////////////////////////


void visualhoming_set_goal(float n, float e) {
  static float last_n, last_e;
  if (params.sw.enable) {
    if (n != last_n || e != last_e) {
      last_n = n;
      last_e = e;
      float dist = 0;
      float time = dist / params.vref;
      if (time < 1.0f) time = 1.0;
      crtpCommanderHighLevelGoTo(n, e, params.z, logGetFloat(varid.att_yaw), time, false);
    }
  } else {
    crtpCommanderHighLevelDisable();
  }
}

void visualhoming_position_update(float dn, float de) {

}

void visualhoming_heading_update(float dpsi) {

}

void visualhoming_log(vh_msg_t *log_msg) {
  switch (log_msg->type) {
    case VH_MSG_VECTOR:
      DEBUG_PRINT("Vector received!\n");
    default:
      break;
  }
}

struct state_t visualhoming_get_state(void) {
  struct state_t state;
  state.pos.n = logGetFloat(varid.pos_x);
  state.pos.e = logGetFloat(varid.pos_y);
  state.att.phi = 0;
  state.att.theta = 0;
  state.att.psi = logGetFloat(varid.att_yaw) / 180.0f * (float)M_PI;
  return state;
}


///////////////////////////////////////////////////////////


static void app_init(void) {
  params.z = VISUALHOMING_Z;
  params.vref = VISUALHOMING_VREF;
  varid.pos_x = logGetVarId("stateEstimate", "x");
  varid.pos_y = logGetVarId("stateEstimate", "y");
  varid.pos_z = logGetVarId("stateEstimate", "z");
  varid.att_roll = logGetVarId("stateEstimate", "roll");
  varid.att_pitch = logGetVarId("stateEstimate", "pitch");
  varid.att_yaw = logGetVarId("stateEstimate", "yaw");
  camera_init();
  visualhoming_common_init();
}


static void app_periodic(void) {
  // Handle param switches
  static enum camera_state_t state;
  if (params.sw.record_clear) {
    params.sw.record_clear = 0;
    state = RECORD_CLEAR;
  } else if (params.sw.record_snapshot_single) {
    params.sw.record_snapshot_single = 0;
    state = RECORD_SNAPSHOT;
  } else if (params.sw.record_snapshot_sequence) {
    params.sw.record_snapshot_sequence = 0;
    state = RECORD_SNAPSHOT | RECORD_SEQUENCE;
  } else if (params.sw.record_odometry) {
    params.sw.record_odometry = 0;
    state = RECORD_ODOMETRY;
  } else if (params.sw.record_both_single) {
    params.sw.record_both_single = 0;
    state = RECORD_SNAPSHOT | RECORD_ODOMETRY;
  } else if (params.sw.record_both_sequence) {
    params.sw.record_both_sequence = 0;
    state = RECORD_SNAPSHOT | RECORD_ODOMETRY | RECORD_SEQUENCE;
  } else if (params.sw.follow_stay) {
    params.sw.follow_stay = 0;
    state = FOLLOW_STAY;
  } else if (params.sw.follow) {
    params.sw.follow = 0;
    state = FOLLOW;
  }
  // Run record/follow functions
  if (state & 0x10) {
    // Follow
    visualhoming_follow(state);
  } else {
    // Record
    visualhoming_record(state);
  }
  // Run visualhoming_common
  visualhoming_common_periodic();
}


void appMain() {
  app_init();
  while (1) {
    app_periodic();
    vTaskDelay(M2T(50));
  }
}

