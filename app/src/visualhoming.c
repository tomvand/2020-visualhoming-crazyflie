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
#include "estimator.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "VISUALHOMING"
#include "debug.h"
#include "log.h"
#include "param.h"


#ifndef VISUALHOMING_Z
#define VISUALHOMING_Z 0.5
#endif

#ifndef VISUALHOMING_MAX_Z
#define VISUALHOMING_MAX_Z 1.0
#endif

#ifndef VISUALHOMING_VREF
#define VISUALHOMING_VREF 1.0
#endif

#ifndef VISUALHOMING_MAX_DIST_FROM_HOME
#define VISUALHOMING_MAX_DIST_FROM_HOME 4.0
#endif


#ifndef VISUALHOMING_YAW_RAD_SD
#define VISUALHOMING_YAW_RAD_SD 0.17 // approx 10deg
#endif

#ifndef VISUALHOMING_POS_M_SD
#define VISUALHOMING_POS_M_SD 0.50
#endif


static struct varid_t {
  logVarId_t pos_x;
  logVarId_t pos_y;
  logVarId_t pos_z;
  logVarId_t att_roll;
  logVarId_t att_pitch;
  logVarId_t att_yaw;
} varid;


static struct params_t {
  struct {
    // Bool switches
    uint8_t enable;
    uint8_t kill;
    uint8_t record_clear;
    uint8_t record_snapshot_single;
    uint8_t record_snapshot_sequence;
    uint8_t record_odometry;
    uint8_t record_both_single;
    uint8_t record_both_sequence;
    uint8_t follow_stay;
    uint8_t follow;
  } sw;
  float z;
  float vref;
  float yaw_rad_sd;
  float pos_m_sd;
  struct {
    float max_dist_from_home;
    float max_z;
  } safety;
  struct {
    uint8_t force_yaw;
    uint8_t force_pos;
  } debug;
} params;


PARAM_GROUP_START(vh)
PARAM_ADD(PARAM_UINT8,  SW_ENABLE, &params.sw.enable)
PARAM_ADD(PARAM_UINT8, SW_KILL, &params.sw.kill)
PARAM_ADD(PARAM_UINT8, sw_clear, &params.sw.record_clear)
PARAM_ADD(PARAM_UINT8, sw_ss_single, &params.sw.record_snapshot_single)
PARAM_ADD(PARAM_UINT8, sw_ss_seq, &params.sw.record_snapshot_sequence)
PARAM_ADD(PARAM_UINT8, sw_odo, &params.sw.record_odometry)
PARAM_ADD(PARAM_UINT8, sw_both_single, &params.sw.record_both_single)
PARAM_ADD(PARAM_UINT8, sw_both_seq, &params.sw.record_both_sequence)
PARAM_ADD(PARAM_UINT8, sw_follow_stay, &params.sw.follow_stay)
PARAM_ADD(PARAM_UINT8, sw_follow, &params.sw.follow)
PARAM_ADD(PARAM_UINT8, z, &params.z)
PARAM_ADD(PARAM_UINT8, vref, &params.vref)
PARAM_ADD(PARAM_UINT8, yaw_rad_sd, &params.yaw_rad_sd)
PARAM_ADD(PARAM_UINT8, pos_m_sd, &params.pos_m_sd)
PARAM_ADD(PARAM_UINT8, S_max_dist, &params.safety.max_dist_from_home)
PARAM_ADD(PARAM_UINT8, S_max_alt, &params.safety.max_z)
PARAM_ADD(PARAM_UINT8, db_yaw, &params.debug.force_yaw)
PARAM_ADD(PARAM_UINT8, db_pos, &params.debug.force_pos)
PARAM_GROUP_STOP(vh)


static struct state_t state;  // Shared state buffer, to avoid repeated fetches.


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
  positionMeasurement_t pos = {
      .x = state.pos.n + dn,
      .y = -(state.pos.e + de),
      .z = state.pos.u,  // Can only provide all three axes
      .stdDev = params.pos_m_sd,
      .source = 99,
  };
  estimatorEnqueuePosition(&pos);
}

void visualhoming_heading_update(float dpsi) {
  yawErrorMeasurement_t ye = {
      .yawError = dpsi / 2,
      .stdDev = params.yaw_rad_sd,
  };
  estimatorEnqueueYawError(&ye);
}

void visualhoming_log(vh_msg_t *log_msg) {
  switch (log_msg->type) {
    case VH_MSG_VECTOR:
      DEBUG_PRINT("Vector received!\n");
      break;
    default:
      break;
  }
}

struct state_t visualhoming_get_state(void) {
  // Note: pos and att returned in NED frame!
  struct state_t state;
  state.pos.n = logGetFloat(varid.pos_x);
  state.pos.e = -logGetFloat(varid.pos_y);
  state.pos.u = logGetFloat(varid.pos_z);
  state.att.phi = 0;
  state.att.theta = 0;
  state.att.psi = -logGetFloat(varid.att_yaw) / 180.0f * (float)M_PI;
  return state;
}


///////////////////////////////////////////////////////////


static void app_init(void) {
  params.sw.enable = 0;

  params.z = VISUALHOMING_Z;
  params.vref = VISUALHOMING_VREF;
  params.yaw_rad_sd = VISUALHOMING_YAW_RAD_SD;
  params.pos_m_sd = VISUALHOMING_POS_M_SD;

  params.safety.max_dist_from_home = VISUALHOMING_MAX_DIST_FROM_HOME;
  params.safety.max_z = VISUALHOMING_MAX_Z;

  varid.pos_x = logGetVarId("stateEstimate", "x");
  varid.pos_y = logGetVarId("stateEstimate", "y");
  varid.pos_z = logGetVarId("stateEstimate", "z");
  varid.att_roll = logGetVarId("stateEstimate", "roll");
  varid.att_pitch = logGetVarId("stateEstimate", "pitch");
  varid.att_yaw = logGetVarId("stateEstimate", "yaw");

  camera_init();
  visualhoming_common_init();
}


static void app_debug_periodic(void) {
  if (params.debug.force_yaw) {
    float psi_tgt = radians(20.0); // NED; -20.0deg in cfclient
    float dpsi = psi_tgt - state.att.psi;
    visualhoming_heading_update(dpsi);
  }
  if (params.debug.force_pos) {
    struct pos3f_t pos_tgt = {
        .n = 1.0,
        .e = 2.0,
    };
    visualhoming_position_update(pos_tgt.n - state.pos.n, pos_tgt.e  -state.pos.e);
  }
}

static bool is_safe(void) {
  float dist2_home = state.pos.n * state.pos.n + state.pos.e * state.pos.e;
  float dist2_thres = params.safety.max_dist_from_home * params.safety.max_dist_from_home;

  bool safe = true;
  safe &= params.sw.enable;
  safe &= dist2_home < dist2_thres;
  safe &= state.pos.u < params.safety.max_z;
  return safe;
}


static enum camera_state_t get_camera_mode(void) {
  // Handle param switches
  static enum camera_state_t mode;
  if (params.sw.record_clear) {
    params.sw.record_clear = 0;
    mode = RECORD_CLEAR;
  } else if (params.sw.record_snapshot_single) {
    params.sw.record_snapshot_single = 0;
    mode = RECORD_SNAPSHOT;
  } else if (params.sw.record_snapshot_sequence) {
    params.sw.record_snapshot_sequence = 0;
    mode = RECORD_SNAPSHOT | RECORD_SEQUENCE;
  } else if (params.sw.record_odometry) {
    params.sw.record_odometry = 0;
    mode = RECORD_ODOMETRY;
  } else if (params.sw.record_both_single) {
    params.sw.record_both_single = 0;
    mode = RECORD_SNAPSHOT | RECORD_ODOMETRY;
  } else if (params.sw.record_both_sequence) {
    params.sw.record_both_sequence = 0;
    mode = RECORD_SNAPSHOT | RECORD_ODOMETRY | RECORD_SEQUENCE;
  } else if (params.sw.follow_stay) {
    params.sw.follow_stay = 0;
    mode = FOLLOW_STAY;
  } else if (params.sw.follow) {
    params.sw.follow = 0;
    mode = FOLLOW;
  }
  return mode;
}


static void app_periodic(void) {
  static bool in_flight = false;

  // Kill switch
  if (params.sw.kill) {
    crtpCommanderHighLevelStop();
    in_flight = false;
    params.sw.enable = 0;
    return;
  }

  // Get drone state
  state = visualhoming_get_state();

  // Flight control
  if (!in_flight) {
    if (is_safe()) { // includes 'enable' switch
      // TODO reset kalman
      crtpCommanderHighLevelTakeoff(params.z, 1.0);
      vTaskDelay(M2T(1000));
      in_flight = true;
    }
  } else { // in_flight
    if (!is_safe()) {
      crtpCommanderHighLevelLand(0.0, 1.0);
      vTaskDelay(M2T(1500));
      crtpCommanderHighLevelStop();
      params.sw.enable = 0;
      in_flight = false;
    } else { // is_safe
      enum camera_state_t mode = get_camera_mode();
      // Run record/follow functions
      if (mode & 0x10) {
        visualhoming_follow(mode);
      } else {
        visualhoming_record(mode);
      }
      visualhoming_common_periodic();
    }
  }
}


void appMain() {
  app_init();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = M2T(100);
  while (1) {
    app_periodic();
    app_debug_periodic();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

