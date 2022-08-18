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
#include "usec_time.h"

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
  paramVarId_t kalman_reset;
} varid;


static struct params_t {
  struct {
    // Switches
    uint8_t enable;
    uint8_t kill;
    uint8_t experiment;
  } sw;
  struct {
    // Bool buttons
    uint8_t record_clear;
    uint8_t record_snapshot_single;
    uint8_t record_snapshot_sequence;
    uint8_t record_odometry;
    uint8_t record_both_single;
    uint8_t record_both_sequence;
    uint8_t follow_stay;
    uint8_t follow;
    uint8_t experiment[4];
  } btn;
  struct {
    float max_dist_from_home;
    float max_z;
    float z;
    float vref;
    float yaw_rad_sd;
    float pos_m_sd;
  } conf;
  struct {
    uint8_t force_yaw;
    uint8_t force_pos;
    uint8_t dry_run;
  } debug;
} params;


PARAM_GROUP_START(vh)
PARAM_ADD(PARAM_UINT8, SW_ENABLE, &params.sw.enable)
PARAM_ADD(PARAM_UINT8, SW_KILL, &params.sw.kill)
PARAM_ADD(PARAM_FLOAT, S_max_dist, &params.conf.max_dist_from_home)
PARAM_ADD(PARAM_FLOAT, S_max_alt, &params.conf.max_z)
PARAM_ADD(PARAM_UINT8, sw_experiment, &params.sw.experiment)
PARAM_ADD(PARAM_UINT8, btn_clear, &params.btn.record_clear)
PARAM_ADD(PARAM_UINT8, btn_ss_single, &params.btn.record_snapshot_single)
PARAM_ADD(PARAM_UINT8, btn_ss_seq, &params.btn.record_snapshot_sequence)
PARAM_ADD(PARAM_UINT8, btn_odo, &params.btn.record_odometry)
PARAM_ADD(PARAM_UINT8, btn_both_single, &params.btn.record_both_single)
PARAM_ADD(PARAM_UINT8, btn_both_seq, &params.btn.record_both_sequence)
PARAM_ADD(PARAM_UINT8, btn_follow_stay, &params.btn.follow_stay)
PARAM_ADD(PARAM_UINT8, btn_follow, &params.btn.follow)
PARAM_ADD(PARAM_FLOAT, conf_z, &params.conf.z)
PARAM_ADD(PARAM_FLOAT, conf_vref, &params.conf.vref)
PARAM_ADD(PARAM_FLOAT, conf_yaw_rad_sd, &params.conf.yaw_rad_sd)
PARAM_ADD(PARAM_FLOAT, conf_pos_m_sd, &params.conf.pos_m_sd)
PARAM_ADD(PARAM_UINT8, db_yaw, &params.debug.force_yaw)
PARAM_ADD(PARAM_UINT8, db_pos, &params.debug.force_pos)
PARAM_ADD(PARAM_UINT8, db_dryrun, &params.debug.dry_run)
PARAM_GROUP_STOP(vh)


static struct state_t state;  // Shared state buffer, to avoid repeated fetches.


///////////////////////////////////////////////////////////


static void armedGoTo(const float x, const float y, const float z, const float yaw, const float duration_s, const bool relative) {
  if (params.sw.enable) {
    crtpCommanderHighLevelGoTo(x, y, z, yaw, duration_s, relative);
  } else {
    crtpCommanderHighLevelDisable();
  }
}

static void armedTakeoff(const float absoluteHeight_m, const float duration_s) {
  if (params.sw.enable) {
    crtpCommanderHighLevelTakeoff(absoluteHeight_m, duration_s);
  } else {
    crtpCommanderHighLevelDisable();
  }
}

static void armedLand(const float absoluteHeight_m, const float duration_s) {
  if (!params.debug.dry_run) {  // Not ideal
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
  } else {
    crtpCommanderHighLevelDisable();
  }
}


///////////////////////////////////////////////////////////


void visualhoming_set_goal(float n, float e) {
  static float last_n, last_e;
  if (n != last_n || e != last_e) {
    last_n = n;
    last_e = e;
    float dn = n - state.pos.n;
    float de = e - state.pos.e;
    float dist = sqrtf(dn* dn + de * de);
    float time = dist / params.conf.vref;
    if (time < 0.1f) time = 0.1;
    armedGoTo(n, -e, params.conf.z, 0.0, time, false);
  }
}

void visualhoming_position_update(float dn, float de) {
  positionMeasurement_t pos = {
      .x = state.pos.n + dn,
      .y = -(state.pos.e + de),
      .z = state.pos.u,  // Can only provide all three axes
      .stdDev = params.conf.pos_m_sd,
      .source = 99,
  };
  estimatorEnqueuePosition(&pos);
}

void visualhoming_heading_update(float dpsi) {
  yawErrorMeasurement_t ye = {
      .yawError = dpsi / 2,
      .stdDev = params.conf.yaw_rad_sd,
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


static float dist2_to(float n, float e) {
  float dn = n - state.pos.n;
  float de = e - state.pos.e;
  return dn * dn + de * de;
}


static struct {
  uint8_t experiment;
  uint8_t block;
  uint8_t stage;
  uint64_t timer;
} experiment_state;


static void next_block(void) {
  experiment_state.block++;
  experiment_state.stage = 0;
  experiment_state.timer = 0;
}


static void experiment_idle_periodic(void) {
  // Do nothing
}

static struct pos3f_t fake_vector(float n_home, float e_home) {
  // Calculate fake homing vector
  struct pos3f_t rel_pos = {
      .n = state.pos.n - n_home,
      .e = state.pos.e - e_home
  };
  struct pos3f_t vector = {
      .n = -0.2f * rel_pos.n,
      .e = 0.2f * rel_pos.n - 0.5f * rel_pos.e,
  };
  float norm = sqrtf(vector.n * vector.n + vector.e * vector.e);
  float scale = norm < 0.2f ? 1.0f : 0.2f / norm;
  vector.n *= scale;
  vector.e *= scale;
  // Transform to global coordinates
  struct pos3f_t to = {
      .n = state.pos.n + vector.n,
      .e = state.pos.e + vector.e,
  };
  return to;
}

static void experiment_fake_homing_periodic(void) {
  switch (experiment_state.block) {
    case 0:  // Go to homing position
      visualhoming_set_goal(0, 0);
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 1:  // Go to start position
      visualhoming_set_goal(0, -2);
      if (dist2_to(0, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 2:  // Fake homing
      struct pos3f_t to = fake_vector(0, 0);
      visualhoming_set_goal(to.n, to.e);
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 10.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 3:  // Reset
      experiment_state.block = 0;
      break;
    default:
      break;
  }
}

static void experiment_single_snapshot_periodic(void) {
  switch (experiment_state.block) {
    case 0:  // Go to homing position
      visualhoming_set_goal(0, 0);
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 1:  // Take snapshot
      params.btn.record_snapshot_single = 1;
      next_block();
      break;
    case 2:  // Take snapshot (wait)
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 3:  // Go to start position
      visualhoming_set_goal(0, -2);
      if (dist2_to(0, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 4:  // Homing
      params.btn.follow = 1;
      next_block();
      break;
    case 5:  // Homing (wait)
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 10.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 6:  // Reset
      params.btn.record_clear = 1;
      experiment_state.block = 0;
      break;
    default:
      break;
  }
}


typedef void (*experiment_fn)(void);

experiment_fn experiment_periodic_fn[] = {
    experiment_idle_periodic,
    experiment_fake_homing_periodic,
    experiment_single_snapshot_periodic,
};
static const int NUM_EXPERIMENTS = sizeof(experiment_periodic_fn) / sizeof(experiment_periodic_fn[0]);


static void experiment_periodic(void) {
  if (params.sw.experiment >= 0 && params.sw.experiment < NUM_EXPERIMENTS) {
    if (params.sw.experiment != experiment_state.experiment) {
      memset(&experiment_state, 0, sizeof(experiment_state));
      experiment_state.experiment = params.sw.experiment;
    }
    experiment_periodic_fn[params.sw.experiment]();
  }
}


///////////////////////////////////////////////////////////


static void app_init(void) {
  params.sw.enable = 0;
  params.sw.kill = 0;
  params.sw.experiment = 0;

  params.conf.max_dist_from_home = VISUALHOMING_MAX_DIST_FROM_HOME;
  params.conf.max_z = VISUALHOMING_MAX_Z;
  params.conf.z = VISUALHOMING_Z;
  params.conf.vref = VISUALHOMING_VREF;
  params.conf.yaw_rad_sd = VISUALHOMING_YAW_RAD_SD;
  params.conf.pos_m_sd = VISUALHOMING_POS_M_SD;

  varid.pos_x = logGetVarId("stateEstimate", "x");
  varid.pos_y = logGetVarId("stateEstimate", "y");
  varid.pos_z = logGetVarId("stateEstimate", "z");
  varid.att_roll = logGetVarId("stateEstimate", "roll");
  varid.att_pitch = logGetVarId("stateEstimate", "pitch");
  varid.att_yaw = logGetVarId("stateEstimate", "yaw");

  varid.kalman_reset = paramGetVarId("kalman", "resetEstimation");

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
    visualhoming_position_update(pos_tgt.n - state.pos.n, pos_tgt.e - state.pos.e);
  }
}

static bool is_safe(void) {
  float dist2_home = state.pos.n * state.pos.n + state.pos.e * state.pos.e;
  float dist2_thres = params.conf.max_dist_from_home * params.conf.max_dist_from_home;

  bool safe = true;
  safe &= !params.sw.kill;
  safe &= params.sw.enable || (params.debug.dry_run && !params.sw.enable);
  safe &= dist2_home < dist2_thres;
  safe &= state.pos.u < params.conf.max_z;
  return safe;
}


static void handle_buttons(enum camera_state_t *cam_state) {
  // Handle camera state buttons
  if (params.btn.record_clear) {
    *cam_state = RECORD_CLEAR;
  } else if (params.btn.record_snapshot_single) {
    *cam_state = RECORD_SNAPSHOT;
  } else if (params.btn.record_snapshot_sequence) {
    *cam_state = RECORD_SNAPSHOT | RECORD_SEQUENCE;
  } else if (params.btn.record_odometry) {
    *cam_state = RECORD_ODOMETRY;
  } else if (params.btn.record_both_single) {
    *cam_state = RECORD_SNAPSHOT | RECORD_ODOMETRY;
  } else if (params.btn.record_both_sequence) {
    *cam_state = RECORD_SNAPSHOT | RECORD_ODOMETRY | RECORD_SEQUENCE;
  } else if (params.btn.follow_stay) {
    *cam_state = FOLLOW_STAY;
  } else if (params.btn.follow) {
    *cam_state = FOLLOW;
  }
  // Clear buttons
  memset(&params.btn, 0, sizeof(params.btn));
}


static void app_periodic(void) {
  static bool in_flight = false;
  static enum camera_state_t mode = 0x00;

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
      params.sw.experiment = 0;
      paramSetInt(varid.kalman_reset, 1);
      vTaskDelay(M2T(1000));
      armedTakeoff(params.conf.z, 1.0);
      vTaskDelay(M2T(1000));
      in_flight = true;
    }
  } else { // in_flight
    if (!is_safe()) {
      armedLand(0.0, 1.0);
      vTaskDelay(M2T(1500));
      crtpCommanderHighLevelStop();
      params.sw.enable = 0;
      in_flight = false;
    } else { // is_safe
      handle_buttons(&mode);
      // Run pre-programmed experiments
      experiment_periodic();
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

