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
#include "commander.h"
#include "estimator.h"
#include "usec_time.h"
#include "crtp.h"
#include "pm.h"

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

#ifndef VISUALHOMING_P_GAIN
#define VISUALHOMING_P_GAIN 1.0  // (m/s)/m
#endif

#ifndef VISUALHOMING_MAX_DIST_FROM_HOME
#define VISUALHOMING_MAX_DIST_FROM_HOME 8.0
#endif


#ifndef VISUALHOMING_YAW_RAD_SD
#define VISUALHOMING_YAW_RAD_SD 0.001
#endif

#ifndef VISUALHOMING_POS_M_SD
#define VISUALHOMING_POS_M_SD 0.001
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
    uint8_t idle;
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
    float p_gain;
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
PARAM_ADD(PARAM_UINT8, btn_idle, &params.btn.idle)
PARAM_ADD(PARAM_UINT8, btn_ss_single, &params.btn.record_snapshot_single)
PARAM_ADD(PARAM_UINT8, btn_ss_seq, &params.btn.record_snapshot_sequence)
PARAM_ADD(PARAM_UINT8, btn_odo, &params.btn.record_odometry)
PARAM_ADD(PARAM_UINT8, btn_both_single, &params.btn.record_both_single)
PARAM_ADD(PARAM_UINT8, btn_both_seq, &params.btn.record_both_sequence)
PARAM_ADD(PARAM_UINT8, btn_follow_stay, &params.btn.follow_stay)
PARAM_ADD(PARAM_UINT8, btn_follow, &params.btn.follow)
PARAM_ADD(PARAM_FLOAT, conf_z, &params.conf.z)
PARAM_ADD(PARAM_FLOAT, conf_vref, &params.conf.vref)
PARAM_ADD(PARAM_FLOAT, conf_p_gain, &params.conf.p_gain)
PARAM_ADD(PARAM_FLOAT, conf_yaw_rad_sd, &params.conf.yaw_rad_sd)
PARAM_ADD(PARAM_FLOAT, conf_pos_m_sd, &params.conf.pos_m_sd)
PARAM_ADD(PARAM_UINT8, db_yaw, &params.debug.force_yaw)
PARAM_ADD(PARAM_UINT8, db_pos, &params.debug.force_pos)
PARAM_ADD(PARAM_UINT8, db_dryrun, &params.debug.dry_run)
PARAM_GROUP_STOP(vh)


static struct state_t state;  // Shared state buffer, to avoid repeated fetches.


///////////////////////////////////////////////////////////


static struct {
  bool low_level;
  struct {
    float x;
    float y;
    float z;
  } setpoint;
} control;



static void armedGoTo(const float x, const float y, const float z, const float yaw, const float duration_s, const bool relative) {
  if (params.sw.enable) {
    control.low_level = true;
    control.setpoint.x = x;
    control.setpoint.y = y;
    control.setpoint.z = z;
  } else {
    crtpCommanderHighLevelDisable();
  }
}

static void armedTakeoff(const float absoluteHeight_m, const float duration_s) {
  if (params.sw.enable) {
    if (control.low_level) {
      commanderRelaxPriority();
      control.low_level = false;
    }
    crtpCommanderHighLevelTakeoff(absoluteHeight_m, duration_s);
  } else {
    crtpCommanderHighLevelDisable();
  }
}

static void armedLand(const float absoluteHeight_m, const float duration_s) {
  if (!params.debug.dry_run) {  // Not ideal
    if (control.low_level) {
      commanderRelaxPriority();
      control.low_level = false;
    }
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
  } else {
    crtpCommanderHighLevelDisable();
  }
}


static void control_periodic(void) {
  if (control.low_level) {
    static setpoint_t setpoint;  // Initialized 0
    setpoint.mode.x = modeAbs;
    setpoint.mode.y = modeAbs;
    setpoint.mode.z = modeAbs;
    setpoint.position.x = control.setpoint.x;
    setpoint.position.y = control.setpoint.y;
    setpoint.position.z = control.setpoint.z;
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
  }
}


///////////////////////////////////////////////////////////


static struct {
  struct msg_vector_t vector;
  struct msg_ins_correction_t ins_correction;
  struct msg_map_t map;
  struct pos2f_t pos;
  struct {
    struct pos2f_t pos;
    uint8_t experiment;
    uint8_t point;
    uint8_t block;
  } point;
} log_buffer;

LOG_GROUP_START(vh)
LOG_ADD(LOG_FLOAT, v_from_e, &log_buffer.vector.from.e)
LOG_ADD(LOG_FLOAT, v_from_n, &log_buffer.vector.from.n)
LOG_ADD(LOG_FLOAT, v_to_e, &log_buffer.vector.to.e)
LOG_ADD(LOG_FLOAT, v_to_n, &log_buffer.vector.to.n)
LOG_ADD(LOG_FLOAT, v_pos_e, &log_buffer.pos.e)
LOG_ADD(LOG_FLOAT, v_pos_n, &log_buffer.pos.n)
LOG_ADD(LOG_FLOAT, v_dpsi, &log_buffer.vector.delta_psi)
LOG_ADD(LOG_UINT8, v_source, &log_buffer.vector.source)
LOG_ADD(LOG_UINT8, m_ss_idx, &log_buffer.map.snapshot_idx)
LOG_ADD(LOG_FLOAT, m_ss_e, &log_buffer.map.snapshot_pos.e)
LOG_ADD(LOG_FLOAT, m_ss_n, &log_buffer.map.snapshot_pos.n)
LOG_ADD(LOG_UINT8, m_odo_idx, &log_buffer.map.odometry_idx)
LOG_ADD(LOG_FLOAT, m_odo_e, &log_buffer.map.odometry_pos.e)
LOG_ADD(LOG_FLOAT, m_odo_n, &log_buffer.map.odometry_pos.n)
LOG_ADD(LOG_UINT8, i_ss_idx, &log_buffer.ins_correction.idx)
LOG_ADD(LOG_FLOAT, i_from_e, &log_buffer.ins_correction.from.e)
LOG_ADD(LOG_FLOAT, i_from_n, &log_buffer.ins_correction.from.n)
LOG_ADD(LOG_FLOAT, i_from_psi, &log_buffer.ins_correction.psi_from)
LOG_ADD(LOG_FLOAT, i_to_e, &log_buffer.ins_correction.to.e)
LOG_ADD(LOG_FLOAT, i_to_n, &log_buffer.ins_correction.to.n)
LOG_ADD(LOG_FLOAT, i_to_psi, &log_buffer.ins_correction.psi_to)
LOG_ADD(LOG_FLOAT, p_e, &log_buffer.point.pos.e)
LOG_ADD(LOG_FLOAT, p_n, &log_buffer.point.pos.n)
LOG_ADD(LOG_UINT8, p_exp, &log_buffer.point.experiment)
LOG_ADD(LOG_UINT8, p_pt, &log_buffer.point.point)
LOG_ADD(LOG_UINT8, p_block, &log_buffer.point.block)
LOG_GROUP_STOP(vh)


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
      .yawError = dpsi,
      .stdDev = params.conf.yaw_rad_sd,
  };
  estimatorEnqueueYawError(&ye);
}

void visualhoming_log(vh_msg_t *log_msg) {
  switch (log_msg->type) {
    case VH_MSG_VECTOR:
      log_buffer.vector = log_msg->vector;
      log_buffer.pos.e = state.pos.e;
      log_buffer.pos.n = state.pos.n;
      break;
    case VH_MSG_INS_CORRECTION:
      log_buffer.ins_correction = log_msg->ins_correction;
      break;
    case VH_MSG_MAP:
      log_buffer.map = log_msg->map;
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


static void log_point(uint8_t experiment, uint8_t point) {
  log_buffer.point.experiment = experiment;
  log_buffer.point.point = point;
  log_buffer.point.pos.e = state.pos.e;
  log_buffer.point.pos.n = state.pos.n;
}


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


#define MOVE_TO_AND_WAIT(_n, _e, _radius, _time) \
  visualhoming_set_goal((_n), (_e)); \
  if (dist2_to((_n), (_e)) > (float)(_radius) * (float)(_radius)) break; \
  if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + (float)(_time) * (float)1.0e6; \
  if (usecTimestamp() < experiment_state.timer) break;

#define WAIT(_time) \
    if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + (float)(_time) * (float)1.0e6; \
    if (usecTimestamp() < experiment_state.timer) break;

static void experiment_single_snapshot_periodic(void) {
  static int8_t start_pos;
  float start_n, start_e;

  switch (experiment_state.block) {
    case 0:  // Go to homing position
      start_pos = 0;
      MOVE_TO_AND_WAIT(0, 0, 0.30, 1.0);
      next_block();
      break;
    case 1:  // Take snapshot (start)
      params.btn.record_snapshot_single = 1;
      log_point(0, 1);
      next_block();
      break;
    case 2:  // Take snapshot (wait)
      WAIT(1.0);
      next_block();
      break;
    case 3:  // Go to start position
      start_n = (start_pos % 5) - 2;
      start_e = (start_pos / 5) - 2;
      MOVE_TO_AND_WAIT(start_n, start_e, 0.30, 1.0);
      log_point(start_pos + 1, 1);
      next_block();
      break;
    case 4:  // Homing (start)
      params.btn.follow = 1;
      next_block();
      break;
    case 5:  // Homing (wait)
      WAIT(10.0)
      log_point(start_pos + 1, 2);
      params.btn.idle = 1;
      start_pos++;
      next_block();
      experiment_state.block = 3;
      break;
    default:
      break;
  }
}

static void experiment_odometry_periodic(void) {
  switch (experiment_state.block) {
    case 0:  // Go to homing position
      visualhoming_set_goal(0, 0);
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 1:  // Start recording
      params.btn.record_odometry = 1;
      next_block();
      break;
    case 2:  // Wait for recording to start
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 3:  // Go to position 1
      visualhoming_set_goal(0, -2);
      if (dist2_to(0, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 4:  // Go to position 2
      visualhoming_set_goal(2, -2);
      if (dist2_to(2, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 5:  // Go to position 3
      visualhoming_set_goal(2, 2);
      if (dist2_to(2, 2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 6:  // Go to position 4
      visualhoming_set_goal(0, 2);
      if (dist2_to(0, 2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 7:  // Homing
      params.btn.follow = 1;
      next_block();
      break;
    case 8:  // Wait for arrival
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 9:  // Reset
      params.btn.record_clear = 1;
      experiment_state.block = 0;
      break;
    default:
      break;
  }
}

static void experiment_both_sequence_periodic(void) {
  switch (experiment_state.block) {
    case 0:  // Go to homing position
      visualhoming_set_goal(0, 0);
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 1:  // Start recording
      params.btn.record_both_sequence = 1;
      next_block();
      break;
    case 2:  // Wait for recording to start
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 3:  // Go to position 1
      visualhoming_set_goal(0, -2);
      if (dist2_to(0, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 4:  // Go to position 2
      visualhoming_set_goal(2, -2);
      if (dist2_to(2, -2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 5:  // Go to position 3
      visualhoming_set_goal(2, 2);
      if (dist2_to(2, 2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 6:  // Go to position 4
      visualhoming_set_goal(0, 2);
      if (dist2_to(0, 2) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 7:  // Homing
      params.btn.follow = 1;
      next_block();
      break;
    case 8:  // Wait for arrival
      if (dist2_to(0, 0) > 0.30f * 0.30f) break;
      if (experiment_state.timer == 0) experiment_state.timer = usecTimestamp() + 1.0e6;
      if (usecTimestamp() > experiment_state.timer) next_block();
      break;
    case 9:  // Reset
      params.btn.record_clear = 1;
      experiment_state.block = 0;
      break;
    default:
      break;
  }
}

static void experiment_snapshot_distance_periodic(void) {
  static struct pos2f_t ss_pos;
  static float ss_psi;
  static uint8_t run_idx;
  switch (experiment_state.block) {
    case 0:  // Take snapshot at start (move, btn)
      run_idx = 0;
      MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0)
      params.btn.record_snapshot_single = 1;
      // log snapshot pose for manual realignment
      ss_pos.n = state.pos.n;
      ss_pos.e = state.pos.e;
      ss_psi = state.att.psi;
      log_point(0, 1);
      next_block();
      break;
    case 1:  // Take snapshot at start (wait)
      WAIT(1.0)
      next_block();
      break;
    case 2:  // Move forward for heading alginment
      MOVE_TO_AND_WAIT(2, 0, 0.3, 1.0)
      log_point(0, 2);
      next_block();
      break;
    case 3:  // Home to start position (move, btn)
      MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0)
      params.btn.follow = 1;
      next_block();
      break;
    case 4:  // Home to start position (wait)
      WAIT(2.0)
      run_idx++;
      log_point(run_idx, 1);  // Homing accuracy
      params.btn.idle = 1;
      /* XXX Manually re-align INS */
      visualhoming_position_update(ss_pos.n - state.pos.n, ss_pos.e - state.pos.e);
      visualhoming_heading_update(-log_buffer.vector.delta_psi);
      next_block();
      break;
    case 5:  // Move forwards
      MOVE_TO_AND_WAIT((run_idx - 1) % 5 + 1, 0, 0.3, 0.5);
      log_point(run_idx, 2);  // Homing + odo accuracy
      next_block();
      experiment_state.block = 3;  // Loop
      break;
    default:
      break;
  }
}

void experiment_ins_correction_yaw(void) {
  switch (experiment_state.block) {
    case 0:  // Take snapshot (btn)
      MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0);
      params.btn.record_snapshot_single = 1;
      next_block();
      break;
    case 1:  // Take snapshot (wait), follow (btn)
      WAIT(2.0);
      params.btn.follow = 1;
      next_block();
      break;
    case 2:  // Follow (wait)
      WAIT(2.0);
      next_block();
      break;
    case 3:  // Incorrectly update heading
      visualhoming_heading_update(-0.80f);  // DRONE SHOULD YAW RIGHT!
      next_block();
      break;
    case 4:  // Wait
      WAIT(5.0);
      next_block();
      break;
    case 5:  // Re-align INS
      // Note: a vector to turn right means the true yaw is more to the left, hence the '-' sign.
      visualhoming_heading_update(-log_buffer.vector.delta_psi);  // DRONE SHOULD RESTORE HEADING!
      next_block();
      break;
    case 6:  // Wait
      WAIT(5.0);
      next_block();
      experiment_state.block = 3;
      break;
    default:
      break;
  }
}

void experiment_u_both(void) {
  switch (experiment_state.block) {
      case 0:  // Take snapshot (btn)
        MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0);
        params.btn.record_both_sequence = 1;
        next_block();
        break;
      case 1:  // Take snapshot (wait)
        WAIT(2.0);
        next_block();
        break;
      case 2:  // Go to bottom left
        MOVE_TO_AND_WAIT(-5, 0, 0.3, 1.0);
        next_block();
        break;
      case 3:  // Go to bottom right
        MOVE_TO_AND_WAIT(-5, 4, 0.3, 1.0);
        next_block();
        break;
      case 4:  // Go to top right
        MOVE_TO_AND_WAIT(0, 4, 0.3, 1.0);
        next_block();
        break;
      case 5:  // Homing
        params.btn.follow = 1;
        next_block();
        break;
      case 6:  // Wait for arrival
        if (dist2_to(0, 0) > 0.30f * 0.30f) break;
        WAIT(5.0);
        next_block();
        break;
      case 7:  // Reset
        params.btn.record_clear = 1;
        experiment_state.block = 0;
        break;
    }
}

void experiment_u_odo(void) {
  switch (experiment_state.block) {
      case 0:  // Take snapshot (btn)
        MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0);
        params.btn.record_odometry = 1;
        next_block();
        break;
      case 1:  // Take snapshot (wait)
        WAIT(2.0);
        next_block();
        break;
      case 2:  // Go to bottom left
        MOVE_TO_AND_WAIT(-5, 0, 0.3, 1.0);
        next_block();
        break;
      case 3:  // Go to bottom right
        MOVE_TO_AND_WAIT(-5, 4, 0.3, 1.0);
        next_block();
        break;
      case 4:  // Go to top right
        MOVE_TO_AND_WAIT(0, 4, 0.3, 1.0);
        next_block();
        break;
      case 5:  // Homing
        params.btn.follow = 1;
        next_block();
        break;
      case 6:  // Wait for arrival
        if (dist2_to(0, 0) > 0.30f * 0.30f) break;
        WAIT(5.0);
        next_block();
        break;
      case 7:  // Reset
        params.btn.record_clear = 1;
        experiment_state.block = 0;
        break;
    }
}

void experiment_corridor_both(void) {
  switch (experiment_state.block) {
      case 0:  // Take snapshot (btn)
        MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0);
        params.btn.record_both_sequence = 1;
        next_block();
        break;
      case 1:  // Take snapshot (wait)
        WAIT(2.0);
        next_block();
        break;
      case 2:  // Go to top
        MOVE_TO_AND_WAIT(6, 0, 0.3, 1.0);
        next_block();
        break;
      case 3:  // Homing
        params.btn.follow = 1;
        next_block();
        break;
      case 4:  // Wait for arrival
        if (dist2_to(0, 0) > 0.30f * 0.30f) break;
        WAIT(5.0);
        next_block();
        break;
      case 5:  // Reset
        params.btn.record_clear = 1;
        experiment_state.block = 0;
        break;
    }
}

void experiment_corridor_odo(void) {
  switch (experiment_state.block) {
      case 0:  // Take snapshot (btn)
        MOVE_TO_AND_WAIT(0, 0, 0.3, 1.0);
        params.btn.record_odometry = 1;
        next_block();
        break;
      case 1:  // Take snapshot (wait)
        WAIT(2.0);
        next_block();
        break;
      case 2:  // Go to top
        MOVE_TO_AND_WAIT(6, 0, 0.3, 1.0);
        next_block();
        break;
      case 3:  // Homing
        params.btn.follow = 1;
        next_block();
        break;
      case 4:  // Wait for arrival
        if (dist2_to(0, 0) > 0.30f * 0.30f) break;
        WAIT(5.0);
        next_block();
        break;
      case 5:  // Reset
        params.btn.record_clear = 1;
        experiment_state.block = 0;
        break;
    }
}


typedef void (*experiment_fn)(void);

experiment_fn experiment_periodic_fn[] = {
    experiment_idle_periodic,
    experiment_fake_homing_periodic,
    experiment_single_snapshot_periodic,
    experiment_odometry_periodic,
    experiment_both_sequence_periodic,
    experiment_snapshot_distance_periodic,
    experiment_ins_correction_yaw,
    experiment_corridor_both,
    experiment_corridor_odo,
    experiment_u_both,
    experiment_u_odo,
};
static const int NUM_EXPERIMENTS = sizeof(experiment_periodic_fn) / sizeof(experiment_periodic_fn[0]);


static void experiment_periodic(void) {
  if (params.sw.experiment >= 0 && params.sw.experiment < NUM_EXPERIMENTS) {
    if (params.sw.experiment != experiment_state.experiment) {
      memset(&experiment_state, 0, sizeof(experiment_state));
      experiment_state.experiment = params.sw.experiment;
    }
    experiment_periodic_fn[params.sw.experiment]();
    log_buffer.point.block = experiment_state.block;
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
  safe &= crtpIsConnected();
  safe &= !pmIsBatteryLow();
  return safe;
}


static void handle_buttons(enum camera_state_t *cam_state) {
  // Handle camera state buttons
  if (params.btn.idle) {
    *cam_state = STATE_IDLE;
  } else if (params.btn.record_clear) {
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
    // HACK reset kalman filter on ground
    static uint8_t i = 0;
    i++;
    if ((i % 16) == 0) {
      paramSetInt(varid.kalman_reset, 1);
    }
    // Take-off check
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
      control_periodic();
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
