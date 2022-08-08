// Tom van Dijk, 2022 <tomvand@users.noreply.github.com>
// GPLv2 or later
//
// visualhoming-common
// Shared code for visual homing on both the Paparazzi autopilot and the
// Crazyflie autopilot.

#include "visualhoming_common.h"


// Camera communication ///////////////////////////////////

enum camera_state_t {
  STATE_SYNCED    = 0xFF, // Local only!
  STATE_IDLE      = 0x00,
  RECORD_CLEAR    = 0x0F,
  RECORD_SNAPSHOT = 0x01,
  RECORD_ODOMETRY = 0x02,
  RECORD_SEQUENCE = 0x04,
  FOLLOW_STAY     = 0x11,
  FOLLOW          = 0x12,
};

static struct {
  enum camera_state_t local;
  enum camera_state_t remote;
} camera_state;

static struct {
  bool is_new;
  struct msg_vector_t vector;
} vector;

static void camera_communicate_state(void) {
  if (camera_state.local == STATE_SYNCED) {
    // Do nothing
  } else if (camera_state.local == camera_state.remote) {
    camera_state.local = STATE_SYNCED;
  } else {
    vh_msg_t msg;
    msg.type = VH_MSG_COMMAND;
    msg.command.command = (uint8_t) camera_state.local;
    visualhoming_camera_send(&msg);
  }
}

static void camera_set_state(enum camera_state_t cmd) {
  camera_state.local = cmd;
}

// returns: TRUE when additional calls required
static bool camera_set_verify_state(enum camera_state_t cmd) {
  if (camera_state.local == STATE_SYNCED && camera_state.remote == cmd) {
    return false;
  } else {
    camera_set_state(cmd);
    return true;
  }
}

static void camera_receive(void) {
  static uint16_t ins_correction_idx = 0;
  vh_msg_t msg;
  while (visualhoming_camera_receive(&msg)) {
    visualhoming_log(&msg);
    switch (msg.type) {
      case VH_MSG_COMMAND:
        camera_state.remote = msg.command.command;
        break;
      case VH_MSG_VECTOR:
        vector.vector = msg.vector;
        vector.is_new = true;
        break;
      case VH_MSG_INS_CORRECTION:
        if (msg.ins_correction.idx != ins_correction_idx) {
          ins_correction_idx = msg.ins_correction.idx;
          float dn = msg.ins_correction.to.n - msg.ins_correction.from.n;
          float de = msg.ins_correction.to.e - msg.ins_correction.from.e;
          float dpsi = msg.ins_correction.psi_to - msg.ins_correction.psi_from;
          visualhoming_position_update(dn, de);
          visualhoming_heading_update(dpsi);
        }
        break;
      default:
        break;
    }
  }
}



// External functions /////////////////////////////////////

void visualhoming_common_init(void) {
  // Do nothing for now
}


void visualhoming_common_periodic(void) {
  // Send drone state to camera
  vh_msg_t msg;
  msg.type = VH_MSG_STATE;
  msg.state.state = visualhoming_get_state();
  visualhoming_camera_send(&msg);
  // Update camera state/command
  camera_communicate_state();
  // Receive readback and homing vectors
  camera_receive();
}


// returns: TRUE when additional calls required
bool visualhoming_record(record_mode_t mode) {
  return camera_set_verify_state(mode);
}


// returns: TRUE when additional calls required
bool visualhoming_follow(follow_mode_t mode) {
  if (camera_state.remote != FOLLOW) {
    camera_set_state(FOLLOW);
  } else if (camera_state.local == STATE_SYNCED &&
      camera_state.remote >= FOLLOW_STAY &&
      camera_state.remote <= FOLLOW &&
      vector.is_new) {
    visualhoming_set_goal(vector.vector.to.n, vector.vector.to.e);
  }
  vector.is_new = false;
  return true;  // Never ends
}
