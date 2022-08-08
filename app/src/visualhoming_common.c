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

static void camera_communicate_state(void) {
  if (camera_state.local == STATE_SYNCED) {
    // Do nothing
  } else if (camera_state.local == camera_state.remote) {
    camera_state.local = STATE_SYNCED;
  } else {
    camera_msg_t msg;
    log_msg_t log;
    msg.type = CAM_MSG_T_COMMAND;
    msg.command.command = (uint8_t) camera_state.local;
    visualhoming_camera_send(msg);
    log.type = LOG_MSG_T_COMMAND;
    log.command.command = msg.command.command;
    visualhoming_log(log);
  }
}

static void camera_set_state(enum camera_state_t cmd) {
  camera_state.local = cmd;
}

static void camera_receive(void) {
  static uint16_t ins_correction_idx = 0;
  camera_msg_t message;
  while (visualhoming_camera_receive(&message)) {
    switch (message.type) {
      case CAM_MSG_T_COMMAND:
        camera_state.remote = message.command.command;
        break;
      case CAM_MSG_T_VECTOR:
        visualhoming_set_goal(message.vector.target.n, message.vector.target.e);
        break;
      case CAM_MSG_T_MAP: // TODO
        break;
      case CAM_MSG_T_INS_CORRECTION:
        if (message.ins_correction.idx != ins_correction_idx) {
          ins_correction_idx = message.ins_correction.idx;
          float dn = message.ins_correction.to.n - message.ins_correction.from.n;
          float de = message.ins_correction.to.e - message.ins_correction.from.e;
          float dpsi = message.ins_correction.psi_to - message.ins_correction.psi_from;
          visualhoming_position_update(dn, de);
          visualhoming_heading_update(dpsi);
        }
        break;
      case CAM_MSG_T_SNAPSHOT: // TODO
        break;
      default:
        break;
    }
  }
}



// External functions /////////////////////////////////////

void visualhoming_common_init(void) {

}


void visualhoming_common_periodic(void) {

}


extern void visualhoming_record(record_mode_t mode) {

}


extern void visualhoming_follow(follow_mode_t mode) {

}
