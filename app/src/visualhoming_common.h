// Tom van Dijk, 2022 <tomvand@users.noreply.github.com>
// GPLv2 or later
//
// visualhoming-common
// Shared code for visual homing on both the Paparazzi autopilot and the
// Crazyflie autopilot.

#ifndef VISUALHOMING_COMMON_H__
#define VISUALHOMING_COMMON_H__

#include <stdbool.h>


struct pos2f_t {
  float n;
  float e;
};

struct att3f_t {
  float phi;
  float theta;
  float psi;
};

struct state_t {
  struct pos2f_t pos;
  struct att3f_t att;
};

typedef uint8_t record_mode_t; // TODO
typedef uint8_t follow_mode_t; // TODO

enum vh_message_type {
  VH_MSG_COMMAND,
  VH_MSG_VECTOR,
  VH_MSG_MAP,
  VH_MSG_INS_CORRECTION,
  VH_MSG_SNAPSHOT,
  VH_MSG_STATE,
};

struct msg_command_t {
  uint8_t command;
};

struct msg_vector_t {
  uint8_t source;
  struct pos2f_t to;
  struct pos2f_t from;
  float delta_psi;
};

struct msg_ins_correction_t {
  uint16_t idx;
  struct pos2f_t from;
  struct pos2f_t to;
  float psi_from;
  float psi_to;
};

struct msg_state_t {
  struct state_t state;
};

typedef struct {
  uint8_t type;
  union {
    struct msg_command_t        command;
    struct msg_vector_t         vector;
    struct msg_ins_correction_t ins_correction;
    struct msg_state_t          state;
  };
} vh_msg_t;


// TO BE PROVIDED BY EXTERNAL CODE:
extern void visualhoming_camera_send(vh_msg_t *camera_msg);
extern bool visualhoming_camera_receive(vh_msg_t *camera_msg_out);
extern void visualhoming_set_goal(float n, float e);
extern void visualhoming_position_update(float dn, float de);
extern void visualhoming_heading_update(float dpsi);
extern void visualhoming_log(vh_msg_t *log_msg);
extern struct state_t visualhoming_get_state(void);


// Provided by visualhoming_common
extern void visualhoming_common_init(void);
extern void visualhoming_common_periodic(void);

extern bool visualhoming_record(record_mode_t mode);
extern bool visualhoming_follow(follow_mode_t mode);


#endif // VISUALHOMING_COMMON_H__
