// Tom van Dijk, 2022 <tomvand@users.noreply.github.com>
// GPLv2 or later
//
// visualhoming-common
// Shared code for visual homing on both the Paparazzi autopilot and the
// Crazyflie autopilot.

#ifndef VISUALHOMING_COMMON_H__
#define VISUALHOMING_COMMON_H__

struct vec2f_t {
  float n;
  float e;
};

typedef uint8_t record_mode_t; // TODO
typedef uint8_t follow_mode_t; // TODO

enum camera_msg_type {
  CAM_MSG_T_COMMAND,
  CAM_MSG_T_VECTOR,
  CAM_MSG_T_MAP,
  CAM_MSG_T_INS_CORRECTION,
  CAM_MSG_T_SNAPSHOT,
};

typedef struct {
  uint8_t type;
  union {
    struct {
      uint8_t command;
    } command;
    struct {
      uint8_t source;
      struct vec2f_t target;
      struct vec2f_t from;
      float delta_psi;
    } vector;
    struct {
      uint16_t idx;
      struct vec2f_t from;
      struct vec2f_t to;
      float psi_from;
      float psi_to;
    } ins_correction;
  };
} camera_msg_t;

enum log_msg_type {
  LOG_MSG_T_COMMAND,
  LOG_MSG_T_VECTOR,
  LOG_MSG_T_MAP,
};

typedef struct {
  uint8_t type;
  union {
    struct {
      uint8_t command;
    } command;
  };
} log_msg_t;


// TO BE PROVIDED BY EXTERNAL CODE:
extern void visualhoming_camera_send(camera_msg_t *camera_msg);
extern bool visualhoming_camera_receive(camera_msg_t *camera_msg_out);
extern void visualhoming_set_goal(float n, float e);
extern void visualhoming_position_update(float dn, float de);
extern void visualhoming_heading_update(float dpsi);
extern void visualhoming_log(log_msg_t *log_msg);


// Provided by visualhoming_common
extern void visualhoming_common_init(void);
extern void visualhoming_common_periodic(void);

extern void visualhoming_record(record_mode_t mode);
extern void visualhoming_follow(follow_mode_t mode);


#endif // VISUALHOMING_COMMON_H__
