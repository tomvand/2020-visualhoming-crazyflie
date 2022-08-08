// Tom van Dijk, 2022 <tomvand@users.noreply.github.com>
// GPLv2 or later
//
// camera
// Communication over uart/pprzlink with camera

#ifndef CAMERA_H__
#define CAMERA_H__

#include "visualhoming_common.h"

void camera_init(void);

// Implemented here:
extern void visualhoming_camera_send(vh_msg_t *camera_msg);
extern bool visualhoming_camera_receive(vh_msg_t *camera_msg_out);


#endif // CAMERA_H__
