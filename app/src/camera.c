// Tom van Dijk, 2022 <tomvand@users.noreply.github.com>
// GPLv2 or later
//
// camera
// Communication over uart/pprzlink with camera

#include "camera.h"

#include <stddef.h>
#include "uart2.h"
#include "FreeRTOS.h"

#include "pprzlink/pprzlink.h"


// Pprzlink callbacks /////////////////////////////////////
// TX
static int check_space(uint8_t n) {
  return true;  // Always 'enough' space
}

static void put_char(uint8_t c) {
  uart2Putchar(c);
}

static void send_message(void) {
  // Do nothing
}

struct pprzlink_device_tx dev_tx;

// RX
static uint8_t single_char_buf;

static int char_available(void) {
  return uart2GetCharWithTimeout(&single_char_buf, M2T(10));
}

static uint8_t get_char(void) {
  return single_char_buf;
}

static uint8_t rx_buffer[255];

struct pprzlink_device_rx dev_rx;

static struct message_buffer_t {
  bool is_new;
  vh_msg_t msg;
} message_buffer;

static void new_message_cb(uint8_t sender_id, uint8_t receiver_id, uint8_t class_id, uint8_t message_id, uint8_t *buf, void *user_data) {
  switch (message_id) {
    case PPRZ_MSG_ID_VISUALHOMING_COMMAND:
      message_buffer.is_new = true;
      message_buffer.msg.type = VH_MSG_COMMAND;
      message_buffer.msg.command.command = pprzlink_get_VISUALHOMING_COMMAND_command(buf);
      break;
    case PPRZ_MSG_ID_VISUALHOMING_VECTOR:
      message_buffer.is_new = true;
      message_buffer.msg.type = VH_MSG_VECTOR;
      message_buffer.msg.vector.source = pprzlink_get_VISUALHOMING_VECTOR_source(buf);
      message_buffer.msg.vector.to.n = pprzlink_get_VISUALHOMING_VECTOR_target_n(buf);
      message_buffer.msg.vector.to.e = pprzlink_get_VISUALHOMING_VECTOR_target_e(buf);
      message_buffer.msg.vector.from.n = pprzlink_get_VISUALHOMING_VECTOR_start_n(buf);
      message_buffer.msg.vector.from.e = pprzlink_get_VISUALHOMING_VECTOR_start_e(buf);
      message_buffer.msg.vector.delta_psi = pprzlink_get_VISUALHOMING_VECTOR_delta_yaw(buf);
      break;
    case PPRZ_MSG_ID_VISUALHOMING_INS_CORRECTION:
      message_buffer.is_new = true;
      message_buffer.msg.type = VH_MSG_INS_CORRECTION;
      message_buffer.msg.ins_correction.idx = pprzlink_get_VISUALHOMING_INS_CORRECTION_snapshot_index(buf);
      message_buffer.msg.ins_correction.from.n = pprzlink_get_VISUALHOMING_INS_CORRECTION_n_from(buf);
      message_buffer.msg.ins_correction.from.e = pprzlink_get_VISUALHOMING_INS_CORRECTION_e_from(buf);
      message_buffer.msg.ins_correction.to.n = pprzlink_get_VISUALHOMING_INS_CORRECTION_n_to(buf);
      message_buffer.msg.ins_correction.to.e = pprzlink_get_VISUALHOMING_INS_CORRECTION_e_to(buf);
      message_buffer.msg.ins_correction.psi_from = pprzlink_get_VISUALHOMING_INS_CORRECTION_psi_from(buf);
      message_buffer.msg.ins_correction.psi_to = pprzlink_get_VISUALHOMING_INS_CORRECTION_psi_to(buf);
      break;
    case PPRZ_MSG_ID_VISUALHOMING_CAMERA:
    case PPRZ_MSG_ID_VISUALHOMING_MAP:
      break; // TODO
    default:
      break;
  }
}


// Camera communication functions /////////////////////////

void camera_init(void) {
  uart2Init(921600);
  dev_tx = pprzlink_device_tx_init(
      &check_space,
      &put_char,
      &send_message);
  dev_rx = pprzlink_device_rx_init(
      &char_available,
      &get_char,
      rx_buffer,
      NULL);
}

void visualhoming_camera_send(vh_msg_t *camera_msg) {
  switch (camera_msg->type) {
    case VH_MSG_COMMAND:
      pprzlink_msg_send_VISUALHOMING_COMMAND(&dev_tx, 0, 0,
          &camera_msg->command.command,
          0, NULL);
      break;
    case VH_MSG_STATE:
      pprzlink_msg_send_VISUALHOMING_STATE(&dev_tx, 0, 0,
          &camera_msg->state.state.pos.e, &camera_msg->state.state.pos.n,
          &camera_msg->state.state.att.phi, &camera_msg->state.state.att.theta, &camera_msg->state.state.att.psi);
      break;
    default:
      break;
  }
}

bool visualhoming_camera_receive(vh_msg_t *camera_msg_out) {
  pprzlink_check_and_parse(&dev_rx, &new_message_cb);
  if (message_buffer.is_new) {
    message_buffer.is_new = false;
    *camera_msg_out = message_buffer.msg;
    return true;
  }
  return false;
}
