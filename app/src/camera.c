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


// Camera communication functions /////////////////////////

void camera_init(void) {
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
    default:
      break;
  }
}

bool visualhoming_camera_receive(vh_msg_t *camera_msg_out) {
  return false;
}
