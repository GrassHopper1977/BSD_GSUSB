#include <unistd.h>
#include <signal.h>
#include "gsusb.h"
#define LOG_LEVEL   6
#include "utils/logs.h"
#include "utils/timestamp.h"

#define TAG "test.c"

#define TX_TIME_NS  7812500 // = 7812.5us = 7.8125ms
#define SLEEP_TIME  1  // sleep time in us e.g. 1000 = 1ms, 

struct gsusb_ctx ctx;

void sigint_handler(int sig) {
  printf("\nSignal received (%i).\n", sig);
  if(sig == SIGINT) {
    LOGI(TAG, "Closing device");
    gsusbClose(&ctx);
    LOGI(TAG, "Closing gsusb");
    gsusbExit(&ctx);
    // Make sure the signal is passed down the line correctly.
    signal(SIGINT, SIG_DFL);
    kill(getpid(), SIGINT);
  }
}

int main(void) {
  uint64_t now, trigger;

  // Create the signal handler here - ensures that Ctrl-C gets passed back up to 
  signal(SIGINT, sigint_handler);

  gsusbInit(&ctx);
  int n = gsusbGetDevicesCount(&ctx);
  LOGI(TAG, "3. GSUSB devices detected = %i\n", n);

  // LOGI(TAG, "Opening device 0 @ 250kbps sample point = 87.5%%");
  // int rep = gsusbOpen(&ctx, 0, 6, 7, 2, 1, 12);
  LOGI(TAG, "Opening device 0 @ 500kbps sample point = 87.5%%");
  int rep = gsusbOpen(&ctx, 0, 6, 7, 2, 1, 6);
  // LOGI(TAG, "Opening device 0 @ 1Mbps sample point = 75%%");
  // int rep = gsusbOpen(&ctx, 0, 5, 6, 4, 1, 3);
  // LOGI(TAG, "Opening device 0 @ 1Mbps sample point = 87.5%%");
  // int rep = gsusbOpen(&ctx, 0, 6, 7, 2, 1, 3);
  if(rep == GSUSB_OK) {
    LOGI(TAG, "Device opened!");
  } else {
    LOGE(TAG, "Error opening!");
  }

  // Write items from the CAN bus, display them and then reply.
  uint32_t count = 0;
  struct can_frame frame = {
    .can_id = 0x02008001 | CAN_EFF_FLAG,
    .len = 2,
    .data = { 0, 0, 0, 0, 0, 0, 0, 0 }
  };
  struct can_frame frame2;
  trigger = nanos() + TX_TIME_NS;
  while(1) {
    int reply = gsusbRead(&ctx, &frame2);
    if(reply == GSUSB_OK) {
      LOGI(TAG, " IN ID: %08x, len: %u, Data: %02x %02x %02x %02x %02x %02x %02x %02x", frame2.can_id, frame2.len,
        frame2.data[0], frame2.data[1], frame2.data[2], frame2.data[3], frame2.data[4], frame2.data[5], frame2.data[6], frame2.data[7]);
    }
    now = nanos();
    if(now >= trigger) {
      trigger = nanos() + TX_TIME_NS;
      frame.data[0] = count &0x00ff;
      frame.data[1] = (count >> 8) &0x00ff;
      reply = gsusbWrite(&ctx, &frame);
      switch(reply) {
        case GSUSB_OK:
          // LOGI(TAG, "Message sent!");
          LOGI(TAG, "OUT ID: %08x, len: %u, Data: %02x %02x %02x %02x %02x %02x %02x %02x", frame2.can_id, frame2.len,
            frame2.data[0], frame2.data[1], frame2.data[2], frame2.data[3], frame2.data[4], frame2.data[5], frame2.data[6], frame2.data[7]);
          break;
        case GSUSB_ERROR_TIMEOUT:
          LOGE(TAG, "Message Tx timedout!");
          break;
        default:
          LOGE(TAG, "Another error coccurred.");
          break;
      }
      count++;
      count &= 0x03FF;
    }
    usleep(SLEEP_TIME);
  }
  
  LOGI(TAG, "Closing device");
  gsusbClose(&ctx);
  LOGI(TAG, "Closing gsusb");
  gsusbExit(&ctx);
  return 0;
}