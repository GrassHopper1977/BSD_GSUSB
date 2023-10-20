#include <unistd.h>
#include <signal.h>
#include "gsusb.h"
#define LOG_LEVEL   3
#include "utils/logs.h"
#include "utils/timestamp.h"

#define TAG "test.c"

// #define TIMING_OPTION_1  // Using nanosleep()
#define TIMING_OPTION_2  // Using our nanos functions.
// #define TIMING_OPTION_3  // Using clock(). Useless on this system since CLOCK_PER_SEC is only 128! This must be clock(3c) not clock(3)!


#ifdef TIMING_OPTION_1
#define TX_TIME_NS  7812500 // = 7812.5us = 7.8125ms
// struct timespec remaining, request = {0, TX_TIME_NS };
// struct timespec remaining, request = {0, TX_TIME_NS - 1000000 };
struct timespec remaining, request = {0, TX_TIME_NS - (TX_TIME_NS * 0.125) }; // fudge factor to achieve our desired target
#endif
#ifdef TIMING_OPTION_2
#define TX_TIME_NS  7812500 // = 7812.5us = 7.8125ms
#define SLEEP_TIME  1  // sleep time in us e.g. 1000 = 1ms, 
// struct timespec remaining, request = {0, 1000 }; // fudge factor to achieve our desired target
#endif
#ifdef TIMING_OPTION_3
#define SLEEP_TIME  10  // sleep time in us e.g. 1000 = 1ms
#define TX_TIME_US (7813)  // Nearest we can get to 7812.5us
#endif

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
#ifdef TIMING_OPTION_2
  uint64_t now, trigger;
#endif
#ifdef TIMING_OPTION_3
  clock_t now, trigger;
#endif

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
    .can_id = 0x2008001 & CAN_EFF_FLAG,
    .len = 2,
    .data = { 0, 0, 0, 0, 0, 0, 0, 0 }
  };
  struct can_frame frame2;
#ifdef TIMING_OPTION_2
  trigger = nanos() + TX_TIME_NS;
#endif
#ifdef TIMING_OPTION_3
  now = clock();
  trigger = now + TX_TIME_US;
  LOGI(TAG, "1. now = %d, trigger = %d", now, trigger);
#endif
  uint64_t loopstart = nanos();
  while(1) {
    loopstart = nanos();  
    LOGI(TAG, "1. %10luns - before gsusbRead()", nanos()- loopstart);
    int reply = gsusbRead(&ctx, &frame2);
    LOGI(TAG, "2. %10luns - after gsusbRead()", nanos()- loopstart);
#ifdef TIMING_OPTION_2
    now = nanos();
    if(now >= trigger) {
      LOGI(TAG,"target-now = %luns", now-trigger);
      trigger = nanos() + TX_TIME_NS;
#endif
#ifdef TIMING_OPTION_3
    now = clock();
    LOGI(TAG, "2. now = %d, trigger = %d", now, trigger);
    if(now >= trigger) {
      trigger = now + TX_TIME_US;
      LOGI(TAG, "3. now = %d, trigger = %d", now, trigger);
#endif
      printf("count = %04x\r", count);
      frame.data[0] = count &0x00ff;
      frame.data[1] = (count >> 8) &0x00ff;
      reply = gsusbWrite(&ctx, &frame);
      switch(reply) {
        case GSUSB_OK:
          LOGI(TAG, "Message sent!");
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
#ifdef TIMING_OPTION_2
    }
    // uint64_t before = nanos();
    usleep(SLEEP_TIME);
    // LOGI(TAG, "usleep(%u) lasted %luns", SLEEP_TIME, nanos()-before);
    // nanosleep(&request, &remaining);
    // LOGI(TAG, "nanosleep(%lu) lasted %luns", request.tv_nsec, nanos()-before);
#endif
#ifdef TIMING_OPTION_3
    }
    usleep(SLEEP_TIME);
#endif
#ifdef TIMING_OPTION_1
    nanosleep(&request, &remaining);
#endif
  }
  
  LOGI(TAG, "Closing device");
  gsusbClose(&ctx);
  LOGI(TAG, "Closing gsusb");
  gsusbExit(&ctx);
  return 0;
}