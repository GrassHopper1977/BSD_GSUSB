#include <unistd.h>
#include "gsusb.h"
#define LOG_LEVEL   3
#include "utils/logs.h"
#include "utils/timestamp.h"

#define TAG "test.c"

#define SLEEP_TIME  100  // sleep time in us e.g. 1000 = 1ms

int main(void) {
  struct gsusb_ctx ctx;
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

  // Read 100 items from the CAN bus, display them and then reply.
  int read = 0;
  struct can_frame frame;
  struct can_frame frame2 = {
    .can_id = 0x101,
    .len = 8,
    .data = { 0, 1, 2, 3, 4, 5, 6, 7 }
  };
  while(read < 1000) {
    int reply = gsusbRead(&ctx, &frame);
    if(GSUSB_OK == reply) {
      LOGI(TAG, "Read Id: %u DLC: %u Data: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X", 
        frame.can_id, frame.len, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
      read++;
      LOGI(TAG, "read = %i", read);
      if(frame.can_id != 0x101) {
        reply = gsusbWrite(&ctx, &frame2);
        switch(reply) {
          case GSUSB_OK:
            LOGI(TAG, "Message sent!\n");
            break;
          case GSUSB_ERROR_TIMEOUT:
            LOGE(TAG, "Message Tx timedout!");
            break;
          default:
            LOGE(TAG, "Another error coccurred.");
            break;
        }
        frame2.data[0]++;
      }
    }
    usleep(SLEEP_TIME);
  }
  printf("read = %i\n", read);

  LOGI(TAG, "Closing device");
  gsusbClose(&ctx);
  LOGI(TAG, "Closing gsusb");
  gsusbExit(&ctx);
  return 0;
}