#ifndef __GSUSB_H__
#define __GSUSB_H__

#include "libusb.h"
#include "can.h"

// We only send a maximum of GSUSB_MAX_TX_REQ per channel at any one time.
// We keep track of how many are in play at a time by setting the echo_id and
// looking for it when it comes back.
#define GSUSB_MAX_TX_REQ  (10)
#define RX_BUFFER_SIZE (30)

#define GSUSB_OK                          (0)
#define GSUSB_ERROR_GENERAL               (-1)
#define GSUSB_ERROR_LIBUSB_INIT           (-2)
#define GSUSB_ERROR_NO_MEMORY             (-3)
#define GSUSB_ERROR_NO_DEVICES            (-4)
#define GSUSB_ERROR_OUT_OF_RANGE          (-5)
#define GSUSB_ERROR_OPEN_FAILED           (-6)
#define GSUSB_ERROR_UNABLE_TO_DETACH      (-7)
#define GSUSB_ERROR_UNABLE_TO_CLAIM       (-8)
#define GSUSB_ERROR_UNABLE_TO_SET_UID     (-9)
#define GSUSB_ERROR_UNABLE_TO_SET_FORMAT  (-10)
#define GSUSB_ERROR_UNABLE_TO_GET_CONFIG  (-11)
#define GSUSB_ERROR_UNABLE_TO_GET_BT      (-12)
#define GSUSB_ERROR_UNABLE_TO_SET_BT      (-13)
#define GSUSB_ERROR_UNABLE_TO_OPEN_PORT   (-14)
#define GSUSB_ERROR_NO_DEVICE             (-15)
#define GSUSB_ERROR_READING               (-16)
#define GSUSB_ERROR_TIMEOUT               (-17)
#define GSUSB_ERROR_WRITING               (-18)
#define GSUSB_ERROR_THREADING             (-19)

#define GSUSB_FLAGS_LIBUSB_OPEN   (0x01)
#define GSUSB_FLAGS_PORT_OPEN     (0x02)
#define GSUSB_FLAGS_RX_OVERFLOW   (0x04)

/// @brief Bit Timing struct
struct gsusb_device_bt_const {
  uint32_t feature;
  uint32_t fclk_can;
  uint32_t tseg1_min;
  uint32_t tseg1_max;
  uint32_t tseg2_min;
  uint32_t tseg2_max;
  uint32_t sjw_max;
  uint32_t brp_min;
  uint32_t brp_max;
  uint32_t brp_inc;
} __packed;

/// @brief USB device information
struct gsusb_device_config {
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t icount;       // The number of interfaces available on this device (can be up to 3)
  uint32_t sw_version;
  uint32_t hw_version;
} __packed;

/// @brief The transmit context. We keep track of transmissions as we can only have GSUSB_MAX_TX_REQ transmissions at a time
struct gsusb_tx_context {
  struct gsusb_ctx* can;
  uint32_t echo_id;
  uint64_t timestamp;
  struct can_frame* frame;
};

/// @brief Struct to keep track of the connection
struct gsusb_ctx {
  uint8_t flags;
  libusb_context* libusb_ctx;
  int interface;
  struct libusb_device_handle* devh;
  struct gsusb_device_bt_const bt_const;
  struct gsusb_device_config device_config;
  struct gsusb_tx_context tx_context[GSUSB_MAX_TX_REQ];
  uint32_t rx_buffer_count;
  struct can_frame rxBuffer[RX_BUFFER_SIZE];
  ssize_t frameSize;
};

int gsusbInit(struct gsusb_ctx *ctx);
void gsusbExit(struct gsusb_ctx *ctx);
/// @brief Gets a list of the GSUSB compatible devices available.
/// @return Number of devices found. Less than 0 is an error.
int gsusbGetDevicesCount(struct gsusb_ctx *ctx);
int gsusbOpen(struct gsusb_ctx *ctx, uint8_t deviceNo, uint8_t prop, uint8_t seg1, uint8_t seg2, uint8_t sjw, uint16_t brp);
void gsusbClose(struct gsusb_ctx *ctx);
int gsusbRead(struct gsusb_ctx *ctx, struct can_frame* frame);
int gsusbWrite(struct gsusb_ctx *ctx, struct can_frame* frame);

#endif  // __GSUSB_H__
