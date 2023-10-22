// logs.h

#ifndef __LOCALS_H__
#define __LOCALS_H__

#include <stdint.h>


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif



// Supported USB products
#define USB_VENDOR_ID_GS_USB_1            0x1D50
#define USB_PRODUCT_ID_GS_USB_1           0x606F
#define USB_VENDOR_ID_CANDLELIGHT         0x1209
#define USB_PRODUCT_ID_CANDLELIGHT        0x2323
#define USB_VENDOR_ID_CES_CANEXT_FD       0x1cd2
#define USB_PRODUCT_ID_CES_CANEXT_FD      0x606f
#define USB_VENDOR_ID_ABE_CANDEBUGGER_FD  0x16d0
#define USB_PRODUCT_ID_ABE_CANDEBUGGER_FD 0x10b8

// The endpoint for these devices
#define ENDPOINT_FLAG_IN        0x80
#define ENDPOINT_IN     (0x01 | ENDPOINT_FLAG_IN)
#define ENDPOINT_OUT    0x02

// We have to read more than we write or we won't get our Tx messages echoed back to us. We tend to read 30 times for each 1 write.
#define GSUSB_MAX_RX_REQ  (30)

// There may be some 3 channel devices out there but not more.
#define GSUSB_MAX_CHANNELS (3)

// Mode message - mode
enum gsusb_mode {
  GSUSB_MODE_RESET = 0, // Reset a channel, turns it off
  GSUSB_MODE_START,   // Start a channel
};
// Mode message - flags
#define GSUSB_CTRLMODE_NORMAL                   (0)
#define GSUSB_CTRLMODE_LOOPBACK                 (0x00000001)  // Bit 0
#define GSUSB_CTRLMODE_LISTENONLY               (0x00000002)  // Bit 1
#define GSUSB_CTRLMODE_3_SAMPLES                (0x00000004)  // Bit 2
#define GSUSB_CTRLMODE_ONE_SHOT                 (0x00000010)  // Bit 3
#define GSUSB_CTRLMODE_HW_TIMESTAMP             (0x00000020)  // Bit 4
// #define GSUSB_CTRLMODE_INTENTIFY                (0x00000100)  // Bit 5
// #define GSUSB_CTRLMODE_USER_ID_BIT              (0x00000200)  // Bit 6
#define GSUSB_CTRLMODE_PAD_PKT_TO_MAX_PKT_SIZE  (0x00000400)  // Bit 7
#define GSUSB_CTRLMODE_FD                       (0x00001000)  // Bit 8
// #define GSUSB_CTRLMODE_REQ_USB_QUIRK_LPC546XX   (0x00002000)  // Bit 9
// #define GSUSB_CTRLMODE_BT_CONST_EXT             (0x00004000)  // Bit 10
// #define GSUSB_CTRLMODE_TERMINATION              (0x00001000)  // Bit 11
#define GSUSB_CTRLMODE_BERR_REPORTING           (0x00010000)  // Bit 12
// #define GSUSB_CTRLMODE_GET_STATE                (0x00020000)  // Bit 13

/// @brief USB Mode struct
struct gsusb_device_mode {
  uint32_t mode;
  uint32_t flags;
} __packed;



// Bit Timing Const Definitions
#define GSUSB_FEATURE_LISTEN_ONLY (1 << 0)
#define GSUSB_FEATURE_LOOP_BACK (1 << 1)
#define GSUSB_FEATURE_TRIPLE_SAMPLE (1 << 2)
#define GSUSB_FEATURE_ONE_SHOT (1 << 3)
#define GSUSB_FEATURE_HW_TIMESTAMP (1 << 4)
#define GSUSB_FEATURE_IDENTIFY (1 << 5)
#define GSUSB_FEATURE_USER_ID (1 << 6)
#define GSUSB_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1 << 7)
#define GSUSB_FEATURE_FD (1 << 8)
#define GSUSB_FEATURE_REQ_USB_QUIRK_LPC546XX (1 << 9)
#define GSUSB_FEATURE_BT_CONST_EXT (1 << 10)
#define GSUSB_FEATURE_TERMINATION (1 << 11)
#define GSUSB_FEATURE_BERR_REPORTING (1 << 12)
#define GSUSB_FEATURE_GET_STATE (1 << 13)

/// @brief This is the CAN frame that is sent over the USB
struct host_frame {
  uint32_t echo_id; // So that we can recognise messages that we have sent.
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t channel;
  uint8_t flags;
  uint8_t reserved;
  uint8_t data[8];
  // uint32_t timestamp;
} __packed;

#define HOST_FRAME_FLAG_OVERFLOW  (0x01)
#define HOST_FRAME_FLAG_FD        (0x02)
#define HOST_FRAME_FLAG_BRS       (0x04)
#define HOST_FRAME_FLAG_ESI       (0x08)


#endif  // __LOCALS_H__