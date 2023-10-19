// gsusb.c

#define LOG_LEVEL   0
#include "utils/logs.h"
#include "utils/timestamp.h"
#include "locals.h"
#include "gsusb.h"
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/endian.h>


#define TAG "libGSUSB"

// Device Specific Constants
enum gsusb_breq {
  GSUSB_BREQ_HOST_FORMAT = 0,
  GSUSB_BREQ_BITTIMING,
  GSUSB_BREQ_MODE,
  GSUSB_BREQ_BERR,
  GSUSB_BREQ_BT_CONST,
  GSUSB_BREQ_DEVICE_CONFIG,
  GSUSB_BREQ_TIMESTAMP,
  GSUSB_BREQ_IDENTIFY,
  GSUSB_BREQ_GET_USER_ID,
  GSUSB_BREQ_SET_USER_ID,
  GSUSB_BREQ_DATA_BITTIMING,
  GSUSB_BREQ_BT_CONST_EXT,
  GSUSB_BREQ_SET_TERMINATION,
  GSUSB_BREQ_GET_TERMINATION,
  GSUSB_BREQ_GET_STATE,
};

enum gsusb_mode {
  GSUSB_MODE_RESET = 0, // Reset a channel, turns it off
  GSUSB_MODE_START,   // Start a channel
};


void sigint_handler(int sig) {
  LOGE(TAG, "\nSignal received (%i).\n", sig);

  if(sig == SIGINT) {
    // Make sure the signal is passed down the line correctly.
    signal(SIGINT, SIG_DFL);
    kill(getpid(), SIGINT);
  }
}

#define BITRATE_DATA_LEN (20)
// These are the speed settings. We'd like to offer custom bitrates and sample points too.
int set_bitrate(struct gsusb_ctx *ctx, uint8_t prop, uint8_t seg1, uint8_t seg2, uint8_t sjw, uint16_t brp) {
  LOGI(TAG, "Setting bitrate...");
  uint8_t bmReqType = 0x41;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_BITTIMING;            // the request field for this packet
  uint16_t wVal = 0x0000;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  uint16_t wLen = BITRATE_DATA_LEN;   // length of this setup packet 
  unsigned char data[] = {
    prop, 0x00, 0x00, 0x00, // Prop seg
    seg1, 0x00, 0x00, 0x00, // Phase Seg 1
    seg2, 0x00, 0x00, 0x00, // Seg 2
    sjw, 0x00, 0x00, 0x00, // SJW
    (brp & 0x00ff), ((brp >> 8) & 0x00ff), 0x00, 0x00  // BRP
  };
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int config = libusb_control_transfer(ctx->devh,bmReqType,bReq,wVal,wIndex, data, wLen,to);

  return config;
}

// Set User ID: candleLight allows optional support for reading/writing of a user defined value into the device's flash. It's isn't widely supported and probably isn't required most of the time.
int port_set_user_id(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Set the User ID (GSUSB_BREQ_SET_USER_ID)");
  uint8_t bmReqType = 0x00;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_SET_USER_ID;            // the request field for this packet
  uint16_t wVal = 0x0001;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  uint16_t wLen = 0;      // length of this setup packet 
  unsigned char data[] = {0x00};
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int config = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, data, wLen, to);

  return config;
}

// Host format sets the endian. We write 0x0000beef to the device and it determines the endianess from that. However, candleLight may only support Little Endian so that why we always send it it Little Endian.
int port_set_host_format(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Set the host format to Little Endian (as candleLight devices are always Little Endian) (GSUSB_BREQ_HOST_FORMAT)");
  uint8_t bmReqType = 0x41;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_HOST_FORMAT;            // the request field for this packet
  uint16_t wVal = 0x0001;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  // the data buffer for the in/output data
  unsigned char data[] = {
    0xef, 0xbe, 0x00, 0x00
  };
  uint16_t wLen = sizeof(data);   // length of this setup packet 
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int config = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, data, wLen, to);

  return config;
}

// Get device information from the USB 2 CAN device. Includes SW and HW versions.
int port_get_device_config(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Getting device config info (GSUSB_BREQ_DEVICE_CONFIG)");
  uint8_t bmReqType = 0xC1;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_DEVICE_CONFIG;            // the request field for this packet
  uint16_t wVal = 0x0001;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  // the data buffer for the in/output data
  struct gsusb_device_config data;
  uint16_t wLen = sizeof(data);   // length of this setup packet 
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int ret = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, (uint8_t *)(&data), wLen, to);
  if(ret >=0) {
    ctx->device_config.reserved1 = data.reserved1;
    ctx->device_config.reserved2 = data.reserved2;
    ctx->device_config.reserved3 = data.reserved3;
    ctx->device_config.sw_version = le32toh(data.sw_version);
    ctx->device_config.hw_version = le32toh(data.hw_version);
    LOGI(TAG, "Device Config:");
    LOGI(TAG, "   reserved1: 0x%02x", ctx->device_config.reserved1);
    LOGI(TAG, "   reserved2: 0x%02x", ctx->device_config.reserved2);
    LOGI(TAG, "   reserved3: 0x%02x", ctx->device_config.reserved3);
    LOGI(TAG, "     i count: %u (%u interfaces)", ctx->device_config.icount, data.icount + 1);
    LOGI(TAG, "  sw_version: %u", ctx->device_config.sw_version);
    LOGI(TAG, "  hw_version: %u", ctx->device_config.hw_version);
  }

  return ret;
}

// Get the Bit Timming settings.
int port_get_bit_timing(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Getting the port's bit timing info (GSUSB_BREQ_BT_CONST)");
  uint8_t bmReqType = 0xC1;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_BT_CONST;            // the request field for this packet
  uint16_t wVal = 0x0000;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  // the data buffer for the in/output data
  struct gsusb_device_bt_const data;
  uint16_t wLen = sizeof(data);   // length of this setup packet 
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int ret = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, (uint8_t *)(&data), wLen, to);
  if(ret >=0) {
    ctx->bt_const.feature = le32toh(data.feature);
    ctx->bt_const.fclk_can = le32toh(data.fclk_can);
    ctx->bt_const.tseg1_min = le32toh(data.tseg1_min);
    ctx->bt_const.tseg1_max = le32toh(data.tseg1_max);
    ctx->bt_const.tseg2_min = le32toh(data.tseg2_min);
    ctx->bt_const.tseg2_max = le32toh(data.tseg2_max);
    ctx->bt_const.sjw_max = le32toh(data.sjw_max);
    ctx->bt_const.brp_min = le32toh(data.brp_min);
    ctx->bt_const.brp_max = le32toh(data.brp_max);
    ctx->bt_const.brp_inc = le32toh(data.brp_inc);
    LOGI(TAG, "BT Const Information:");
    int16_t feature = ctx->bt_const.feature;
    LOGI(TAG, "    Features: %08x", feature);
    if(feature & GSUSB_FEATURE_LISTEN_ONLY) {
      LOGI(TAG, "    GSUSB_FEATURE_LISTEN_ONLY");
    }
    if(feature & GSUSB_FEATURE_LOOP_BACK) {
      LOGI(TAG, "    GSUSB_FEATURE_LOOP_BACK");
    }
    if(feature & GSUSB_FEATURE_TRIPLE_SAMPLE) {
      LOGI(TAG, "    GSUSB_FEATURE_TRIPLE_SAMPLE");
    }
    if(feature & GSUSB_FEATURE_ONE_SHOT) {
      LOGI(TAG, "    GSUSB_FEATURE_ONE_SHOT");
    }
    if(feature & GSUSB_FEATURE_HW_TIMESTAMP) {
      LOGI(TAG, "    GSUSB_FEATURE_HW_TIMESTAMP");
    }
    if(feature & GSUSB_FEATURE_IDENTIFY) {
      LOGI(TAG, "    GSUSB_FEATURE_IDENTIFY");
    }
    if(feature & GSUSB_FEATURE_USER_ID) {
      LOGI(TAG, "    GSUSB_FEATURE_USER_ID");
    }
    if(feature & GSUSB_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE) {
      LOGI(TAG, "    GSUSB_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE");
    }
    if(feature & GSUSB_FEATURE_FD) {
      LOGI(TAG, "    GSUSB_FEATURE_FD");
    }
    if(feature & GSUSB_FEATURE_REQ_USB_QUIRK_LPC546XX) {
      LOGI(TAG, "    GSUSB_FEATURE_REQ_USB_QUIRK_LPC546XX");
    }
    if(feature & GSUSB_FEATURE_BT_CONST_EXT) {
      LOGI(TAG, "    GSUSB_FEATURE_BT_CONST_EXT");
    }
    if(feature & GSUSB_FEATURE_TERMINATION) {
      LOGI(TAG, "    GSUSB_FEATURE_TERMINATION");
    }
    if(feature & GSUSB_FEATURE_BERR_REPORTING) {
      LOGI(TAG, "    GSUSB_FEATURE_BERR_REPORTING");
    }
    if(feature & GSUSB_FEATURE_GET_STATE) {
      LOGI(TAG, "    GSUSB_FEATURE_GET_STATE");
    }
  LOGI(TAG, "   fclk_can: %u", ctx->bt_const.fclk_can);
    LOGI(TAG, "  tseg1_min: %u", ctx->bt_const.tseg1_min);
    LOGI(TAG, "  tseg1_max: %u", ctx->bt_const.tseg1_max);
    LOGI(TAG, "  tseg2_min: %u", ctx->bt_const.tseg2_min);
    LOGI(TAG, "  tseg2_max: %u", ctx->bt_const.tseg2_max);
    LOGI(TAG, "    sjw_max: %u", ctx->bt_const.sjw_max);
    LOGI(TAG, "    brp_min: %u", ctx->bt_const.brp_min);
    LOGI(TAG, "    brp_max: %u", ctx->bt_const.brp_max);
    LOGI(TAG, "    brp_inc: %u", ctx->bt_const.brp_inc);
  }

  return ret;
}

int port_open(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Opening the port (GSUSB_BREQ_MODE)");
  uint8_t bmReqType = 0x41;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_MODE;            // the request field for this packet
  uint16_t wVal = 0x0000;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  
  // the data buffer for the in/output data
  unsigned char data[] = {
    0x01, 0x00, 0x00, 0x00, // CAN_MODE_START
    0x00, 0x00, 0x00, 0x00 
  };
  uint16_t wLen = sizeof(data);   // length of this setup packet 
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int config = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, data, wLen, to);

  return config;
}

int port_close(struct gsusb_ctx *ctx) {
  LOGI(TAG, "Closing the port (GSUSB_BREQ_MODE)");
  uint8_t bmReqType = 0x41;       // the request type (direction of transfer)
  uint8_t bReq = GSUSB_BREQ_MODE;            // the request field for this packet
  uint16_t wVal = 0x0000;         // the value field for this packet
  uint16_t wIndex = 0x0000;       // the index field for this packet
  // the data buffer for the in/output data
  unsigned char data[] = {
    0x00, 0x00, 0x00, 0x00, // CAN_MODE_RESET
    0x00, 0x00, 0x00, 0x00
  };
  uint16_t wLen = sizeof(data);   // length of this setup packet 
  unsigned int to = 0;    // timeout duration (if transfer fails)

  // transfer the setup packet to the USB device
  int config = libusb_control_transfer(ctx->devh, bmReqType, bReq, wVal, wIndex, data, wLen, to);

  return config;
}

int gsusbInit(struct gsusb_ctx *ctx) {
  libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_DEBUG);
  //libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_INFO);

  memset(ctx, 0, sizeof(struct gsusb_ctx));
  int ret = libusb_init(&(ctx->libusb_ctx));
  if (ret < 0) {
    LOGE(TAG, "Failed to initialize libusb (ret = %d)\n", ret);
    return GSUSB_ERROR_LIBUSB_INIT;
  }
  return GSUSB_OK;
}

void gsusbExit(struct gsusb_ctx *ctx) {
  if(ctx->libusb_ctx != NULL) {
    gsusbClose(ctx);  // Check that everything is closed.
    libusb_exit(ctx->libusb_ctx);
    ctx->libusb_ctx = NULL;
  }
}

int gsusbGetDevicesCount(struct gsusb_ctx *ctx) {
  libusb_device **list;
  
  ssize_t cnt = libusb_get_device_list(ctx->libusb_ctx, &list);
  LOGI(TAG, "cnt = %zi", cnt);
  if (cnt < 0)
    return GSUSB_ERROR_LIBUSB_INIT;
  else if (cnt == 0) {
    libusb_free_device_list(list, 1);
    return 0;
  }

  LOGI(TAG, "%ld USB devices found.", cnt);
  LOGI(TAG, "USB Devices found:");

  // This is where we build our list of devices that we are looking for.
  libusb_device* validdevices[cnt];
  int devCnt = 0;
  for (ssize_t i = 0; i < cnt; i++) {
    libusb_device *dev = list[i];
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      LOGE(TAG, "failed to get device descriptor (r = %d)", r);
    if(((USB_VENDOR_ID_GS_USB_1 == desc.idVendor) && (USB_PRODUCT_ID_GS_USB_1 == desc.idProduct))
      || ((USB_VENDOR_ID_CANDLELIGHT == desc.idVendor) && (USB_PRODUCT_ID_CANDLELIGHT == desc.idProduct))
      || ((USB_VENDOR_ID_CES_CANEXT_FD == desc.idVendor) && (USB_PRODUCT_ID_CES_CANEXT_FD == desc.idProduct))
      || ((USB_VENDOR_ID_ABE_CANDEBUGGER_FD == desc.idVendor) && (USB_PRODUCT_ID_ABE_CANDEBUGGER_FD == desc.idProduct))) {
      LOGI(TAG, "%2ld Vendor ID: %i (0x%04x), Product ID: %i (0x%04x), Manufacturer: %i, Product: %i, Serial: %i *** DEVICE %i ***", i+1, desc.idVendor, desc.idVendor, desc.idProduct, desc.idProduct, desc.iManufacturer, desc.iProduct, desc.iSerialNumber, devCnt);
      validdevices[devCnt] = dev;
      devCnt++;
    } else {
      LOGI(TAG, "%2ld Vendor ID: %i (0x%04x), Product ID: %i (0x%04x), Manufacturer: %i, Product: %i, Serial: %i", i+1, desc.idVendor, desc.idVendor, desc.idProduct, desc.idProduct, desc.iManufacturer, desc.iProduct, desc.iSerialNumber);
    }
  }

  LOGI(TAG, "Compatible USB to CAN Devices found: %i", devCnt);

  libusb_free_device_list(list, 1);
  return devCnt;
}

int gsusbOpen(struct gsusb_ctx *ctx, uint8_t deviceNo, uint8_t prop, uint8_t seg1, uint8_t seg2, uint8_t sjw, uint16_t brp) {
  libusb_device **list;
  int ret = GSUSB_OK;
  
  ssize_t cnt = libusb_get_device_list(ctx->libusb_ctx, &list);
  LOGI(TAG, "cnt = %zi", cnt);
  if (cnt < 0) {
    return GSUSB_ERROR_LIBUSB_INIT;
  } else if (cnt == 0) {
    libusb_free_device_list(list, 1);
    return GSUSB_ERROR_NO_DEVICES;
  }

  LOGI(TAG, "%ld USB devices found.", cnt);
  LOGI(TAG, "USB Devices found:");

  // This is where we build our list of devices that we are looking for.
  libusb_device* validdevices[cnt];
  int devCnt = 0;
  for (ssize_t i = 0; i < cnt; i++) {
    libusb_device *dev = list[i];
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      LOGE(TAG, "failed to get device descriptor (r = %d)", r);
    if(((USB_VENDOR_ID_GS_USB_1 == desc.idVendor) && (USB_PRODUCT_ID_GS_USB_1 == desc.idProduct))
      || ((USB_VENDOR_ID_CANDLELIGHT == desc.idVendor) && (USB_PRODUCT_ID_CANDLELIGHT == desc.idProduct))
      || ((USB_VENDOR_ID_CES_CANEXT_FD == desc.idVendor) && (USB_PRODUCT_ID_CES_CANEXT_FD == desc.idProduct))
      || ((USB_VENDOR_ID_ABE_CANDEBUGGER_FD == desc.idVendor) && (USB_PRODUCT_ID_ABE_CANDEBUGGER_FD == desc.idProduct))) {
      LOGI(TAG, "%2ld Vendor ID: %i (0x%04x), Product ID: %i (0x%04x), Manufacturer: %i, Product: %i, Serial: %i *** DEVICE %i ***", i+1, desc.idVendor, desc.idVendor, desc.idProduct, desc.idProduct, desc.iManufacturer, desc.iProduct, desc.iSerialNumber, devCnt);
      validdevices[devCnt] = dev;
      devCnt++;
    } else {
      LOGI(TAG, "%2ld Vendor ID: %i (0x%04x), Product ID: %i (0x%04x), Manufacturer: %i, Product: %i, Serial: %i", i+1, desc.idVendor, desc.idVendor, desc.idProduct, desc.idProduct, desc.iManufacturer, desc.iProduct, desc.iSerialNumber);
    }
  }

  LOGI(TAG, "Compatible USB to CAN Devices found: %i", devCnt);

  if(deviceNo < devCnt) {
    LOGI(TAG, "Attempting to device %i now...", deviceNo);
    // int ret = libusb_open(validdevices[deviceNumber], &devh);
    if(0 != libusb_open(validdevices[deviceNo], &ctx->devh)) {
      LOGE(TAG, "failed to open the device.");
      ret = GSUSB_ERROR_OPEN_FAILED;
    } else {
      LOGI(TAG, "Device opened.");
      ctx->flags |= GSUSB_FLAGS_LIBUSB_OPEN;
    }
  } else {
    LOGE(TAG, "Unable to open device %i as there are only %i devices. Note: Device numbering starts at 0.", deviceNo, devCnt);
    ret = GSUSB_ERROR_OUT_OF_RANGE;
  }

  libusb_free_device_list(list, 1);

  // If opened then we need to check that we can use the device and detach any existing connections.
  if(ctx->flags & GSUSB_FLAGS_LIBUSB_OPEN) {
    LOGI(TAG, "Checking if kernel driver is active...");
    if(libusb_kernel_driver_active(ctx->devh, ctx->interface) == 1) {
      LOGI(TAG, "Detaching kernel driver...");
      int reply = libusb_detach_kernel_driver(ctx->devh, ctx->interface);
      if(reply < 0) {
        LOGE(TAG, "%s: %s Unable to detach.", libusb_error_name(ret), libusb_strerror(ret));
        ret = GSUSB_ERROR_UNABLE_TO_DETACH;
      }
    }

    LOGI(TAG, "Claiming interface...");
    int reply = libusb_claim_interface(ctx->devh, ctx->interface);
    if(reply < 0) {
      LOGE(TAG, "%s: %s Unable to claim interface %d.", libusb_error_name(reply), libusb_strerror(reply), ctx->interface);
      ret = GSUSB_ERROR_UNABLE_TO_CLAIM;
    }


    LOGI(TAG, "Setting up for comms...");
    reply = port_set_user_id(ctx);
    if(reply < 0) {
      LOGE(TAG, "ERROR! Unable to set user ID.");
      ret = GSUSB_ERROR_UNABLE_TO_SET_UID;
    }
    reply = port_set_host_format(ctx);
    if(reply < 0) {
      LOGE(TAG, "ERROR! Unable to set host format.");
      ret = GSUSB_ERROR_UNABLE_TO_SET_FORMAT;
    }
    reply = port_get_device_config(ctx);
    if(reply < 0) {
      fprintf(stderr, "ERROR! Unable to setup2.");
      ret = GSUSB_ERROR_UNABLE_TO_GET_CONFIG;
    }
    reply = port_get_bit_timing(ctx);
    if(reply < 0) {
      LOGE(TAG, "ERROR! Unable to get bit timing.");
      ret = GSUSB_ERROR_UNABLE_TO_GET_BT;
    }

    reply = set_bitrate(ctx, prop, seg1, seg2, sjw, brp);
    if(reply < 0) {
      LOGE(TAG, "ERROR! Unable to set bitrate.");
      ret = GSUSB_ERROR_UNABLE_TO_SET_BT;
    }

    LOGI(TAG, "Opening port...");
    reply = port_open(ctx);
    if(reply < 0) {
      LOGE(TAG, "ERROR! Unable to open port.");
      ret = GSUSB_ERROR_UNABLE_TO_OPEN_PORT;
    } else {
      ctx->flags |= GSUSB_FLAGS_PORT_OPEN;
    }
    LOGI(TAG, "USB to CAN device is connected!");

  }

  return ret;
}

void gsusbClose(struct gsusb_ctx *ctx) {
  int ret = 0;
  if(ctx->flags & GSUSB_FLAGS_PORT_OPEN) {
    ret = port_close(ctx);
    if(ret < 0) {
      LOGE(TAG, "ERROR! Unable to close port.");
    }
    ctx->flags &= ~GSUSB_FLAGS_PORT_OPEN;
  }
  if(ctx->flags & GSUSB_FLAGS_LIBUSB_OPEN) {
    ret = libusb_attach_kernel_driver(ctx->devh, ctx->interface);
    if(ret < 0) {
      LOGE(TAG, "%s: %s Unable to reattach existing driver.", libusb_error_name(ret), libusb_strerror(ret));
    }
    ctx->flags &= ~GSUSB_FLAGS_LIBUSB_OPEN;
  }
}

void print_host_frame(const char* tag, struct host_frame *data, uint8_t err, const char *format, ...) {
  FILE * fd = stdout;
  int loglevel = LOG_LEVEL_INFO;

  if(err) {
    LOGE(tag, "ID: ");
    fd = stderr;
    loglevel = LOG_LEVEL_ERROR;
  } else {
    LOGI(tag, "ID: ");
  }

  if(LOG_LEVEL < loglevel)
    return;

  if((data->can_id & CAN_EFF_FLAG) || (data->can_id & CAN_ERR_FLAG)) {
    fprintf(fd, "%08x", data->can_id & CAN_EFF_MASK);
  } else {
    fprintf(fd, "     %03x", data->can_id & CAN_SFF_MASK);
  }

  if(data->can_id & CAN_ERR_FLAG) {
    if(data->can_id & CAN_ERR_RESTARTED) {
      fprintf(fd, ", ERROR State: CAN Restarted ");
    }
    if(data->can_id & CAN_ERR_BUSERROR) {
      fprintf(fd, ", ERROR State: CAN Bus Error ");
    }
    if(data->can_id & CAN_ERR_BUSOFF) {
      fprintf(fd, ", ERROR State: CAN Bus Off ");
    }
    if(data->can_id & CAN_ERR_ACK) {
      fprintf(fd, ", ERROR State: No ACK on Tx ");
    }
    if(data->can_id & CAN_ERR_TRX) {
      fprintf(fd, ", ERROR State: Transceiver Status ");
      if(data->data[4] == CAN_ERR_TRX_UNSPEC) {
        fprintf(fd, "unspecified ");
      } else {
        switch(data->data[4] & 0x0f) {
          case CAN_ERR_TRX_CANH_NO_WIRE:
            fprintf(fd, "CAN High no wire ");
            break;
          case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
            fprintf(fd, "CAN High short to BAT ");
            break;
          case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
            fprintf(fd, "CAN High short to Vcc ");
            break;
          case CAN_ERR_TRX_CANH_SHORT_TO_GND:
            fprintf(fd, "CAN High short to GND ");
            break;
        }
        switch(data->data[4] & 0xf0) {
          case CAN_ERR_TRX_CANL_NO_WIRE:
            fprintf(fd, "CAN Low no wire ");
            break;
          case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
            fprintf(fd, "CAN Low short to BAT ");
            break;
          case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
            fprintf(fd, "CAN Low short to Vcc ");
            break;
          case CAN_ERR_TRX_CANL_SHORT_TO_GND:
            fprintf(fd, "CAN Low short to GND ");
            break;
        }
        if(data->data[4] & CAN_ERR_TRX_CANL_SHORT_TO_CANH) {
          fprintf(fd, "CAN Low short to CAN High ");
        }
      }
    }
    if(data->can_id & CAN_ERR_PROT) {
      fprintf(fd, ", ERROR State: CAN Protocol Violations: ");
      if(data->data[2] == CAN_ERR_PROT_UNSPEC) {
        fprintf(fd, "unspecified ");
      }
      if(data->data[2] & CAN_ERR_PROT_BIT) {
        fprintf(fd, "single bit error, ");
      }
      if(data->data[2] & CAN_ERR_PROT_FORM) {
        fprintf(fd, "frame format error, ");
      }
      if(data->data[2] & CAN_ERR_PROT_STUFF) {
        fprintf(fd, "bit stuffing error, ");
      }
      if(data->data[2] & CAN_ERR_PROT_BIT0) {
        fprintf(fd, "unable to send dominant bit, ");
      }
      if(data->data[2] & CAN_ERR_PROT_BIT1) {
        fprintf(fd, "unable to send recessive bit, ");
      }
      if(data->data[2] & CAN_ERR_PROT_OVERLOAD) {
        fprintf(fd, "bus overload, ");
      }
      if(data->data[2] & CAN_ERR_PROT_ACTIVE) {
        fprintf(fd, "active error announcement, ");
      }
      if(data->data[2] & CAN_ERR_PROT_TX) {
        fprintf(fd, "error occurred on transmission, ");
      }
      fprintf(fd, "at ");
      switch(data->data[3]){
        default:
        case CAN_ERR_PROT_LOC_UNSPEC:
          fprintf(fd, "unspecified ");
          break;
        case CAN_ERR_PROT_LOC_SOF:
          fprintf(fd, "start of frame ");
          break;
        case CAN_ERR_PROT_LOC_ID28_21:
          fprintf(fd, "ID bits 28 - 21 (SFF: 10 - 3) ");
          break;
        case CAN_ERR_PROT_LOC_ID20_18:
          fprintf(fd, "ID bits 20 - 18 (SFF: 2 - 0 ) ");
          break;
        case CAN_ERR_PROT_LOC_SRTR:
          fprintf(fd, "substitute RTR (SFF: RTR) ");
          break;
        case CAN_ERR_PROT_LOC_IDE:
          fprintf(fd, "identifier extension ");
          break;
        case CAN_ERR_PROT_LOC_ID17_13:
          fprintf(fd, "ID bits 17-13 ");
          break;
        case CAN_ERR_PROT_LOC_ID12_05:
          fprintf(fd, "ID bits 12-5 ");
          break;
        case CAN_ERR_PROT_LOC_ID04_00:
          fprintf(fd, "ID bits 4-0 ");
          break;
        case CAN_ERR_PROT_LOC_RTR:
          fprintf(fd, "RTR ");
          break;
        case CAN_ERR_PROT_LOC_RES1:
          fprintf(fd, "reserved bit 1 ");
          break;
        case CAN_ERR_PROT_LOC_RES0:
          fprintf(fd, "reserved bit 0 ");
          break;
        case CAN_ERR_PROT_LOC_DLC:
          fprintf(fd, "data length code ");
          break;
        case CAN_ERR_PROT_LOC_DATA:
          fprintf(fd, "data section ");
          break;
        case CAN_ERR_PROT_LOC_CRC_SEQ:
          fprintf(fd, "CRC sequence ");
          break;
        case CAN_ERR_PROT_LOC_CRC_DEL:
          fprintf(fd, "CRC delimiter ");
          break;
        case CAN_ERR_PROT_LOC_ACK:
          fprintf(fd, "ACK slot ");
          break;
        case CAN_ERR_PROT_LOC_ACK_DEL:
          fprintf(fd, "ACK delimiter ");
          break;
        case CAN_ERR_PROT_LOC_EOF:
          fprintf(fd, "end of frame ");
          break;
        case CAN_ERR_PROT_LOC_INTERM:
          fprintf(fd, "intermission ");
          break;
      }
    }
    if(data->can_id & CAN_ERR_CRTL) {
      fprintf(fd, ", ERROR State: Controller Problems: ");
      if(data->data[1] & CAN_ERR_CRTL_RX_OVERFLOW) {
        fprintf(fd, "Rx Overflow, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_TX_OVERFLOW) {
        fprintf(fd, "Tx Overflow, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_RX_WARNING) {
        fprintf(fd, "Rx Warning, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_TX_WARNING) {
        fprintf(fd, "Tx Warning, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_RX_PASSIVE) {
        fprintf(fd, "Rx Passive, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_TX_PASSIVE) {
        fprintf(fd, "Tx Passive, ");
      }
      if(data->data[1] & CAN_ERR_CRTL_ACTIVE) {
        fprintf(fd, "Active ");
      }
    }
    if(data->can_id & CAN_ERR_LOSTARB) {
      fprintf(fd, ", ERROR State: Lost Arbitration ");
    }
    if(data->can_id & CAN_ERR_TX_TIMEOUT) {
      fprintf(fd, ", ERROR State: Tx Timeout ");
    } 
    fprintf(fd, ", Tx Error Count: %u, Rx Error Count: %u ", data->data[6], data->data[7]);
  } else {
    fprintf(fd, ", DLC: %2u,", data->can_dlc);

    fprintf(fd, " Data: ");
    for(int i = 0; i < CAN_MAX_DLC; i++) {
      fprintf(fd, "%02x, ", data->data[i]);
    }

    fprintf(fd, "echo_id: %08x, ", data->echo_id);
    fprintf(fd, "channel: %2u, ", data->channel);
    fprintf(fd, "flags: %02x", data->flags);
    if(data->flags > 0) {
      fprintf(fd, " (");
      if(data->flags & HOST_FRAME_FLAG_OVERFLOW) {
        fprintf(fd, "HOST_FRAME_FLAG_OVERFLOW ");
      }
      if(data->flags & HOST_FRAME_FLAG_FD) {
        fprintf(fd, "HOST_FRAME_FLAG_FD ");
      }
      if(data->flags & HOST_FRAME_FLAG_BRS) {
        fprintf(fd, "HOST_FRAME_FLAG_BRS ");
      }
      if(data->flags & HOST_FRAME_FLAG_ESI) {
        fprintf(fd, "HOST_FRAME_FLAG_ESI ");
      }
      fprintf(fd, ")");
    }
    fprintf(fd, ", reserved: %02x, ", data->reserved);
  }

  va_list args;
  va_start(args, format);

  vfprintf(fd, format, args);
  va_end(args);

  fprintf(fd, "\n");
}

void print_host_frame_raw(struct host_frame *data) {
  printf("Raw: |     echo_id      |       can_id      |dlc | ch |flg | rs |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |\n");
  // printf("Raw:  xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx   xx");
  printf("Raw:  ");
  for(int i = 0; i < sizeof(struct host_frame); i++) {
    printf("%02x   ", ((uint8_t*)data)[i]);
  }
  printf("\n");
}

#define TX_TIMEOUT_LENGTH_MS  (8) //(50) // When we Tx we should see the message come back to us within this time threshold in ms.

// Checks to see if there is space to Tx.
// If we have space then we return a pointer to to the struct usb2can_tx_context, else we return NULL.
struct gsusb_tx_context* get_tx_context(struct gsusb_ctx *ctx, struct can_frame* frame) {
  for(uint32_t i = 0; i < GSUSB_MAX_TX_REQ; i++) {
    if(ctx->tx_context[i].echo_id == GSUSB_MAX_TX_REQ) {
      ctx->tx_context[i].can = ctx;
      ctx->tx_context[i].echo_id = i;
      ctx->tx_context[i].timestamp = millis() + TX_TIMEOUT_LENGTH_MS;  // Set a timestamp.
      ctx->tx_context[i].frame = malloc(sizeof(struct can_frame));
      memcpy(ctx->tx_context[i].frame, frame, sizeof(struct can_frame));
      return &ctx->tx_context[i];
    }
  }
  return NULL;
}

// Release the tx_echo_id
// If you pass an echo_id out of range it will be ignored.
int release_tx_context(struct gsusb_ctx *ctx, uint32_t tx_echo_id) {
  if(tx_echo_id == 0xFFFFFFFF) {
    return 0;
  } else if(tx_echo_id >= GSUSB_MAX_TX_REQ){
    return -2;
  } else if(tx_echo_id != ctx->tx_context[tx_echo_id].echo_id){
    return -1;
  } else if(tx_echo_id < GSUSB_MAX_TX_REQ) {
    ctx->tx_context[tx_echo_id].can = NULL;
    ctx->tx_context[tx_echo_id].echo_id = GSUSB_MAX_TX_REQ;
    free(ctx->tx_context[tx_echo_id].frame);
    ctx->tx_context[tx_echo_id].frame = NULL;  // House keeping
    ctx->tx_context[tx_echo_id].timestamp = 0; // House keeping
    return 1;
  }
  LOGE(TAG, "Unreachable code reached!, Line: %i\n", __LINE__);

  return 0;
}

// Go through the tx_contexts and check if the messages were sent within the specified time period. If not then we need to cancel the context.
void handleRetries(struct gsusb_ctx *ctx) {
  uint64_t now = millis();
  struct can_frame frame;
  for(uint32_t i = 0; i < GSUSB_MAX_TX_REQ; i++) {
    if((ctx->tx_context[i].echo_id < GSUSB_MAX_TX_REQ) && (now > ctx->tx_context[i].timestamp)) {
      memcpy(&frame, ctx->tx_context[i].frame, sizeof(struct can_frame));
      release_tx_context(ctx, ctx->tx_context[i].echo_id);
    }
  }
}

int read_packet(struct gsusb_ctx *ctx, struct can_frame* frame) {
  int reply = GSUSB_ERROR_TIMEOUT;
  struct host_frame data;
  memset(&data, 0, sizeof(data));
  int len = 0;
  int ret = libusb_bulk_transfer(ctx->devh, ENDPOINT_IN, (uint8_t*) &data, sizeof(data), &len, 1);
  if(ret == 0) {
    if(len != sizeof(data)) {
      LOGE(TAG, "Size mismatch! sizeof(data) = %lu, len = %u, ret = %x", sizeof(data), len, ret);
      print_host_frame_raw(&data);
      fflush(stdout);
      reply = GSUSB_ERROR_GENERAL;
    }

    if(data.can_id & CAN_ERR_FLAG) {
      print_host_frame(TAG, &data, 1, "");
      print_host_frame_raw(&data);
    } else if((data.channel >= GSUSB_MAX_CHANNELS) || (data.can_dlc > CAN_MAX_DLC)) {
      print_host_frame(TAG, &data, 1, "");
      print_host_frame_raw(&data);
    } else {
      int tmp1 = release_tx_context(ctx, le32toh(data.echo_id));
      if(tmp1 > 0) {
        print_host_frame(TAG, &data, 0, "Context Released");
      } else if(tmp1 == 0) {
        print_host_frame(TAG, &data, 0, "");
      } else if(tmp1 == -2) {
        print_host_frame(TAG, &data, 1, "echo_id: %08x (%u) is invalid! TOO LARGE - ERROR!.", data.echo_id, data.echo_id);

        print_host_frame_raw(&data);
        fflush(stdout);
      } else if(tmp1 == -1) {
        // print_host_frame("CAN", "IN", &data, 1, "Context Error");
        print_host_frame(TAG, &data, 1, "echo_id %08x (%u) is invalid! MISMATCH with %08x (%u). - ERROR!.", data.echo_id, data.echo_id, ctx->tx_context[data.echo_id].echo_id, ctx->tx_context[data.echo_id].echo_id);

        print_host_frame_raw(&data);
        fflush(stdout);
      } else if(tmp1 < 0) {
        print_host_frame(TAG, &data, 1, "Context Error");

        print_host_frame_raw(&data);
        fflush(stdout);
      }

      // struct can_frame frame;

      frame->can_id = le32toh(data.can_id);

      frame->len = data.can_dlc;
      if(frame->len > CAN_MAX_DLC) {
        frame->len = CAN_MAX_DLC;
      }

      for(int i = 0; i < CAN_MAX_DLC; i++) {
        if(i < frame->len) {
          frame->data[i] = data.data[i];
        } else {
          frame->data[i] = 0x00;
        }
      }

      reply = GSUSB_OK;
    }
  } else if(ret != LIBUSB_ERROR_TIMEOUT) {
    reply = GSUSB_ERROR_READING;
    switch(ret) {
    case LIBUSB_ERROR_TIMEOUT:
      // print_host_frame("CAN", "IN", &data, 1, "LIBUSB_ERROR_TIMEOUT");
      reply = GSUSB_ERROR_TIMEOUT;
      break;
    case LIBUSB_ERROR_PIPE:
      print_host_frame(TAG, &data, 1, "LIBUSB_ERROR_PIPE");
      break;
    case LIBUSB_ERROR_OVERFLOW:
      print_host_frame(TAG, &data, 1, "LIBUSB_ERROR_OVERFLOW");
      break;
    case LIBUSB_ERROR_NO_DEVICE:
      print_host_frame(TAG, &data, 1, "LIBUSB_ERROR_NO_DEVICE");
      reply = GSUSB_ERROR_NO_DEVICE;
      break;
    case LIBUSB_ERROR_BUSY:
      print_host_frame(TAG, &data, 1, "LIBUSB_ERROR_BUSY");
      break;
    case LIBUSB_ERROR_INVALID_PARAM:
      print_host_frame(TAG, &data, 1, "LIBUSB_ERROR_INVALID_PARAM");
      break;
    default:
      print_host_frame(TAG, &data, 1, "UKNOWN (0x%08x)", ret);
      break;
    }
    fflush(stdout);
  }

  return reply;
}

int gsusbRead(struct gsusb_ctx *ctx, struct can_frame* frame) {
  int max = 0;
  int rep = GSUSB_ERROR_TIMEOUT;
  while((GSUSB_ERROR_TIMEOUT == rep) && (max < GSUSB_MAX_RX_REQ)) {
    rep = read_packet(ctx, frame);
    max++;
  }

  if(GSUSB_ERROR_NO_DEVICE == rep) {
    LOGE(TAG, "No CAN device!");
  }
  handleRetries(ctx);
  return rep;
}

int send_packet(struct gsusb_ctx *ctx, struct can_frame* frame) {

  struct gsusb_tx_context* tx_context = get_tx_context(ctx, frame);
  if(tx_context == NULL) {
    // print_can_frame("Q", "OUT", frame, 1, "BUSY");
    return LIBUSB_ERROR_BUSY;
  }

  struct host_frame data = {
    .echo_id = htole32(tx_context->echo_id),
    .can_id = htole32(frame->can_id),
    .can_dlc = frame->len,
    .channel = 0,
    .flags = 0,
    .reserved = 0
  };
  if(frame->can_id > 0x03ff) {
    data.can_id |= CAN_EFF_FLAG; // Set the extended bit flag (if not already set)
  }
  for(int i = 0; i < CAN_MAX_DLEN; i++) {
    if(i < frame->len) {
      data.data[i] = frame->data[i];
    } else {
      data.data[i] = 0;
    }
  }

  int len = 0;
  int ret = libusb_bulk_transfer(ctx->devh, ENDPOINT_OUT, (uint8_t*) &data, sizeof(data), &len, 1);
  if((len != sizeof(data)) && (ret != LIBUSB_ERROR_TIMEOUT)) {
    // print_can_frame(TAG, frame, 1, "ERROR");
    LOGE(TAG, "Size mismatch! sizeof(data) = %lu, len = %u, ret = %x \n", sizeof(data), len, ret);
    print_host_frame_raw(&data);
    fflush(stdout);
  }
  if(ret == 0) {
    // print_can_frame(TAG, frame, 0, "SUCCESS");
    print_host_frame(TAG, &data, 0, "Tx Queue");
    fflush(stdout);
  } else if (ret == LIBUSB_ERROR_TIMEOUT) {
    // print_can_frame(TAG, frame, 1, "TIMEOUT");
  } else {
    // print_can_frame(TAG, frame, 1, "ERROR");
    print_host_frame(TAG, &data, 1, "%s: %s\n", libusb_error_name(ret), libusb_strerror(ret));
    print_host_frame_raw(&data);
    fflush(stdout);
  }
  return ret;
}

int gsusbWrite(struct gsusb_ctx *ctx, struct can_frame* frame) {
  int ret = send_packet(ctx, frame);
  switch(ret) {
    case 0:
      return GSUSB_OK;
    case LIBUSB_ERROR_TIMEOUT:
      return GSUSB_ERROR_TIMEOUT;
    default:
      return GSUSB_ERROR_WRITING;
  }
}