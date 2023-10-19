#ifndef __CAN_H__
#define __CAN_H__

#include "libusb.h"

// Max payload & DLC definitions accroding to ISO 11898-1
#define CAN_MAX_DLEN	8
#define CAN_MAX_DLC		8 	// Note DLC is NOT the same as DLEN. CANFD uses short codes to define lengths of 

// Special address flags for the CAN_ID
#define CAN_EFF_FLAG	0x80000000U	// EFF/SFF (extended frame format or standard frame format)
#define CAN_RTR_FLAG	0x40000000U	// Remote Transmission Request
#define CAN_ERR_FLAG	0x20000000U	// Error message frame

// Valid bits in the CAN_ID for frame formats
#define CAN_SFF_MASK	0x000007FFU	// Standard Frame Format (SFF)
#define CAN_EFF_MASK	0x1FFFFFFFU	// Extended Frame Format (EFF)
#define CAN_ERR_MASK	0x1FFFFFFFU	// Error Message Frame

#define CAN_ERR_DLC		8  // DLC for error message frames.

// Error class mask in can_id
#define CAN_ERR_TX_TIMEOUT 0x00000001U   // TX timeout (by netdevice driver)
#define CAN_ERR_LOSTARB	   0x00000002U   // lost arbitration    / data[0]   
#define CAN_ERR_CRTL	   0x00000004U   // controller problems / data[1]   
#define CAN_ERR_PROT	   0x00000008U   // protocol violations / data[2..3]
#define CAN_ERR_TRX		   0x00000010U   // transceiver status  / data[4]
#define CAN_ERR_ACK		   0x00000020U   // received no ACK on transmission
#define CAN_ERR_BUSOFF	   0x00000040U   // bus off
#define CAN_ERR_BUSERROR   0x00000080U   // bus error (may flood!)
#define CAN_ERR_RESTARTED  0x00000100U   // controller restarted
#define CAN_ERR_CNT  	   0x00000200U   // error counts (data[6] = Tx error counter, data[7] = Rx error counter)

// error status of CAN-controller / data[1]
#define CAN_ERR_CRTL_UNSPEC      0x00 // unspecified
#define CAN_ERR_CRTL_RX_OVERFLOW 0x01 // RX buffer overflow
#define CAN_ERR_CRTL_TX_OVERFLOW 0x02 // TX buffer overflow
#define CAN_ERR_CRTL_RX_WARNING  0x04 // reached warning level for RX errors
#define CAN_ERR_CRTL_TX_WARNING  0x08 // reached warning level for TX errors
#define CAN_ERR_CRTL_RX_PASSIVE  0x10 // reached error passive status RX
#define CAN_ERR_CRTL_TX_PASSIVE  0x20 // reached error passive status TX (at least one error counter exceeds the protocol-defined level of 127)
#define CAN_ERR_CRTL_ACTIVE      0x40 // recovered to error active state

// error in CAN protocol (type) / data[2]
#define CAN_ERR_PROT_UNSPEC      0x00 // unspecified
#define CAN_ERR_PROT_BIT         0x01 // single bit error
#define CAN_ERR_PROT_FORM        0x02 // frame format error
#define CAN_ERR_PROT_STUFF       0x04 // bit stuffing error
#define CAN_ERR_PROT_BIT0        0x08 // unable to send dominant bit
#define CAN_ERR_PROT_BIT1        0x10 // unable to send recessive bit
#define CAN_ERR_PROT_OVERLOAD    0x20 // bus overload
#define CAN_ERR_PROT_ACTIVE      0x40 // active error announcement
#define CAN_ERR_PROT_TX          0x80 // error occurred on transmission

// error in CAN protocol (location) / data[3]
#define CAN_ERR_PROT_LOC_UNSPEC  0x00 // unspecified
#define CAN_ERR_PROT_LOC_SOF     0x03 // start of frame
#define CAN_ERR_PROT_LOC_ID28_21 0x02 // ID bits 28 - 21 (SFF: 10 - 3)
#define CAN_ERR_PROT_LOC_ID20_18 0x06 // ID bits 20 - 18 (SFF: 2 - 0 )
#define CAN_ERR_PROT_LOC_SRTR    0x04 // substitute RTR (SFF: RTR)
#define CAN_ERR_PROT_LOC_IDE     0x05 // identifier extension
#define CAN_ERR_PROT_LOC_ID17_13 0x07 // ID bits 17-13
#define CAN_ERR_PROT_LOC_ID12_05 0x0F // ID bits 12-5
#define CAN_ERR_PROT_LOC_ID04_00 0x0E // ID bits 4-0
#define CAN_ERR_PROT_LOC_RTR     0x0C // RTR
#define CAN_ERR_PROT_LOC_RES1    0x0D // reserved bit 1
#define CAN_ERR_PROT_LOC_RES0    0x09 // reserved bit 0
#define CAN_ERR_PROT_LOC_DLC     0x0B // data length code
#define CAN_ERR_PROT_LOC_DATA    0x0A // data section
#define CAN_ERR_PROT_LOC_CRC_SEQ 0x08 // CRC sequence
#define CAN_ERR_PROT_LOC_CRC_DEL 0x18 // CRC delimiter
#define CAN_ERR_PROT_LOC_ACK     0x19 // ACK slot
#define CAN_ERR_PROT_LOC_ACK_DEL 0x1B // ACK delimiter
#define CAN_ERR_PROT_LOC_EOF     0x1A // end of frame
#define CAN_ERR_PROT_LOC_INTERM  0x12 // intermission

// error status of CAN-transceiver / data[4]
//                                             CANH CANL
#define CAN_ERR_TRX_UNSPEC             0x00 // 0000 0000
#define CAN_ERR_TRX_CANH_NO_WIRE       0x04 // 0000 0100
#define CAN_ERR_TRX_CANH_SHORT_TO_BAT  0x05 // 0000 0101
#define CAN_ERR_TRX_CANH_SHORT_TO_VCC  0x06 // 0000 0110
#define CAN_ERR_TRX_CANH_SHORT_TO_GND  0x07 // 0000 0111
#define CAN_ERR_TRX_CANL_NO_WIRE       0x40 // 0100 0000
#define CAN_ERR_TRX_CANL_SHORT_TO_BAT  0x50 // 0101 0000
#define CAN_ERR_TRX_CANL_SHORT_TO_VCC  0x60 // 0110 0000
#define CAN_ERR_TRX_CANL_SHORT_TO_GND  0x70 // 0111 0000
#define CAN_ERR_TRX_CANL_SHORT_TO_CANH 0x80 // 1000 0000

// CAN Identifier structure
// bit 0-28	: CAN identifier (11/29 bit)
// bit 29	: error message frame flag (0 = data frame, 1 = error message)
// bit 30	: Remote Transmission Request ( 1 = RTR frame)
// bit 31	: Frame Format Flag (0 = Standard Frame Format, 1 = Extended Frame Format)
typedef uint32_t canid_t;

struct can_frame {
	canid_t	can_id;	// 32-bit CAN_ID + EFF/RTR/ERR flags
	uint8_t	len;	// frame payload length in bytes (0 to CAN_MAX_DLEN)
	uint8_t	__pad;	// padding
	uint8_t	__res0;	// reserved / padding
	uint8_t	__res1;	// reserved / padding
	uint8_t	data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

#endif // __CAN_H__