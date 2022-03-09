#ifndef CO_MESSAGE_H
#define CO_MESSAGE_H

/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* Defintions for CANopen command byte used in the first byte of the data
   in the CAN frame */
#define CO_WRITE_Req_W		(0x23)
#define CO_WRITE_Req_HW		(0x2B)
#define CO_WRITE_Req_B		(0x2F)
#define CO_WRITE_Ack		(0x60)
#define CO_READ_Req			(0x40)
#define CO_READ_Ack_W		(0x43)
#define CO_READ_Ack_HW		(0x4B)
#define CO_READ_Ack_B		(0x4F)

enum CO_msg_type
{
	SDO_Rx,
	SDO_Tx,
	RPDO,
	TPDO,
	NMT,
	SYNC
};

/* struct for sending a Service Data Object (SDO) message, data and sz_data fields are ignored 
   when attempting to read an object */
struct SDO_message
{
	uint16_t index;
	uint8_t subindex;
	uint32_t data;
	uint8_t sz_data;
};

/* struct for sending a Process Data Object (PDO) message */
struct PDO_message
{
	uint8_t PDO_Num;
	uint64_t data;
	uint8_t sz_data;
};

/* struct for sending an NMT message */
struct NMT_message
{
	uint8_t data;
};

/* union for the CO_Message object, the type is mutually exclusive so using
   a union is appropriate */
union message
{
	struct SDO_message  SDO;
	struct PDO_message  PDO;
	struct NMT_message  NMT;
};

struct CO_message
{
	enum CO_msg_type type;
	union message m;
};

/* function to send a CANopen message, only transmission is handled with
   this function, it does not check for proper reception of the message
   by the remote node, ,motor_no = 0 is a broadcast for NMT messages */
void CO_send_message (int s, uint8_t motor_no, const struct CO_message *msg);
void CO_Set_bitrate (int s, unsigned int br);

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif
