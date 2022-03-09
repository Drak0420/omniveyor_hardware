/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include "../include/COB_ID.h"
#include "../include/CO_message.h"

/*-----------------------------defines---------------------------------------*/

/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/
static void CO_SDO_Read_Obj (int s, uint8_t motor_no, const struct SDO_message *msg);
static void CO_SDO_Write_Obj (int s, uint8_t motor_no, const struct SDO_message *msg);
static void CO_Send_PDO (int s, uint8_t motor_no, const struct PDO_message *msg);
static void CO_Send_NMT (int s, uint8_t motor_no, const struct NMT_message *msg);
static void send_frame (int s, uint16_t cob_id, uint8_t dlc, uint8_t data[]);
static void CO_Send_SYNC (int s);

/*---------------------------public functions--------------------------------*/
void 
CO_send_message (int s, uint8_t motor_no, const struct CO_message *msg)
{
	switch (msg->type)
	{
		case SDO_Rx:
			CO_SDO_Write_Obj (s, motor_no, &msg->m.SDO);
			break;

		case SDO_Tx:
			CO_SDO_Read_Obj (s, motor_no, &msg->m.SDO);
			break;

		case RPDO:
			CO_Send_PDO (s, motor_no, &msg->m.PDO);
			break;

		case TPDO:
			puts ("PDO_Tx not implemented");
			break;

		case NMT:
			CO_Send_NMT (s, motor_no, &msg->m.NMT);
			break;

		case SYNC:
			CO_Send_SYNC (s);
			break;
	}
}

/*---------------------------static functions--------------------------------*/
static void 
CO_SDO_Read_Obj (int s, uint8_t motor_no, const struct SDO_message *msg)
{
	uint8_t *p_ind = (uint8_t *)&msg->index;
	uint8_t data[] = {CO_READ_Req, p_ind[0], p_ind[1], msg->subindex, 0, 0, 0, 0};

	send_frame (s, COB_ID_SDO_RX (motor_no), 8, data);
}

static void 
CO_SDO_Write_Obj (int s, uint8_t motor_no, const struct SDO_message *msg)
{
	uint8_t *p_ind = (uint8_t *)&msg->index;
	uint8_t *p_data = (uint8_t *)&msg->data;
	uint8_t command_byte;

	if (msg->sz_data == 4)
		command_byte = CO_WRITE_Req_W;
	else if (msg->sz_data == 2)
		command_byte = CO_WRITE_Req_HW;
	else if (msg->sz_data == 1)
		command_byte = CO_WRITE_Req_B;
	else
		perror ("wrong data size for CO_SDO_Write\n");

	uint8_t data[] = {command_byte, p_ind[0], p_ind[1], msg->subindex, 
														  p_data[0], p_data[1], p_data[2], p_data[3]};
	
	send_frame (s, COB_ID_SDO_RX (motor_no), 4+msg->sz_data, data);
}

static void
CO_Send_PDO (int s, uint8_t motor_no, const struct PDO_message *msg)
{
	uint8_t *p_data = (uint8_t *)&msg->data;
	send_frame (s, COB_ID_RPDO (motor_no, msg->PDO_Num), msg->sz_data, p_data);
}

static void 
CO_Send_NMT (int s, uint8_t motor_no, const struct NMT_message *msg)
{
	uint8_t data[] = {msg->data, motor_no};
	send_frame (s, COB_ID_NMT_RX, 2, data);
}

static void 
CO_Send_SYNC (int s)
{
	uint8_t data[] = {0};
	send_frame (s, COB_ID_SYNC, 0, data);
}

static void 
send_frame (int s, uint16_t cob_id, uint8_t dlc, uint8_t data[])
{
	struct can_frame frame;

	frame.can_id  = cob_id;
	frame.can_dlc = dlc;
	memcpy (frame.data, data, dlc);
	write(s, &frame, sizeof(struct can_frame));
}

void
CO_Set_bitrate (int s, unsigned int br)
{
    uint8_t ind;
    switch (br){
        case 125000:
            ind = 4;
            break;
        case 250000:
            ind = 3;
            break;
        case 500000:
            ind = 2;
            break;
        default:
            ind = 0;
            break;
    }
    uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, ind, 0x00, 0x13};
    send_frame(s, 0X7E5, 8, cmd);
    cmd[6] = 0x00;
    cmd[5] = 0x0A;
    send_frame(s, 0X7E5, 8, cmd);
    usleep(20000);
}

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif
