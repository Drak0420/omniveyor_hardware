//#define COMPILE
#ifdef COMPILE
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
#include <signal.h>
#include <time.h>

#include "../include/COB_ID.h"
#include "../include/CO_objects.h"
#include "../include/CO_message.h"
#include "../include/RT_utils.h"
#include "../include/motor_init_sequence.h"
#include "../include/motor.h"

#define ENCODER_TICKS 8192
#define REV_PER_RAD 100/628
#define VEL_MULTIPLIER 65536
#define SLOW_LOOP_SAMP_PERIOD 1/2000

static void 
start_motor (int s, uint8_t motor_num)
{
	int32_t speed = 0*(4 * Ns * ENCODER_TICKS * 
					   REV_PER_RAD * VEL_MULTIPLIER * SLOW_LOOP_SAMP_PERIOD); // first num in rad/s

	/* loop over initialization sequence */
	puts ("init sequence");
	unsigned i;
	for (i = 0; i < NUM_INIT_STEPS; i++)
	{
		struct CO_message cur = init_sequence[i];

		/* make sure to add the motor base to the data field for PDO inits */
		if (cur.type == SDO_Rx && cur.m.SDO.index == RPDO1_COMM &&
			  cur.m.SDO.subindex == 0x01)
			cur.m.SDO.data += motor_num;
		if (cur.type == SDO_Rx && cur.m.SDO.index == RPDO2_COMM &&
			  cur.m.SDO.subindex == 0x01)
			cur.m.SDO.data += motor_num;
		if (cur.type == SDO_Rx && cur.m.SDO.index == TPDO1_COMM &&
			  cur.m.SDO.subindex == 0x01)
			cur.m.SDO.data += motor_num;
		if (cur.type == SDO_Rx && cur.m.SDO.index == TPDO2_COMM &&
			  cur.m.SDO.subindex == 0x01)
			cur.m.SDO.data += motor_num;

		CO_send_message (s, motor_num, &cur);
		getchar ();
	}

	puts ("home sequence");
	for (i = 0; i < NUM_HOME_STEPS; i++)
	{
		struct CO_message cur = home_sequence[i];

		CO_send_message (s, motor_num, &cur);
		getchar ();
	}


	while (1)
	{
		/*TODO: send sync */


		speed += 5000;
	}
}

int
main(void)
{
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	const char *ifname = "can0";

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	int motor_num =1;
	start_motor (s, motor_num);
	printf("started motor: %u\n", motor_num);
	getchar ();
	
	return 0;
}

#endif