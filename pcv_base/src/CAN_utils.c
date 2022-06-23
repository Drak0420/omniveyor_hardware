/*-----------------------------includes--------------------------------------*/
/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <string.h>
#include "../include/CAN_utils.h"

/*-----------------------------defines---------------------------------------*/

/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/

/*---------------------------public functions--------------------------------*/
int 
create_can_socket (uint16_t can_id, uint16_t filter_mask)
{
	int s;
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	struct can_frame frame;
	struct ifreq ifr;
	struct can_filter cf;
	const char *ifname = "can0";

	/* open the socket */
	if ((s = socket (PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
	{
		perror ("Error while opening socket\n");
		return -1;
	}

	/* create the can filter for this motor no */
	cf.can_id = can_id;
	cf.can_mask = filter_mask;
	setsockopt (s, SOL_CAN_RAW, CAN_RAW_FILTER, &cf, sizeof (cf));
	int bufSize = 8192;
	setsockopt (s, SOL_CAN_RAW, SO_SNDBUF, &bufSize, sizeof(int));	//was 1000000
	setsockopt (s, SOL_CAN_RAW, SO_RCVBUF, &bufSize, sizeof(int));	//was 1000000
	strcpy (ifr.ifr_name, ifname);
	ioctl (s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof (addr)) < 0)
	{
		perror ("Error in socket bind\n");
		return -1;
	}

	return s;
}



/*---------------------------static functions--------------------------------*/

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif
