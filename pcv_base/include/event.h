#ifndef EVENT_H
#define EVENT_H

/* standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* event definitions */
enum event_type
{
	NMT_EC_REC,
	STATUS_WRD_REC,
	SDO_WR_ACK,
	TIMEOUT,
	NEW_Xd_COMMAND,
	NEW_Yd_COMMAND,
	NEW_THETAd_COMMAND,
	BUTTON_PRESSED,
	NO_EVENT
};

/* event object declaration, the type is used to pass interesting data along 
   with the event if it is needed */
struct event
{
	enum event_type type;
	uint32_t param;
};

#endif