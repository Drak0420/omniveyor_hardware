#ifndef CAN_UTILS_H
#define CAN_UTILS_H

/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

/* standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

/* functions for CAN applications */
int create_can_socket (uint16_t can_id, uint16_t filter_mask);

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif