#ifndef MOTOR_INIT_SEQUENCE_H
#define MOTOR_INIT_SEQUENCE_H

/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "./COB_ID.h"
#include "./CO_objects.h"
#include "./event.h"
#include "./hardware.h"

#define NUM_ACT_INIT_STEPS		(sizeof (act_init_sequence) / sizeof (act_init_sequence[0]))
#define NUM_COMM_INIT_STEPS     (sizeof (comm_init_sequence) / sizeof (comm_init_sequence[0]))
#define NUM_ENABLE_STEPS	(sizeof (enable_sequence) / sizeof (enable_sequence[0]))


/* Heartbeat */
#define HEARTBEAT_INTERVAL_ms	(50)

/* Homing */
#define HOME_SPEED_FAST			(0x100000)
#define HOME_SPEED_SLOW			(0x10000)
#define HOME_ACCEL				(0x199A)
#define HOME_METHOD				(19)

// Multipliers for unit conversion
#define POS_MULTIPLIER			(1)
#define VEL_MULTIPLIER			(65536)
#define ACCEL_MULTIPLIER		(65536)

/* modes of operation */
#define	HOME_MODE				(0x06)
#define VELOCITY_MODE			(0xFC) // -4
#define TORQUE_MODE				(0xFB) // -5

/* initialization sequence for the technosoft motors  */
const struct CO_message act_init_sequence[] = {
	{NMT, .m.NMT = 0x81},
	{SDO_Rx, .m.SDO = {POS_FACTOR, 0x01, POS_MULTIPLIER, 4}},					/* Position Factor Numerator */
	{SDO_Rx, .m.SDO = {POS_FACTOR, 0x02, 1, 4}},								/* Position Factor Denominator */
	{SDO_Rx, .m.SDO = {VEL_ENCODER_FACTOR, 0x01, VEL_MULTIPLIER, 4}},			/* Velocity Factor Numerator */
	{SDO_Rx, .m.SDO = {VEL_ENCODER_FACTOR, 0x02, 1, 4}},						/* Velocity Factor Denominator */
	{SDO_Rx, .m.SDO = {ACCEL_FACTOR, 0x01, ACCEL_MULTIPLIER, 4}},				/* Acceleration Factor Numerator */
	{SDO_Rx, .m.SDO = {ACCEL_FACTOR, 0x02, 1, 4}},							    /* Acceleration Factor Denominator */
    {SDO_Rx, .m.SDO = {EXT_REF_TYPE, 0x00, 1, 2}},								/* Set to external reference to on-line */
	{SDO_Rx, .m.SDO = {EXT_ONLINE_REF, 0x00, 0, 4}},							/* Set external reference to 0 */
	{SDO_Rx, .m.SDO = {HOME_OFFSET, 0x00, 0, 4}},								/* Set offset to get zero position from beam breaker (multiple of 45 deg) */
	{SDO_Rx, .m.SDO = {HOMING_METHOD, 0x00, HOME_METHOD, 1}},					/* Set method 19 for homing */
	{SDO_Rx, .m.SDO = {HOMING_SPEEDS, 0x01, HOME_SPEED_FAST, 4}},				/* Set fast homing speed (search for switch) to 1.53 rad/s */
	{SDO_Rx, .m.SDO = {HOMING_SPEEDS, 0x02, HOME_SPEED_SLOW, 4}},				/* Set fast homing speed (search for switch) to 0.096 rad/s */
	{SDO_Rx, .m.SDO = {HOMING_ACCEL, 0x00, HOME_ACCEL, 4}}};					/* Set homing acceleration to 6.74 rad/s^2 */

const struct CO_message comm_init_sequence[] = {
	//{NMT, .m.NMT = 0x82},
	{SDO_Rx, .m.SDO = {ABRT_COMM_OPT, 0x00, 0, 2}},								/* Disable Communication abortion */
	{SDO_Rx, .m.SDO = {FAULT_HANDLING, 0x00, -1, 2}},							/* Disable Communication abortion */
	
	{SDO_Rx, .m.SDO = {RPDO1_COMM, 0x01, COB_ID_RPDO (0,1) | (1 << 31), 4}}, 	/* Disable RPDO1 */
	{SDO_Rx, .m.SDO = {RPDO1_COMM, 0x02, 255, 1}},								/* Set RPDO1 transmission type to asynchronous (drive responds immediately) */
	{SDO_Rx, .m.SDO = {RPDO1_MAPPING, 0x00, 0, 1}},								/* Disable RPDO1 mapping */
	{SDO_Rx, .m.SDO = {RPDO1_MAPPING, 0x01, 0x60400010, 4}},					/* Map controlword to RPDO1 */
	{SDO_Rx, .m.SDO = {RPDO1_MAPPING, 0x00, 1, 1}},								/* Enable RPDO1 mapping (1 entry) */
	{SDO_Rx, .m.SDO = {RPDO1_COMM, 0x01, COB_ID_RPDO (0,1), 4}}, 				/* Enable RPDO1 */

	{SDO_Rx, .m.SDO = {RPDO2_COMM, 0x01, COB_ID_RPDO (0,2) | (1 << 31), 4}}, 	/* Disable RPDO2 */
	{SDO_Rx, .m.SDO = {RPDO2_COMM, 0x02, 255, 1}},  							/* Set RPDO2 transmission type to ASYNC */
	{SDO_Rx, .m.SDO = {RPDO2_MAPPING, 0x00, 0, 1}},								/* Disable RPDO2 mapping */
	{SDO_Rx, .m.SDO = {RPDO2_MAPPING, 0x01, 0x201C0020, 4}},					/* Map external online reference to RPDO2 */
	{SDO_Rx, .m.SDO = {RPDO2_MAPPING, 0x00, 1, 1}},								/* Enable RPDO2 mapping (1 entry) */
	{SDO_Rx, .m.SDO = {RPDO2_COMM, 0x01, COB_ID_RPDO (0,2), 4}}, 				/* Enable RPDO2 */

	{SDO_Rx, .m.SDO = {TPDO1_COMM, 0x01, COB_ID_TPDO (0,1) | (1 << 31), 4}}, 	/* Disable TPDO1 */
	{SDO_Rx, .m.SDO = {TPDO1_COMM, 0x02, 255, 1}},								/* Set TPDO1 transmission type to asynchronous (drive responds immediately on change of state) */
	{SDO_Rx, .m.SDO = {TPDO1_COMM, 0x03, 300, 2}},								/* Set TPDO1 Inhibit time to 30 ms (can only send 1 message per inhibit time) */
	{SDO_Rx, .m.SDO = {TPDO1_COMM, 0x05, 0, 2}},								/* Disable TPDO1 Event timer (timer for regular transmission) */
	{SDO_Rx, .m.SDO = {TPDO1_MAPPING, 0x00, 0, 1}},								/* Disable TPDO1 mapping */
	{SDO_Rx, .m.SDO = {TPDO1_MAPPING, 0x01, 0x60410010, 4}},					/* Map statusword to TPDO1 */
	{SDO_Rx, .m.SDO = {TPDO1_MAPPING, 0x00, 1, 1}},							    /* Enable TPDO1 mapping (1 entry) */
	{SDO_Rx, .m.SDO = {TPDO1_COMM, 0x01, COB_ID_TPDO (0,1), 4}}, 				/* Enable TPDO1 */

	{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x01, COB_ID_TPDO (0,2) | (1 << 31), 4}}, 	/* Disable TPDO2 */
	{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x02, 1, 1}},								/* Set TPDO2 transmission type to 1 (send on every 1st SYNC message) */
	//{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x02, 255, 1}},								/* Set TPDO2 transmission type to asynchronous (drive responds immediately on change of state) */
	//{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x03, 80, 2}},								/* Set TPDO2 Inhibit time to 8 ms (can only send 1 message per inhibit time) */
	{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x05, 20, 2}},								/* Set TPDO2 Event timer to 20 ms (timer for regular transmission) */
	{SDO_Rx, .m.SDO = {TPDO2_MAPPING, 0x00, 0, 1}},								/* Disable TPDO2 mapping */
	{SDO_Rx, .m.SDO = {TPDO2_MAPPING, 0x01, 0x60640020, 4}},					/* Map Position Actual Value to TPDO2 */
	{SDO_Rx, .m.SDO = {TPDO2_MAPPING, 0x02, 0x606C0020, 4}},					/* Map Velocity Actual Value to TPDO2 */
	{SDO_Rx, .m.SDO = {TPDO2_MAPPING, 0x00, 2, 1}},								/* Enable TPDO2 mapping (2 entries) */
	{SDO_Rx, .m.SDO = {TPDO2_COMM, 0x01, COB_ID_TPDO (0,2), 4}}, 				/* Enable TPDO2 */

	{SDO_Rx, .m.SDO = {TPDO3_COMM, 0x01, COB_ID_TPDO (0,3) | (1 << 31), 4}},    /* Disable TPDO3 */
    {SDO_Rx, .m.SDO = {TPDO3_COMM, 0x02, 1, 1}},								/* Set TPD03 transmission type to 1 (send on every 1st SYNC message) */
	//{SDO_Rx, .m.SDO = {TPDO3_COMM, 0x02, 255, 1}},								/* Set TPDO3 transmission type to asynchronous (drive responds immediately on change of state) */
	//{SDO_Rx, .m.SDO = {TPDO3_COMM, 0x03, 80, 2}},								/* Set TPDO3 Inhibit time to 8 ms (can only send 1 message per inhibit time) */
	{SDO_Rx, .m.SDO = {TPDO3_COMM, 0x05, 20, 2}},								/* Set TPDO3 Event timer to 20 ms (timer for regular transmission) */
    {SDO_Rx, .m.SDO = {TPDO3_MAPPING, 0x00, 0, 1}},								/* Disable TPD03 mapping */
    {SDO_Rx, .m.SDO = {TPDO3_MAPPING, 0x01, 0x207E0010, 4}},					/* Map current actual value to TPD03 */
    {SDO_Rx, .m.SDO = {TPDO3_MAPPING, 0x02, 0x20550010, 4}},					/* Changed to DC voltage indicator since control mode is never read from the drivers. //Map modes of operation display to TPD03 */ // Check this
    {SDO_Rx, .m.SDO = {TPDO3_MAPPING, 0x00, 2, 1}},								/* Enable TPD03 mapping (2 entries) */
    {SDO_Rx, .m.SDO = {TPDO3_COMM, 0x01, COB_ID_TPDO (0,3) , 4}},				/* Enable TPD03 */
    
    {SDO_Rx, .m.SDO = {TPDO4_COMM, 0x01, COB_ID_TPDO (0,4) | (1 << 31), 4}},    /* Disable TPD04 */  /*Since there is no bumper attached to digital inputs let's disable tpdo4 for now.*/
#ifdef BUMPER_SENSORS
	{SDO_Rx, .m.SDO = {TPDO4_COMM, 0x02, 255, 1}},								/* Set TPDO4 transmission type to asynchronous (drive responds immediately on change of state) */
	{SDO_Rx, .m.SDO = {TPDO4_COMM, 0x03, 300, 2}},								/* Set TPDO4 Inhibit time to 30 ms (can only send 1 message per inhibit time) */
	{SDO_Rx, .m.SDO = {TPDO4_COMM, 0x05, 0, 2}},								/* Disable TPDO4 Event timer (timer for regular transmission) */
    {SDO_Rx, .m.SDO = {TPDO4_MAPPING, 0x00, 0, 1}},								/* Disable TPD04 mapping */
	{SDO_Rx, .m.SDO = {TPDO4_MAPPING, 0x01, 0x60FD0020, 4}},                    /* Map digit inputs to TPDO4 */
    {SDO_Rx, .m.SDO = {TPDO4_MAPPING, 0x00, 1, 1}},								/* Enable TPD04 mapping (1 entry) */
    {SDO_Rx, .m.SDO = {TPDO4_COMM, 0x01, COB_ID_TPDO (0,4), 4}},				/* Enable TPD04 */
#endif
    {SDO_Rx, .m.SDO = {PROD_HEARTBEAT_TIME, 0x00, HEARTBEAT_INTERVAL_ms, 2}},	/* Set maximum time between drive heartbeats to 50 ms */    
    {NMT, 	 .m.NMT = 0x01}};													/* Set NMT State machine to Operational */

/* expected responses from motor driver during initialization */
const struct event act_init_responses[] = {
	{NMT_EC_REC, 0x0000},
	{SDO_WR_ACK, POS_FACTOR},
	{SDO_WR_ACK, POS_FACTOR},
	{SDO_WR_ACK, VEL_ENCODER_FACTOR},
	{SDO_WR_ACK, VEL_ENCODER_FACTOR},
	{SDO_WR_ACK, ACCEL_FACTOR},
	{SDO_WR_ACK, ACCEL_FACTOR},
	{SDO_WR_ACK, EXT_REF_TYPE},
	{SDO_WR_ACK, EXT_ONLINE_REF},
	{SDO_WR_ACK, HOME_OFFSET},
	{SDO_WR_ACK, HOMING_METHOD},
	{SDO_WR_ACK, HOMING_SPEEDS},
	{SDO_WR_ACK, HOMING_SPEEDS},
	{SDO_WR_ACK, HOMING_ACCEL}};

const struct event comm_init_responses[] = {
    //{NMT_EC_REC, 0x0000},
	{SDO_WR_ACK, ABRT_COMM_OPT},
	{SDO_WR_ACK, FAULT_HANDLING},

	{SDO_WR_ACK, RPDO1_COMM},
	{SDO_WR_ACK, RPDO1_COMM},
	{SDO_WR_ACK, RPDO1_MAPPING},
	{SDO_WR_ACK, RPDO1_MAPPING},
	{SDO_WR_ACK, RPDO1_MAPPING},
	{SDO_WR_ACK, RPDO1_COMM},

	{SDO_WR_ACK, RPDO2_COMM},
	{SDO_WR_ACK, RPDO2_COMM},
	{SDO_WR_ACK, RPDO2_MAPPING},
	{SDO_WR_ACK, RPDO2_MAPPING},
	{SDO_WR_ACK, RPDO2_MAPPING},
	{SDO_WR_ACK, RPDO2_COMM},

	{SDO_WR_ACK, TPDO1_COMM},
	{SDO_WR_ACK, TPDO1_COMM},
	{SDO_WR_ACK, TPDO1_COMM},
	{SDO_WR_ACK, TPDO1_COMM},
	{SDO_WR_ACK, TPDO1_MAPPING},
	{SDO_WR_ACK, TPDO1_MAPPING},
	{SDO_WR_ACK, TPDO1_MAPPING},
	{SDO_WR_ACK, TPDO1_COMM},

	{SDO_WR_ACK, TPDO2_COMM},
	{SDO_WR_ACK, TPDO2_COMM},
	//{SDO_WR_ACK, TPDO2_COMM},
	{SDO_WR_ACK, TPDO2_COMM},
	{SDO_WR_ACK, TPDO2_MAPPING},
	{SDO_WR_ACK, TPDO2_MAPPING},
	{SDO_WR_ACK, TPDO2_MAPPING},
	{SDO_WR_ACK, TPDO2_MAPPING},
	{SDO_WR_ACK, TPDO2_COMM},

    {SDO_WR_ACK, TPDO3_COMM},
    {SDO_WR_ACK, TPDO3_COMM},
	//{SDO_WR_ACK, TPDO3_COMM},
	{SDO_WR_ACK, TPDO3_COMM},
	{SDO_WR_ACK, TPDO3_MAPPING},
    {SDO_WR_ACK, TPDO3_MAPPING},
    {SDO_WR_ACK, TPDO3_MAPPING},
    {SDO_WR_ACK, TPDO3_MAPPING},
    {SDO_WR_ACK, TPDO3_COMM},

    {SDO_WR_ACK, TPDO4_COMM},
# ifdef BUMPER_SENSORS
    {SDO_WR_ACK, TPDO4_COMM},
    {SDO_WR_ACK, TPDO4_COMM},
    {SDO_WR_ACK, TPDO4_COMM},
	{SDO_WR_ACK, TPDO4_MAPPING},
    {SDO_WR_ACK, TPDO4_MAPPING},
    {SDO_WR_ACK, TPDO4_MAPPING},
    {SDO_WR_ACK, TPDO4_COMM},
#endif
    {SDO_WR_ACK, PROD_HEARTBEAT_TIME},
    {STATUS_WRD_REC, 0x0240}};


/* enable sequence */
const struct CO_message enable_sequence[] = {
	{SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x0006, 2}},							/* Set Drive State machine from switch on disabled to ready to switch on */
	{SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x0007, 2}},							/* Set Drive state machine from ready to switch on to switched on (motors still zero torque) */
    {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x000F, 2}}};						/* Set Drive state machine from switched on to operation enabled (motors able to apply torque) */

#define ENABLE_Rx_MASK				(0x006F)

/* expected responses from the motor driver when enabling the 
   motor. This status word should be masked with the ENABLE_Rx_MASK
   and then compared to these values */
const struct event enable_responses[] = {						
	{STATUS_WRD_REC, 0x0021},
	{STATUS_WRD_REC, 0x0023},
	{STATUS_WRD_REC, 0x0027}};

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif

