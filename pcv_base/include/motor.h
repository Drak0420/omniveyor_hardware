#ifndef MOTOR_H
#define MOTOR_H

/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment this line and run "make test" to build the test harness for
	this module */
// #define TEST_MOTOR
// #define TEST_TORQUE
// #define TEST_VELOCITY

/* Standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h>

/* Module for the interface to the technosoft motor drivers */

/* Opaque definition for the motor struct */
struct motor;


/* Enum for whether performing velocity or torque control, homing is used
	internally by the module and the user should not call init with this 
	value  */
enum ctrl_mode 
{
	HOMING,
    VELOCITY,
    TORQUE, 
    MANUAL_ZERO
};

/* Enum for the type of motor, either a steering motor, or 
	rolling motor */
enum motor_type
{
    ROLLING,
    STEERING
};

/* API */

/* Initializes the motor structure performing homing for steering motors 
	and exiting with the motor initialized but disabled. Returns NULL 
	on failure */
struct motor *motor_init (uint8_t motor_no, 
												  enum ctrl_mode cm, enum motor_type mt);

/* Interface to the motor velocity, get velocity returns -1 if no new 
	velocity data was received since the previous call to get velocity */
void motor_set_velocity (struct motor *m, double velocity);
int motor_get_velocity (struct motor *m, double *velocity);

/* Interface to the motor position, get position returns -1 if no new 
	position data was received since the previous call to get position */
int motor_get_position (struct motor *m, double *position);

/* Interface for enabling/disabling the motors */
int motor_enable (struct motor *m);
int motor_enable_fast (struct motor *m);
void motor_disable (struct motor *m);
void motor_disable_fast (struct motor *m);

/* Interface to motor torque */
void motor_set_torque (struct motor *m, double torque);
int motor_get_torque (struct motor *m, double *torque);

/* Interface to motor electrical status : current and voltage */
void motor_get_amp (struct motor *m, double *amp);
void motor_get_volt (struct motor *m, double *volt);

/* Interface to the motor control mode */
void motor_set_ctrl_mode (struct motor *m, enum ctrl_mode cm);
enum ctrl_mode motor_get_ctrl_mode (struct motor *m);

/* Query function for the type */
enum motor_type motor_get_type (struct motor *m);

/* Stop motor - utilizes the technosoft quick stop feature */
void motor_stop (struct motor *m);
void motor_start (struct motor *m);

/* Destroy the motor object - peforms all necessary cleanup on the motor
	object */
void motor_destroy (struct motor *m);

bool motor_get_inputs(struct motor *m);

bool is_motor_enable_pin_active(struct motor *m); 

/* cpp - c cross compilation */
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif
