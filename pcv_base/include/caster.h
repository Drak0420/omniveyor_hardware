#ifndef CASTER_H
#define CASTER_H

/* standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <utility>
#include "../include/motor.h"

/* uncomment this line and run "make test" to build the test harness for
   this module */
//#define TEST_CASTER
//#define TEST_TORQUE
//#define TEST_VELOCITY

/* Module for the interface to the caster modules */

class Caster 
{
	public:
		/* Constructor. Caster number starts at 1 */
		Caster(int casterNum);

		/* Destructor */
		~Caster();

		/* intializes motors through homing */
		int init();  
		
		/* Enable caster motors */
		int enable(); 
		
		/* Disable caster motors */
		int disable();
		
		/* Stop caster motors */
		int stop();   
		int start();

		/* Set control mode to velocity or torque control.
		 * This can only be set when the motors are disabled.
		 */
		int setCtrlMode(enum ctrl_mode cm);
		
		/* Set caster motor velocities in rad/s */
		int setVelocities(double steerVel, double rollVel);
		
		/* Set caster motor torques in Nm*/
		int setTorques(double steerTorque, double rollTorque);

		/* Get caster number */
		int getNumber() const;

		/* Get control mode (velocity/torque) */
		enum ctrl_mode getCtrlMode() const;
		
		/* Get steering angle of caster wheel in rad */
		double getSteerPosition();

		/* Get change in steer angle of caster wheel in rad */
		double getSteerDeltaQ();

		/* Get change in steer angle of caster wheel in rad */
		double getRollDeltaQ(); 
		
		/* Get steer and roll motor velocities in rad/s 
 		 * Returns pair with first = steer, second = roll
 		 */
		std::pair<double,double> getVelocities();

		/* Get steer and roll motor velocities in rad/s 
 		 * Returns pair with first = steer, second = roll
 		 */
		std::pair<double,double> getTorques();

		void getAmps(double *steerMotorAmp, double *rollMotorAmp);
		void getVolts(double *steerMotorVolt, double *rollMotorVolt);

		/* Returns true if the bumper is hit */
		bool getBumperState();

		/* Get initialization status of caster motors */
		bool isInitialized() const;
		
		/* Get enable status of caster motors */
		bool isEnabled() const;
		
		/* Get stopped status of caster motors */
		bool isStopped() const;

	private:
		int number;
		struct motor *steerMotor;
		struct motor *rollMotor;
		bool initialized;
		bool enabled;
		bool stopped;
		enum ctrl_mode control;
		double lastSteerMotorPos;
		double lastRollMotorPos;
		double steerPos;
		double lastRollPos;
		double steerVel;
		double rollVel;
		double steerTorque;
		double rollTorque;
};

#endif
