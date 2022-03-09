#ifndef VEHICLE_H
#define VEHICLE_H

/* uncomment this line and run "make test" to build the test harness for
   this module */
// #define TEST_VEHICLE

/* standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <utility>
#include "./motor.h"
#include "./caster.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <omniveyor_common/definitions.h>

typedef struct robotElectrical_T{
	double steerMotorCurrent [NUM_CASTERS];
	double steerMotorVoltage [NUM_CASTERS];
	double rollMotorCurrent [NUM_CASTERS];
	double rollMotorVoltage [NUM_CASTERS];
} robotElectrical_T;

class Vehicle 
{
	public:
		/* socket */
		int s;

		/* Constructor. Caster number starts at 1 */
		Vehicle();

		/* Destructor */
		~Vehicle();

		/* Intializes all casters */
		int init();  
		
		/* Enable all casters*/
		int enable(); 

		/* Disable all casters */
		int disable();
		
		/* Stop all motors */
		int stop(); 
		int start();

		/* Update the current x, y, and theta values for the vehicle */
		void updateOdometry();  
		
		/* Set control mode to velocity or torque control.
		 * This can only be set only when the motors are disabled. */
		int setCtrlMode(enum ctrl_mode cm);
		
		/* Set vehicle velocities in x, y, and theta. vx,vy in m/s and vth in rad/s */
		int setGlobalVelocity(Eigen::Vector3d xd_com_in);

		/* Set desired vehicle global position, velocity, and acceleration for torque controller */
		void setTargets(Eigen::Array3d gx_des, Eigen::Array3d gxd_des, Eigen::Array3d gxdd_des);
		
		/* set Kp gains */
		void setKp(Eigen::Array3d Kp_des);

		/* set Kv gains */
		void setKv(Eigen::Array3d Kv_des);

		/* Get control mode (velocity/torque) */
		enum ctrl_mode getCtrlMode() const;
		
		/* Get position in m/s for x and y and rad/s for theta */
		Eigen::Vector3d getGlobalPosition() const;
		
		Eigen::Vector3d getLocalPosition() const; 

		/* Get vehicle velocity in m/s for x and y and rad/s for theta */
		Eigen::Vector3d getGlobalVelocity() const;

		Eigen::Vector3d getLocalVelocity() const; 

		/* Get joint steering angles in rad */
		Eigen::Matrix<double, NUM_CASTERS, 1> getJointSteeringAngles() const;

		/* Get desired joint velocities in rad/s */
		Eigen::Matrix<double, NUM_MOTORS, 1> getDesJointVelocities() const;

		/* Get joint velocities in rad/s */
		Eigen::Matrix<double, NUM_MOTORS, 1> getJointVelocities() const;

		/* Get vehicle (operational space) forces in x and y in N and torque in theta in Nm */
		Eigen::Vector3d getLocalCommandForces() const;

		/* Get joint torques in Nm */
		Eigen::Matrix<double, NUM_MOTORS, 1> getDesJointTorques() const;

		Eigen::Matrix3d getLambda() const; 

		Eigen::MatrixXd getCPinv() const; 

		double getHeading() const;

		/* Returns a 4D vector containing the bumper state of each bumper */
		Eigen::Vector4d getBumperState() const;

		/* Return true if one or more bumper is hit */
		bool isBumperHit() const;
		
		/* Get initialization status of vehicle */
		bool isInitialized() const;
		
		/* Get enable status of vehicle */
		bool isEnabled() const;
		
		/* Get stopped status of vehicle */
		bool isStopped() const;

		bool reachedTarget(Eigen::Vector3d curr_pos, Eigen::Vector3d curr_target, double max_linear_dist, double max_rot_dist) const;
		
		void getElectricalStatus (robotElectrical_T *status);

	private:
		bool _initialized;
		bool _enabled;
		bool _stopped;
		enum ctrl_mode _control;

		Caster *casters[NUM_CASTERS];             		 		// array of casters
		
		Eigen::Vector3d _x_local;                             	// local position
		Eigen::Vector3d _gx;                             		// global position
		Eigen::Vector3d _gx_des;  								// last global operational space position command sent 
		Eigen::Vector3d _xd_local; 								// local operational space velocity 
		Eigen::Vector3d _gxd; 								 	// global operational space velocity 
		Eigen::Vector3d _gxd_com;								// command global operational space velocity (sent from outside)
		Eigen::Vector3d _gxd_des;								// last global operational space velocity command sent
		Eigen::Vector3d _gxdd;									// global operational space acceleration 
		Eigen::Vector3d _gxdd_des; 								// last global operational space acceleration command sent 
		
		Eigen::Matrix<double, NUM_CASTERS, 1> _q_steer; 	    // joint steering angles
		Eigen::Matrix<double, NUM_MOTORS,  1> _qd; 		 		// joint velocities
		Eigen::Matrix<double, NUM_MOTORS,  1> _qd_des; 	 	    // commanded joint velocities
		Eigen::Matrix<double, NUM_MOTORS,  1> _tq; 		 		// joint torques
		Eigen::Matrix<double, NUM_MOTORS,  1> _tq_des; 	 		// commanded joint torques
		
		Eigen::Vector3d _g_cf;    	 							// global control force
		Eigen::Vector3d _cf_des_local; 							// local commanded control force
		Eigen::Matrix3d _lambda; 								// vehicle mass matrix
		Eigen::Vector3d _mu;     					 			// velocity coupling vector (centripetal,coriolis)
		
		Eigen::Matrix<double, NUM_MOTORS, 3> _C; 		 		// constraint matrix
		Eigen::Matrix<double, NUM_MOTORS, 3> _Cp; 
		Eigen::Matrix<double, NUM_MOTORS, NUM_MOTORS> _Jq; 		 					
		Eigen::MatrixXd _C_pinv; 	 							// constraint matrix pseudoinverse (calculated -> dynamic size)		 	  		
		
		double _heading; 										// Vehicle heading (theta)

		Eigen::Array3d _Kp; 									// position gain (array for elementwise math)
		Eigen::Array3d _Kv;										// velocity gain (array for elementwise math)

		/* set operational space forces fx,fy in N and torque tz in Nm */
		int setTorque(const Eigen::Vector3d& cf_command);

		/* update joint data */
		void updateJointData();

		/* Get change in joint positions */
		Eigen::Matrix<double, NUM_MOTORS, 1> getDeltaQ();

		/* update constraint matrix */
		void updateConstraintMatrix();

		/* update lambda and mu */
		void updateDynamics();

		/* calculate A matrix */
		Eigen::Matrix3d calcAMatrix();

		/* calculate p vector */
		Eigen::Vector3d calcpVector(int caster_no);	

		/* calculate C_th matrix */
		Eigen::Matrix3d calcC_thMatrix(int caster_no);

		/* calculate J_dot matrix */
		Eigen::Matrix3d calcJ_dotMatrix(int caster_no);

		/* calculate lambda pumpkin matrix */
		Eigen::Matrix3d calcLambdaPumpkinMatrix(int caster_no);

		/* calculate lambda vehicle matrix */
		Eigen::Matrix3d calcLambdaVehicleMatrix();

		/* calculate Q_dot vector */
		Eigen::Vector3d calcQ_dotVector(int caster_no);	

		/* calculate mu pumpkin vector */
		Eigen::Vector3d calcMuPumpkinVector(int caster_no);	

		/* update Jq matrix */
		void updateJqMatrix();

		/* update Cp matrix */
		void updateCpMatrix();

		/* update constraint matrix pseudoinverse */
		void updateConstraintPinvMatrix();

		/* Saturate the torque values of the eight motors */
		void clampTorque(); 
};

#endif
