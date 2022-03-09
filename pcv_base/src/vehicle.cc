/* ------------Includes---------------------- */
#include <cmath>
#include <iostream>
#include <utility>
#include <unistd.h>
#include "../include/CAN_utils.h"
#include "../include/CO_message.h"
#include "../include/vehicle.h"
#include "../include/motor.h"
#include "../include/caster.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <omniveyor_common/definitions.h>

// #define VELOCITY_SATURATION 
// #define SEND_ZERO_TORQUES
#define CLAMP_TORQUES

using std::cout;
using std::endl; 

/* --------------------------- STATIC FUNCTION PROTOTYPES ------------------------------- */
static double casterPosXSign(int casterNum);
static double casterPosYSign(int casterNum);
static Eigen::Vector3d clampVelocity(Eigen::Vector3d xd_com_in);
static Eigen::Vector3d clampAcceleration(Eigen::Vector3d xdd_com_in);
static float saturate(float x); 

/* --------------------------- PUBLIC MEMBER FUNCTIONS ------------------------------- */

/* Constructor. Caster number starts at 1 */
Vehicle::Vehicle() 
{
	_initialized = false;
	_enabled = false;
	_stopped = false;
	_control = VELOCITY;

	_x_local.setZero(); 
	_xd_local.setZero();
	_gx.setZero(); 
	_gxd.setZero();
	_gxd_com.setZero();
	_gxd_des.setZero();
	_gxdd.setZero();

	_g_cf.setZero();
	_cf_des_local.setZero();

	_q_steer.setZero(); 
	_qd.setZero(); 
	_qd_des.setZero(); 
	_tq.setZero(); 
	_tq_des.setZero(); 
	_lambda.setZero(); 
	_mu.setZero(); 
	_C.setZero(); 
	_Cp.setZero(); 
	_Jq.setZero(); 
	_C_pinv.setZero(); 

	_heading = 0; 

	_Kp.setZero(); // zero gains for torque control initially
	_Kv.setZero(); // zero gains for torque control initially

	// create casters
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		casters[i] = new Caster(i+1);		
	} 

	/* create a socket for the control thread */
	s = create_can_socket (UNUSED_NODE_ID, 0xF);
	if (s < 0)
	{
		perror ("Unable to open control socket\n");
		exit (-1);
	}
	//CO_Set_bitrate(s, 500000);
}


/* Destructor */
Vehicle::~Vehicle()
{
	_initialized = false;
	/* quick stop to motors - wait (x amount of time)*/

	/* destroy caster objects */
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		delete casters[i];
	}
	/* reset comm */	
	struct CO_message msg;
	msg.type = NMT;
	msg.m.NMT.data = 0x81;
	CO_send_message (s, 0, &msg);
	shutdown(s,1);
	//close(s);
}


/* Intializes all casters */
int Vehicle::init() 
{
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->init())
		{
			return -1;
		}
	}
	_initialized = true;
	return 0;
}  


/* Enable all casters*/
int Vehicle::enable() 
{
	if(!_initialized)
	{
		return -1;
	}

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->enable())
		{
			return -1;
		}
	}

	_enabled = true;
	_stopped = false;
	return 0;
}


/* Disable all casters */
int Vehicle::disable()
{
	if(!_initialized)
	{
		return -1;
	}

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->disable())
		{
			return -1;
		}
	}

	_enabled = false;
	return 0;
}


/* Stop all motors */
// HAVEN'T GOTTEN THIS TO WORK YET - CAN USE WHATEVER METHOD YOU WANT
int Vehicle::stop() 
{
	if (!_enabled)
	{
		return -1;
	}

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->stop() == 0)
		{
			cout << "stopping caster " << i << endl;
			// return -1;
		}
	}

	_stopped = true; // wait for stop?
	return 0;
}

int Vehicle::start() 
{
	if (!_enabled)
	{
		return -1;
	}

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->start() == 0)
		{
			cout << "starting caster " << i << endl;
			// return -1;
		}
	}

	_stopped = false; // wait for stop?
	return 0;
}


/* Set control mode to velocity or torque control.
 * This can only be set when the motors are disabled.
 */
int Vehicle::setCtrlMode(enum ctrl_mode cm)
{
	if(_enabled && !_stopped)
	{
		return -1; // only set mode if disabled
	}

	// Set mode
	_control = cm;
	cout << "Setting vehicle control mode to : " << cm << endl;
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->setCtrlMode(cm))
		{
			return -1;
		}
	}

	return 0;
}

/* Update the local and global coordinates of the 
 * operational point of the mobile base
 */
void Vehicle::updateOdometry(){
	updateJointData();
	// Eigen::Matrix<double, NUM_MOTORS, 1> deltaQ = getDeltaQ();
	
	// Calculate C_p^+
	updateConstraintPinvMatrix();

	// Find velocity and position of the base in the frame of the base with respect to the world origin
	// _xd_local = _C_pinv * deltaQ / CONTROL_PERIOD_s;
	_xd_local = _C_pinv * _qd;
	_x_local += _xd_local * CONTROL_PERIOD_s;

	_heading = _gx(2)+ 0.5*_xd_local(2)*CONTROL_PERIOD_s;

	// Rotation matrix to convert vectors from local frame to global frame 
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	R(0, 0) =  cos(_heading);
    R(0, 1) = -sin(_heading);
    R(1, 0) =  sin(_heading);
    R(1, 1) =  cos(_heading);
    R(2, 2) =  1.0;
    
    _gxd = R * _xd_local;
    _gx += _gxd * CONTROL_PERIOD_s;
}


/* Returns the change in steer and roll angle */ 
Eigen::Matrix<double, NUM_MOTORS, 1> Vehicle::getDeltaQ(){
	Eigen::Matrix<double, NUM_MOTORS, 1> deltaQ;
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		deltaQ(2*i) = casters[i]->getSteerDeltaQ();
		deltaQ(2*i + 1) = casters[i]->getRollDeltaQ();
	}
	return deltaQ;
}


/* Set vehicle velocities in x, y, and theta. vx,vy in m/s and vth in rad/s */
// (6.1)
int Vehicle::setGlobalVelocity(Eigen::Vector3d xd_com_in)
{
	// Retrieve latest joint data
	updateJointData();

	// Calculate C
	updateConstraintMatrix();

	// Store velocity command and determine current xd_des (limit acceleration)
	_gxd_com = clampVelocity(xd_com_in);
	_gxd_des = _gxd_des + clampAcceleration(_gxd_com - _gxd_des);

	// Calculate joint velocities to command
	_qd_des = _C * _gxd_des;

	// Set velocities for each caster
	for(int i = 0; i < NUM_CASTERS; i++) 
	{
		if(casters[i]->setVelocities(_qd_des(2*i),_qd_des(2*i+1))) 
		{
			return -1;
		}
	}	

	return 0;
} 


/* Set desired vehicle position, velocity, and acceleration for torque controller */
// (6.4)
void Vehicle::setTargets(Eigen::Array3d gx_des_in, Eigen::Array3d gxd_des_in, Eigen::Array3d gxdd_des_in) 
{
	// update _lambda and mu
	updateDynamics();

	_gx_des   = gx_des_in; 
	_gxd_des  = gxd_des_in; 
	_gxdd_des = gxdd_des_in; 

	// Calculate and set control forces/torques
#ifdef VELOCITY_SATURATION
	Eigen::Array3d dxd; 
	dxd = _Kp / _Kv * (gx_des_in - _gx.array()); 
	Eigen::Array3d nu; 
	nu(0) = saturate(MAX_VEL_X/abs(dxd(0))); 
	nu(1) = saturate(MAX_VEL_Y/abs(dxd(1))); 
	nu(2) = saturate(MAX_VEL_TH/abs(dxd(2))); 
	Eigen::Vector3d g_cf_unit = -_Kv * (_gxd.array() - nu * dxd); 
#else 
	Eigen::Vector3d g_cf_unit = -_Kp * (_gx.array() - gx_des_in) - _Kv * (_gxd.array() - gxd_des_in) + gxdd_des_in;

#endif

	// Rotation matrix to convert vector from global frame to local frame
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	R(0, 0) = cos(_heading);
    R(0, 1) = sin(_heading);
    R(1, 0) = -sin(_heading);
    R(1, 1) = cos(_heading);
    R(2, 2) = 1.0;

	// Map global command to local coordinates 
	Eigen::Array3d cf_unit_local = R * g_cf_unit; 

	_cf_des_local = _lambda * cf_unit_local.matrix();
	// cf_des = _lambda * cf_unit.matrix() + mu;
	
	setTorque(_cf_des_local);
}	


/* set _Kp gains */
void Vehicle::setKp(Eigen::Array3d Kp_des){
	_Kp = Kp_des;
}

/* set Kv gains */
void Vehicle::setKv(Eigen::Array3d Kv_des){
	_Kv = Kv_des;
}

/* Get control mode (velocity/torque) */
enum ctrl_mode Vehicle::getCtrlMode() const 
{
	return _control;
}

/* Get position in m for x and y and rad for theta */
Eigen::Vector3d Vehicle::getGlobalPosition() const
{
	return _gx;
}

Eigen::Vector3d Vehicle::getLocalPosition() const 
{
	return  _x_local; 
}

/* Get vehicle velocity in m/s for x and y and rad/s for theta */
Eigen::Vector3d Vehicle::getGlobalVelocity() const 
{
	return _gxd;
}

Eigen::Vector3d Vehicle::getLocalVelocity() const 
{
	return _xd_local; 
}

/* Get joint steering angles in rad */
Eigen::Matrix<double, NUM_CASTERS, 1> Vehicle::getJointSteeringAngles() const
{
	// updateJointData();
	return _q_steer;
}

/* Get desired joint velocities in rad/s */
Eigen::Matrix<double, NUM_MOTORS, 1> Vehicle::getDesJointVelocities() const
{
	return _qd_des;
}

/* Get joint velocities in rad/s */
Eigen::Matrix<double, NUM_MOTORS, 1> Vehicle::getJointVelocities() const
{
	return _qd;
}

/* Get vehicle (operational space) forces in x and y in N and torque in theta in Nm */
Eigen::Vector3d Vehicle::getLocalCommandForces() const
{
	return _cf_des_local;
}


/* Get joint torques in Nm */
Eigen::Matrix<double, NUM_MOTORS, 1> Vehicle::getDesJointTorques() const 
{
	return _tq_des;
}

/* Get initialization status of vehicle */
bool Vehicle::isInitialized() const
{
	return _initialized;
}

/* Get enable status of vehicle */
bool Vehicle::isEnabled() const
{
	return _enabled;
}

/* Get status of vehicle */
bool Vehicle::isStopped() const
{
	return _stopped;
}

Eigen::Matrix3d Vehicle::getLambda() const 
{
	return _lambda; 
}

Eigen::MatrixXd Vehicle::getCPinv() const 
{
	return _C_pinv; 
}


/* Returns true if at least one bumper is hit */
bool Vehicle::isBumperHit() const
{
	bool retVal = false; 
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if (casters[i]->getBumperState()){
			retVal = true; 
		} 
	}
	return retVal; 
}

/* Returns a 4D vector containing the bumper state of each bumper */
Eigen::Vector4d Vehicle::getBumperState() const
{
	Eigen::Vector4d bumper_state; 
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		bumper_state[i] = casters[i]->getBumperState(); 
	}
	return bumper_state; 
}

double Vehicle::getHeading() const 
{
	return _heading; 
}

void Vehicle::getElectricalStatus (robotElectrical_T *status)
{
	for (int i=0; i<NUM_CASTERS; i++){
		casters[i]->getAmps(&(status->steerMotorCurrent[i]), &(status->rollMotorCurrent[i]));
		casters[i]->getVolts(&(status->steerMotorVoltage[i]), &(status->rollMotorVoltage[i]));
	}
}

bool Vehicle::reachedTarget(Eigen::Vector3d curr_pos, Eigen::Vector3d curr_target, double max_linear_dist, double max_rot_dist) const
{
	return ((fabs(curr_pos[0]-curr_target[0]) < max_linear_dist) && (fabs(curr_pos[1]-curr_target[1]) < max_linear_dist) && (fabs(curr_pos[2]-curr_target[2]) < max_rot_dist));
}

/* --------------------------- PRIVATE MEMBER FUNCTIONS ------------------------------- */

/* Input: operational space forces fx,fy (N) and torque tz (Nm), all in local frame */
/* Outputs torques to casters */
int Vehicle::setTorque(const Eigen::Vector3d& cf_command)
{
	// Retrieve latest joint data
	updateJointData();

	// Calculate C#
	//updateConstraintMatrix();

	// Calculate C_p^+
	updateConstraintPinvMatrix();

	// Calculate joint torques
	// (6.11)
	_tq_des = _C_pinv.transpose() * cf_command; 

#ifdef CLAMP_TORQUES
	clampTorque(); 
#endif

#ifdef SEND_ZERO_TORQUES
	_tq_des.setZero(); 
#endif

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		if(casters[i]->setTorques(_tq_des(2*i),_tq_des(2*i+1)))
		{
			return -1;
		}
	}	
	return 0;
}


/* Update joint data */
void Vehicle::updateJointData() 
{
	std::pair<double,double> jointVel;
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		_q_steer(i) = casters[i]->getSteerPosition();
		jointVel = casters[i]->getVelocities();
		_qd(2*i) = jointVel.first;
		_qd(2*i + 1) = jointVel.second;
	}
}


/* Mass Matrix taken from pg 20 of PCV Dyanmics.pdf*/
void Vehicle::updateDynamics()
{
	_lambda = Eigen::Matrix3d::Zero();
	_mu = Eigen::Vector3d::Zero();
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Calculate relevant matrices and vectors
		Eigen::Matrix3d C_th = calcC_thMatrix(i);
		Eigen::Matrix3d A = calcAMatrix();
		Eigen::Vector3d p = calcpVector(i);
		Eigen::Matrix3d J_dot = calcJ_dotMatrix(i);
		Eigen::Vector3d Q_dot = calcQ_dotVector(i);

		//Calculate lambda_pumpkin and mu_pumpkin	
		Eigen::Matrix3d lambda_pumpkin = calcLambdaPumpkinMatrix(i);
		Eigen::Vector3d mu_pumpkin = calcMuPumpkinVector(i);

		Eigen::Matrix3d lambda_i = C_th.transpose() * A * C_th + lambda_pumpkin;
		_lambda += lambda_i;

		Eigen::Vector3d mu_i = C_th.transpose()*(p - A*C_th*J_dot*Q_dot) + mu_pumpkin; 
 		_mu += mu_i;
	}
	_lambda += calcLambdaVehicleMatrix();
}


/* Update kinematic constraint matrix, C#. Assumes joint data is up to date.
   Converts from operational space (xdot) to joint space (qdot). See pg 27 PCV_Dyanmics.pdf*/
void Vehicle::updateConstraintMatrix() 
{
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		int steerRow = 2*i;
		int rollRow = 2*i + 1;

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

		// Take sin/cos of steeering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Compute steer row
		/**************** Alternative sign convention *******************/
		// C(steerRow,0) = -steerSin / PC_b;
		// C(steerRow,1) = steerCos / PC_b;
		// C(steerRow,2) = (hx * steerCos + hy * steerSin) / PC_b - 1.0;
		/****************************************************************/
		_C(steerRow,0) = steerSin / PC_b;
		_C(steerRow,1) = -steerCos / PC_b;
		_C(steerRow,2) = -(hx * steerCos + hy * steerSin) / PC_b - 1.0;

		// Compute roll row
		_C(rollRow,0) = steerCos / PC_r;
		_C(rollRow,1) = steerSin / PC_r;
		_C(rollRow,2) = (hx * steerSin - hy * steerCos) / PC_r;
	}
}


/* Update Moore-Penrose pseudoinverse of the constraint matrix C_p. Assumes constraint matrix, C#, is up to date */
void Vehicle::updateConstraintPinvMatrix() 
{
	// Update Cp matrix with current data
	updateCpMatrix(); 

	Eigen::MatrixXd CptCli = Eigen::MatrixXd::Zero(3, NUM_MOTORS);

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Cli IS 2x2 BLOCK DIAGONAL, SO MULTIPLY
	    // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS

		// Take sin/cos of steering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

	    int j = 2*i;

	    // Note: sign of p-dot is: wheel as viewed from ground
	    CptCli(0, j ) =  PC_b * steerSin;
	    CptCli(0,j+1) =  PC_r * steerCos;
	    CptCli(1, j ) = -PC_b * steerCos;
	    CptCli(1,j+1) =  PC_r * steerSin;
	    CptCli(2, j ) = -PC_b * ((hx * steerCos + hy * steerSin) + PC_b);
	    CptCli(2,j+1) =  PC_r * (hx * steerSin - hy * steerCos);
	}

	_C_pinv = (_Cp.transpose()*_Cp).llt().solve(CptCli);
}


Eigen::Matrix3d Vehicle::calcAMatrix(){
	Eigen::Matrix3d A;
	A(0, 0) = PC_Mf*pow(PC_e, 2) + PC_If + PC_Ii + PC_Ih + PC_Is*pow(Ns, 2) + PC_It*Nr;
	A(0, 1) = PC_Ii*Nw - PC_Ih*Nw - PC_It*pow(Nr, 2)*Nw;
	A(0, 2) = PC_Mf*pow(PC_e, 2) + PC_If + PC_Ii + PC_Ih - PC_Is*Ns - PC_It*Nr;

	A(1, 0) = A(0, 1); 
	A(1, 1) = PC_Mf*pow(PC_r, 2) + PC_Ii*pow(Nw, 2) + PC_Ih*pow(Nw, 2) + PC_It*pow(Nr, 2)*pow(Nw, 2) + PC_Ij;
	A(1, 2) = PC_Ii*Nw - PC_Ih*Nw + PC_It*Nr*Nw;

	A(2, 0) = A(0, 2);
	A(2, 1) = A(1, 2);
	A(2, 2) = PC_Mf*pow(PC_e, 2) + PC_If + PC_Ii + PC_Ih + PC_Is + PC_It;
	return A;
}


// PCV Dynamics page 21
Eigen::Vector3d Vehicle::calcpVector(int caster_no){
	double steer_d = _qd(2*caster_no);
	double roll_d  = _qd(2*caster_no+1);
	double gth_d   = _gxd(2);

	Eigen::Vector3d p;
	p(0) = PC_Mf*PC_r*PC_e*roll_d*(steer_d+gth_d);
	p(1) = -PC_Mf*PC_r*PC_e*pow((steer_d+gth_d), 2);
	p(2) = PC_Mf*PC_r*PC_e*roll_d*(steer_d+gth_d);
	return p; 
}


// From PCV Dynamics page 29
Eigen::Matrix3d Vehicle::calcC_thMatrix(int caster_no) 
{
	// Caster position values
	double hx = casterPosXSign(caster_no+1) * DIST_TO_CASTER_X;
	double hy = casterPosYSign(caster_no+1) * DIST_TO_CASTER_Y;

	// Take sin/cos of steering angle
	double steerSin = sin(_q_steer(caster_no));
	double steerCos = cos(_q_steer(caster_no));

	Eigen::Matrix3d C_th;
	/***  Alternate sign convention ***/
	// C_th(0, 0) = -steerSin / PC_b;
	// C_th(0, 1) = steerCos / PC_b;
	// C_th(0, 2) = (hx * steerCos + hy * steerSin) / PC_b - 1.0;
	/**********************************/
	C_th(0, 0) = steerSin / PC_b;
	C_th(0, 1) = -steerCos / PC_b;
	C_th(0, 2) = -(hx * steerCos + hy * steerSin) / PC_b - 1.0;

	C_th(1, 0) = steerCos / PC_r;
	C_th(1, 1) = steerSin / PC_r;
	C_th(1, 2) = (hx * steerSin - hy * steerCos) / PC_r;
			
	C_th(2, 0) = 0.0;
	C_th(2, 1) = 0.0;
	C_th(2, 2) = 1.0;
 
	return C_th;
}


// From PCV Dynamics page 25
Eigen::Matrix3d Vehicle::calcJ_dotMatrix(int caster_no)
{
	Eigen::Matrix3d J_dot;
	double steer_d = _qd(2*caster_no);
	double gth_d = _gxd(2);

	// Caster position values
	double hx = casterPosXSign(caster_no+1) * DIST_TO_CASTER_X;
	double hy = casterPosYSign(caster_no+1) * DIST_TO_CASTER_Y;

	// Take sin/cos of steeering angle
	double steerSin = sin(_q_steer(caster_no));
	double steerCos = cos(_q_steer(caster_no));

	// Take sin/cos of steer angle velocity plus PCV angle velocity
	double Sum = steer_d + gth_d;

	J_dot(0, 0) = PC_b * steerCos * Sum;
	J_dot(0, 1) = -PC_r * steerSin * Sum;
	J_dot(0, 2) = hx * gth_d + PC_b * steerCos * Sum;

	J_dot(1, 0) = PC_b * steerSin * Sum;
	J_dot(1, 1) = PC_r * steerCos * Sum;
	J_dot(1, 2) = hy * gth_d + PC_b * steerSin * Sum;
			
	J_dot(2, 0) = 0.0;
	J_dot(2, 1) = 0.0;
	J_dot(2, 2) = 0.0;

	return J_dot;
}


Eigen::Matrix3d Vehicle::calcLambdaPumpkinMatrix(int caster_no) 
{
	Eigen::Matrix3d lambda_pumpkin = Eigen::Matrix3d::Zero();

	// Caster position values
	double hx = casterPosXSign(caster_no+1) * DIST_TO_CASTER_X;
	double hy = casterPosYSign(caster_no+1) * DIST_TO_CASTER_Y;

	lambda_pumpkin(0, 0) = PC_Mp;
	lambda_pumpkin(0, 2) = -PC_Mp*hy;

	lambda_pumpkin(1, 1) = PC_Mp;
	lambda_pumpkin(1, 2) = PC_Mp*hx;

	lambda_pumpkin(2, 0) = lambda_pumpkin(0, 2);
	lambda_pumpkin(2, 1) = lambda_pumpkin(1, 2);
	lambda_pumpkin(2, 2) = PC_Ip + PC_Mp*pow(DIST_TO_CASTER ,2);
	return lambda_pumpkin;
}


Eigen::Matrix3d Vehicle::calcLambdaVehicleMatrix() 
{
	Eigen::Matrix3d lambda_vehicle = Eigen::Matrix3d::Zero();

	lambda_vehicle(0, 0) = PC_Mv; // Effective mass along the x direction is the mass of the vehicle 
	lambda_vehicle(1, 1) = PC_Mv; // Effective mass along the y direction is the mass of the vehicle 
	lambda_vehicle(2, 2) = PC_Iv; // Inertia of the vehicle about the z axis 

	return lambda_vehicle;
}


Eigen::Vector3d Vehicle::calcQ_dotVector(int caster_no)
{
	Eigen::Vector3d Q_dot;

	double steer_d = _qd(2*caster_no);
	double roll_d = _qd(2*caster_no+1);
	double gth_d = _gxd(2);
	Q_dot << steer_d, roll_d, gth_d;

	return Q_dot;
}


Eigen::Vector3d Vehicle::calcMuPumpkinVector(int caster_no)
{
	Eigen::Vector3d mu_pumpkin;
	double gth_d = _gxd(2);

	// Caster position values
	double hx = casterPosXSign(caster_no+1) * DIST_TO_CASTER_X;
	double hy = casterPosYSign(caster_no+1) * DIST_TO_CASTER_Y;

	mu_pumpkin << -PC_Mp * hx * pow(gth_d, 2), -PC_Mp * hy * pow(gth_d, 2), 0.0;

	return mu_pumpkin;
}


/* Update Jq matrix. Transforms q_d's into p_d's (p_d = Jq*q_d). In documentation, Jq = C_q^{-1} */
void Vehicle::updateJqMatrix() 
{
	_Jq = Eigen::MatrixXd::Zero(NUM_MOTORS, NUM_MOTORS);
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Which rows to calculate
		int steerRow = 2*i;
		int rollRow = 2*i + 1;

		// Take sin/cos of steering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Compute steer row
		_Jq(steerRow,steerRow) = steerSin * PC_b;
		_Jq(steerRow,rollRow) = steerCos * PC_r;

		// Compute roll row
		_Jq(rollRow,steerRow) = -steerCos * PC_b;
		_Jq(rollRow,rollRow) = steerSin * PC_r;
	}
}


/* Update Cp matrix. Transforms x_d's into p_d's (p_d = C_p*x_d) */
void Vehicle::updateCpMatrix() 
{
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Which rows to calculate
		int steerRow = 2*i;
		int rollRow = 2*i + 1;

		// Take sin/cos of steering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

		// Cp = Jq*C, could calculate it this way instead
		// Compute steer row
		_Cp(steerRow,0) = 1.0;
		_Cp(steerRow,1) = 0.0;
		_Cp(steerRow,2) = -PC_b * steerSin - hy;

		// Compute roll row
		_Cp(rollRow,0) = 0.0;
		_Cp(rollRow,1) = 1.0;
		_Cp(rollRow,2) = PC_b * steerCos + hx;
	}
}


// Clamp the torque command to max values
void Vehicle::clampTorque(){
	// Clamp torque values to maximum allowed
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		_tq_des(2*i)   = abs(_tq_des(2*i))   < MAX_STEER_TORQUE  ? _tq_des(2*i)   : copysign(MAX_STEER_TORQUE, _tq_des(2*i));
		_tq_des(2*i+1) = abs(_tq_des(2*i+1)) < MAX_ROLL_TORQUE   ? _tq_des(2*i+1) : copysign(MAX_ROLL_TORQUE,  _tq_des(2*i+1));
	}
}



/* ------------ Static functions-------------- */
// Determine sign of x position of caster based on number
static double casterPosXSign(int casterNum)
{
	switch(casterNum)
	{
		case 1:
		case 2:
			return 1.0;
		case 3:
		case 4:
			return -1.0;
		default:
			return 0;
	}
}


// Determine sign of y position of caster based on number
static double casterPosYSign(int casterNum)
{
	switch(casterNum)
	{
		case 2:
		case 3:
			return 1.0;
		case 1:		
		case 4:
			return -1.0;
		default:
			return 0;
	}
}


/*  Saturation function 
	Returns the input float value if its absolute value is less than or equal to 1. 
	Otherwise, it returns 1.0 or -1.0 (depending on the sign of float x) */
static float saturate(float x) {
	x = abs(x) <= 1.0 ? x : copysign(1.0, x); 
	return x; 
}


// Function to clamp velocity command to max values
static Eigen::Vector3d clampVelocity(Eigen::Vector3d xd_com_in){
	// Clamp velocity values to maximum allowed
	xd_com_in(0) = abs(xd_com_in(0)) < MAX_VEL_TRANS ? xd_com_in(0) : copysign(MAX_VEL_TRANS, xd_com_in(0));
	xd_com_in(1) = abs(xd_com_in(1)) < MAX_VEL_TRANS ? xd_com_in(1) : copysign(MAX_VEL_TRANS, xd_com_in(1));
	xd_com_in(2) = abs(xd_com_in(2)) < MAX_VEL_ROT ? xd_com_in(2) : copysign(MAX_VEL_ROT, xd_com_in(2));

	return xd_com_in;
}


// Function to clamp acceleration command to max values (accel is change in xd_des for one control loop here)
static Eigen::Vector3d clampAcceleration(Eigen::Vector3d xdd_com_in){

	// Clamp acceleration values to maximum allowed

	// Check accel/decel for x
	if(xdd_com_in(0) >= 0){
		// Clamp trans accel
		xdd_com_in(0) = xdd_com_in(0) < MAX_VEL_TRANS_INC ? xdd_com_in(0) : MAX_VEL_TRANS_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(0) = xdd_com_in(0) > -MAX_VEL_TRANS_DEC ? xdd_com_in(0) : -MAX_VEL_TRANS_DEC;
	}

	// Check accel/decel for y
	if(xdd_com_in(1) >= 0){
		// Clamp trans accel
		xdd_com_in(1) = xdd_com_in(1) < MAX_VEL_TRANS_INC ? xdd_com_in(1) : MAX_VEL_TRANS_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(1) = xdd_com_in(1) > -MAX_VEL_TRANS_DEC ? xdd_com_in(1) : -MAX_VEL_TRANS_DEC;
	}
	
	// Check accel/decel for theta
	if(xdd_com_in(2) >= 0){
		// Clamp trans accel
		xdd_com_in(2) = xdd_com_in(2) < MAX_VEL_ROT_INC ? xdd_com_in(2) : MAX_VEL_ROT_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(2) = xdd_com_in(2) > -MAX_VEL_ROT_DEC ? xdd_com_in(2) : -MAX_VEL_ROT_DEC;
	}

	return xdd_com_in;
}

/*********************** TEST HARNESS *****************************************/

#ifdef TEST_VEHICLE

// Different types of tests to run
enum test_type 
{
	TEST_INIT,
	TEST_CONSTRAINT_MATRIX,
	TEST_DYNAMICS
};

int main (void)
{
	cout << "Begin vehicle test harness" << endl;

	/* instantiate vehicle */
	Vehicle *vehicle = new Vehicle ();

	// Which test to run
	enum test_type thisTest = TEST_DYNAMICS;	
	
	int s = create_can_socket (0xF, 0xF);

	// Switch on type
	switch(thisTest){

		case TEST_DYNAMICS:
		{
			cout << "Testing Vehicle Dynamics" << endl;

		}
		
		case TEST_INIT:
			cout << "Testing Vehicle Initialization" << endl;

			if (vehicle->init())
				cout << "Vehicle initialization failed" << endl;
			else
				cout << "Vehicle initialization success" << endl;

			getchar ();
			break;

		case TEST_CONSTRAINT_MATRIX:
			{
				cout << "Testing Vehicle constraint matrix" << endl;

				Eigen::Matrix<double, NUM_MOTORS, 3> C; 		 		// constraint matrix
				Eigen::Matrix<double, NUM_CASTERS, 5> _q_steer;
				Eigen::Vector3d xd1;
				Eigen::Vector3d xd2;
				xd1 << 1,0,0;
				xd2 << 0,1,0;

				_q_steer << 0,0,0,0,
								   M_PI/2, M_PI/2, M_PI/2, M_PI/2,
									 M_PI,M_PI,M_PI,M_PI,
									 3*M_PI/2,3*M_PI/2,3*M_PI/2,3*M_PI/2,
									 -M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2;

				// Re-compute constraint matrix C
				for(int j = 0; j < 5; j++){
					cout << "q_steer = [";
					for(int i = 0; i < NUM_CASTERS; i++)
					{
						// Which rows to calculate
						int steerRow = 2*i;
						int rollRow = 2*i + 1;

						// Caster position values
						double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
						double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

						// Take sin/cos of steeering angle
						double steerSin = sin(_q_steer(i,j));
						double steerCos = cos(_q_steer(i,j));

						// Compute steer row
						C(steerRow,0) = -steerSin / PC_b;
						C(steerRow,1) =  steerCos / PC_b;
						C(steerRow,2) = (hx * steerCos + hy * steerSin) / PC_b - 1.0;

								// Compute roll row
						C(rollRow,0) = steerCos / PC_r;
						C(rollRow,1) = steerSin / PC_r;
						C(rollRow,2) = (hx * steerSin - hy * steerCos) / PC_r;

						cout << _q_steer(i,j) << ", ";
					}

					cout << "]" << endl;
					cout << endl << "C = " << endl;
					cout << C << endl;
					cout << "xd1 = " << xd1.transpose() << endl;
					cout << "qd1 = " << (C*xd1).transpose() << endl;
					cout << "xd2 = " << xd2.transpose() << endl;
					cout << "qd2 = " << (C*xd2).transpose() << endl;

					getchar();
				}
			}
			break;

		default:
			break;
	}

	// Delete caster
	delete vehicle;


	getchar ();
	cout << "End vehicle test harness" << endl;
	return 0;
}


#endif
