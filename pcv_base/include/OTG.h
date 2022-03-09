/**
 * OTG.h
 *
 *	A wrapper to use the Reflexxes library (type II or IV) with Eigen library
 *
 * Author: Mikael Jorda
 * Created: October 2018
 */

#ifndef OTG_H
#define OTG_H

#include <Eigen/Dense>
#include <ReflexxesTypeII/include/ReflexxesAPI.h>

#include <iostream>


class OTG {
public:


	/**
	 * @brief      constructor
	 *
	 * @param[in]  task_dof   dimmension of the space in which the trajectory is
	 *                        computed (e.g. 3 for a position trajectory in
	 *                        Cartesian space)
	 * @param[in]  loop_time  The duration of a control loop (typically, 0.001
	 *                        if the robot is controlled at 1 kHz)
	 */
	OTG(const Eigen::VectorXd& initial_position, const double loop_time);

	/**
	 * @brief      destructor
	 */
	~OTG();

	void reInitialize(const Eigen::VectorXd& initial_position);

	/**
	 * @brief      Sets the maximum velocity for the trajectory generator
	 *
	 * @param[in]  max_velocity  Vector of the maximum velocity per direction
	 */
	void setMaxVelocity(const Eigen::VectorXd max_velocity);

	/**
	 * @brief      Sets the maximum velocity.
	 *
	 * @param[in]  max_velocity  Scalar of the maximum velocity in all directions
	 */
	void setMaxVelocity(const double max_velocity);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Vector of the maximum acceleration
	 */
	void setMaxAcceleration(const Eigen::VectorXd max_acceleration);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Scalar of the maximum acceleration
	 */
	void setMaxAcceleration(const double max_acceleration);

	/**
	 * @brief      Sets the maximum jerk.
	 *
	 * @param[in]  max_jerk  Vector of the maximum jerk
	 */
	void setMaxJerk(const Eigen::VectorXd max_jerk);

	/**
	 * @brief      Sets the maximum jerk.
	 *
	 * @param[in]  max_jerk  Scalar of the maximum jerk
	 */
	void setMaxJerk(const double max_jerk);

	/**
	 * @brief      Sets the goal position and velocity
	 *
	 * @param[in]  goal_position  The goal position
	 * @param[in]  goal_velocity  The goal velocity
	 */
	void setGoalPositionAndVelocity(const Eigen::VectorXd goal_position, const Eigen::VectorXd goal_velocity);

	/**
	 * @brief      Calculates the next desired position and velocity for the next step
	 *
	 * @param      next_position  The desired position in the next step
	 * @param      next_velocity  The desired velocity in the next step
	 */
	void computeNextState(Eigen::VectorXd& next_position, Eigen::VectorXd& next_velocity);
	/**
	 * @brief      Function to know if the goal position and velocity is reached
	 *
	 * @return     true if the goal state is reached, false otherwise
	 */
	bool goalReached();

	unsigned int _task_dof;
	double _loop_time;
	bool _goal_reached = false;

	// Reflexxes variables
	int _ResultValue = 0;
    ReflexxesAPI *_RML = NULL;
    RMLPositionInputParameters *_IP = NULL;
    RMLPositionOutputParameters *_OP = NULL;
    RMLPositionFlags _Flags;

};

#endif //OTG_H