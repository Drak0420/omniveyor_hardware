/**
 * OTG.cc
 *
 *	A wrapper to use the Reflexxes library (type II or IV) with Eigen library
 *
 * Author: Mikael Jorda
 * Created: October 2018
 */

#include "../include/OTG.h"
#include <stdexcept>


OTG::OTG(const Eigen::VectorXd& initial_position, const double loop_time)
{
	_task_dof = initial_position.size();
	_loop_time = loop_time;

    _IP  = new RMLPositionInputParameters(_task_dof);
    _OP  = new RMLPositionOutputParameters(_task_dof);
	_RML = new ReflexxesAPI(_task_dof, _loop_time);

    setGoalPositionAndVelocity(initial_position, Eigen::VectorXd::Zero(_task_dof));

    for(int i=0 ; i<_task_dof ; i++)
    {
    	_IP->SelectionVector->VecData[i] = true;
		_OP->NewPositionVector->VecData[i] = initial_position(i);
    }
}

OTG::~OTG()
{
	delete _RML;
	delete _IP;
	delete _OP;
	_RML = NULL;
	_IP = NULL;
	_OP = NULL;
}

void OTG::reInitialize(const Eigen::VectorXd& initial_position)
{
    setGoalPositionAndVelocity(initial_position, Eigen::VectorXd::Zero(_task_dof));
    
    for(int i=0 ; i<_task_dof ; i++)
    {
		_OP->NewPositionVector->VecData[i] = initial_position(i);
		_OP->NewVelocityVector->VecData[i] = 0;
		_OP->NewAccelerationVector->VecData[i] = 0;
    }	
}

void OTG::setMaxVelocity(const Eigen::VectorXd max_velocity)
{
	if(max_velocity.size() != _task_dof)
	{
		throw std::invalid_argument("size of input max velocity vector does not match task size in OTG::setMaxVelocity\n");
	}
	if(max_velocity.minCoeff() <= 0)
	{
		throw std::invalid_argument("max velocity set to 0 or negative value in some directions in OTG::setMaxVelocity\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
	    _IP->MaxVelocityVector->VecData[i] = max_velocity(i);
	}
}

void OTG::setMaxVelocity(const double max_velocity)
{
	setMaxVelocity(max_velocity * Eigen::VectorXd::Ones(_task_dof));
}

void OTG::setMaxAcceleration(const Eigen::VectorXd max_acceleration)
{
	if(max_acceleration.size() != _task_dof)
	{
		throw std::invalid_argument("size of input max acceleration vector does not match task size in OTG::setMaxAcceleration\n");
	}
	if(max_acceleration.minCoeff() <= 0)
	{
		throw std::invalid_argument("max acceleration set to 0 or negative value in some directions in OTG::setMaxAcceleration\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
	    _IP->MaxAccelerationVector->VecData[i] = max_acceleration(i);
	}
}

void OTG::setMaxAcceleration(const double max_acceleration)
{
	setMaxAcceleration(max_acceleration * Eigen::VectorXd::Ones(_task_dof));
}

void OTG::setMaxJerk(const Eigen::VectorXd max_jerk)
{
	if(max_jerk.size() != _task_dof)
	{
		throw std::invalid_argument("size of input max jerk vector does not match task size in OTG::setMaxJerk\n");
	}
	if(max_jerk.minCoeff() <= 0)
	{
		throw std::invalid_argument("max jerk set to 0 or negative value in some directions in OTG::setMaxJerk\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
	    _IP->MaxJerkVector->VecData[i] = max_jerk(i);
	}
}

void OTG::setMaxJerk(const double max_jerk)
{
	setMaxJerk(max_jerk * Eigen::VectorXd::Ones(_task_dof));
}

void OTG::setGoalPositionAndVelocity(const Eigen::VectorXd goal_position, const Eigen::VectorXd goal_velocity)
{
	if(goal_position.size() != _task_dof)
	{
		throw std::invalid_argument("size of input goal position does not match task size in OTG::setGoalPositionAndVelocity\n");
	}
	if(goal_velocity.size() != _task_dof)
	{
		throw std::invalid_argument("size of input goal velocity does not match task size in OTG::setGoalPositionAndVelocity\n");
	}
	for(int i=0 ; i<_task_dof ; i++)
	{
		if( (_IP->TargetPositionVector->VecData[i] != goal_position(i)) || (_IP->TargetVelocityVector->VecData[i] != goal_velocity(i)) )
		{
			_IP->TargetPositionVector->VecData[i] = goal_position(i);
			_IP->TargetVelocityVector->VecData[i] = goal_velocity(i);
			_goal_reached = false;
		}
	}
}


void OTG::computeNextState(Eigen::VectorXd& next_position, Eigen::VectorXd& next_velocity)
{
	next_position.setZero(_task_dof);
	for(int i=0 ; i<_task_dof ; i++)
	{
		_IP->CurrentPositionVector->VecData[i] = _OP->NewPositionVector->VecData[i];
	    _IP->CurrentVelocityVector->VecData[i] = _OP->NewVelocityVector->VecData[i];
	    _IP->CurrentAccelerationVector->VecData[i] = _OP->NewAccelerationVector->VecData[i];
		next_position(i) = _IP->TargetPositionVector->VecData[i];
		next_velocity(i) = 0;
	}

	if(!_goal_reached)
	{
		_ResultValue = _RML->RMLPosition(*_IP, _OP, _Flags);
        if (_ResultValue < 0)
        {
        	if(_ResultValue == -102)
        	{
        		printf("trajectory generation : phase synchronisation not possible.\n");
        	}
        	else
        	{
	        	printf("An error occurred (%d) in OTG::computeNextState.\n", _ResultValue );
	            throw std::runtime_error("error in computing next state in OTG::computeNextState.\n");
        	}
        }

		for(int i=0 ; i<_task_dof ; i++)
		{
			next_position(i) = _OP->NewPositionVector->VecData[i];
			next_velocity(i) = _OP->NewVelocityVector->VecData[i];
		}
	}

	_goal_reached = (_ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
	
}


bool OTG::goalReached()
{
	return _goal_reached;
}
