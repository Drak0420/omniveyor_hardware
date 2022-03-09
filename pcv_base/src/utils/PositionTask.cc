#include "../include/PositionTask.h"
#include <omniveyor_common/definitions.h>

PositionTask::PositionTask()
{

	_current_position.setZero(); 
	_current_velocity.setZero(); 

	// Default values for gains and velocity saturation
	_use_velocity_saturation_flag = false;
	_kp = 50.0; 
	_kv = 14.0;

#ifdef USING_OTG
	_use_interpolation_flag = true; 
	OTG* otg = new OTG(gx, CONTROL_PERIOD_s);
	Eigen::Vector3d max_vel; 
	max_vel << 0.2, 0.2, 0.8; 
	otg->setMaxVelocity(max_vel);
	otg->setMaxAcceleration(10.0);
	otg->setMaxJerk(6.0);
	otg->reInitialize(gx);
#endif 

}