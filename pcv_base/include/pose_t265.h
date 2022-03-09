#ifndef POSE_T265_H
#define POSE_T265_H

#include <librealsense2/rs.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class CameraT265
{
public: 
	/* Constructor for CameraT265 class */
	CameraT265(); 

	/* Initialize the Intel T265 Camera */
	int init(); 

	/* Update the linear and angular position measurements of the camera */
	int updatePose(); 

	/* Get the camera's estimate of the position of the mobile base operational point */
	Eigen::Vector3d getCameraPosition() const;

	/* Get the camera's estimate of the velocity of the mobile base operational point */
	Eigen::Vector3d getCameraVelocity() const;

	/* Get the camera's estimate of the acceleration of the mobile base operational point */
	Eigen::Vector3d getCameraAcceleration() const;

	/* Get the camera's estimate of the angular position of the mobile base */
	Eigen::Vector4d getCameraAngularPosition() const;

	/* Get the camera's estimate of the angular velocity of the mobile base */
	Eigen::Vector3d getCameraAngularVelocity() const;

	/* Get the camera's estimate of the angular acceleration of the mobile base */
	Eigen::Vector3d getCameraAngularAcceleration() const;

	/* Get the camera's estimate of theta */
	float getThCam() const;

private: 
	/* Global reference frame, N */
	Eigen::Vector3d _gx_cam;    // Camera estimate of mobile base's position
	Eigen::Vector3d _gxd_cam;   // Camera estimate of mobile base's velocity
	Eigen::Vector3d _gxdd_cam;  // Camera estimate of mobile base's acceleration
	Eigen::Vector4d _gq_cam;    // Camera estimate of mobile base's angular position
	Eigen::Vector3d _gqd_cam;   // Camera estimate of mobile base's angular velocity
	Eigen::Vector3d _gqdd_cam;  // Camera estimate of mobile base's angular acceleration

	/* Global reference frame, N with respect to initial camera reference
	   frame origin, C0 */
	Eigen::Vector3d _Cx_cam;    // Position
	Eigen::Vector3d _Cxd_cam;   // Velocity
	Eigen::Vector3d _Cxdd_cam;  // Acceleration 

	/* Initial camera reference frame, C */
	Eigen::Quaterniond _Cq_cam;     // Angular position
	Eigen::Vector3d    _Cqd_cam;    // Angular velocity
	Eigen::Vector3d    _Cqdd_cam;   // Angular acceleration

	/* Camera offset from the center of the mobile base */
	float _mx;                // offset along x-direction of frame M
	float _my;                // offset along y-direction of frame M
	float _mz;                // offset along z-direction of frame M
	Eigen::Vector3d _Mx_cam;

	float _th_cam;            // Camera's estimate of theta  
	float _prev_th_cam; 	  // Camera's estimate of theta at previous timestep
	float _th_offset; 		  // Theta offset to deal with wrap around past PI
	
    rs2::pipeline pipe; // RealSense pipeline
    rs2::config cfg;    // RealSense pipeline configuration
};

#endif