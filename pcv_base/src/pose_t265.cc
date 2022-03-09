// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <iostream>
#include <iomanip>
#include <cstring>
#include <math.h>
#include "../include/pose_t265.h"
#include <Eigen/EulerAngles>

using std::cout; 
using std::endl;

/* Constructor for CameraT265 class */
CameraT265::CameraT265()
{
  _gx_cam    = Eigen::Vector3d::Zero();     
  _gxd_cam   = Eigen::Vector3d::Zero();     
  _gxdd_cam  = Eigen::Vector3d::Zero();     
  _gq_cam    = Eigen::Vector4d::Zero();     
  _gqd_cam   = Eigen::Vector3d::Zero();     
  _gqdd_cam  = Eigen::Vector3d::Zero();     

  _Cx_cam    = Eigen::Vector3d::Zero();    
  _Cxd_cam   = Eigen::Vector3d::Zero();     
  _Cxdd_cam  = Eigen::Vector3d::Zero();     
  _Cq_cam    = Eigen::Quaterniond::Identity();  
  _Cqd_cam   = Eigen::Vector3d::Zero();     
  _Cqdd_cam  = Eigen::Vector3d::Zero();     

  _th_cam = 0.0; 
  _prev_th_cam = 0.0; 
  _th_offset = 0.0; 

  _mx = -0.210352;                         
  _my = 0.0;                               
  _mz = 0.0;                             
  _Mx_cam << _mx, _my, _mz; 
}


/* Initialize the Intel T265 Camera */
int CameraT265::init()
{
  try
  {
      // Add pose stream
      cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
      // Start pipeline with chosen configuration
      pipe.start(cfg);

      void rs2_set_option(const rs2_sensor* sensor, rs2_option option, float value, rs2_error** error);
      return 1;
  }
  catch (const rs2::error & e)
  {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
      return 0;
  }
  catch (const std::exception& e)
  {
      std::cerr << e.what() << std::endl;
      return 0;
  }
}


/* Update the linear and angular position measurements of the camera */
int CameraT265::updatePose()
{
  try 
  {
      /*************************************** READ CAMERA MEASUREMENTS *******************************************/ 
      // Wait for the next set of frames from the camera
      auto frames = pipe.wait_for_frames();
      // Get a frame from the pose stream
      auto f = frames.first_or_default(RS2_STREAM_POSE);
      // // Cast the frame to pose_frame and get its data
      auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

      // Camera position measurements in global reference frame, N, with respect to initial camera frame origin, C0
      _Cx_cam << pose_data.translation.z, pose_data.translation.x, pose_data.translation.y; 

      _Cxd_cam << pose_data.velocity.z, pose_data.velocity.x, pose_data.velocity.y; 

      _Cxdd_cam << pose_data.acceleration.z, pose_data.acceleration.x, pose_data.acceleration.y; 

      // Camera orientation measurements in initial camera frame, C
      _Cq_cam = Eigen::Quaterniond(pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y,
                pose_data.rotation.z);   

      _Cqd_cam << pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z; 

      _Cqdd_cam << pose_data.angular_acceleration.x, pose_data.angular_acceleration.y, pose_data.angular_acceleration.z; 

      /************************* CONVERT CAMERA MEASUREMENTS TO GLOBAL FRAME ******************************/ 
      // Rotation from the initial camera frame, C, to the world frame
      Eigen::Matrix3d R_C2World;  
      R_C2World << 0, 0, 1, 1, 0, 0, 0, 1, 0;  
      Eigen::Quaterniond _quat_C2World = Eigen::Quaterniond(R_C2World) * _Cq_cam; 

      _gqd_cam  = R_C2World * _Cqd_cam;  // TODO: CHECK IF THIS IS CORRECT
      _gqdd_cam = R_C2World * _Cqdd_cam; // TODO: CHECK IF THIS IS CORRECT

      /*************************************** ESTIMATE THETA *******************************************/ 
      // z-axis of the camera expressed in the world frame
      Eigen::Vector3d cam_z_axis = _quat_C2World * Eigen::Vector3d::UnitZ(); 

      double x1 = 1.0; 
      double y1 = 0.0; 
      double x2 = cam_z_axis(0); 
      double y2 = cam_z_axis(1); 

      double dot = x1*x2 + y1*y2;              // dot product
      double det = x1*y2 - y1*x2;              // determinant
      _th_cam = atan2(det, dot) + _th_offset;  // atan2(y, x) or atan2(sin, cos)

      // Calculate appropriate theta offset to deal with wrap around at PI and -PI 
      // -3.0 value was chosen as a large enough jump to detec wrap around 
      if ((_th_cam - _prev_th_cam) < -3.0) {
        _th_offset += 2.0 * M_PI; 
        _th_cam += 2.0 * M_PI; 
      } else if ((_th_cam - _prev_th_cam) > 3.0) {
        _th_offset += -2.0 * M_PI; 
        _th_cam += -2.0 * M_PI; 
      }
      _prev_th_cam = _th_cam; 

      // Rotation from the current camera frame to the initial camera frame, C
      _Cq_cam.normalize(); 
      Eigen::Matrix3d R_cam2C = (_Cq_cam).toRotationMatrix(); 

      // Rotation from the mobile base frame, M, to the world frame, N
      Eigen::Matrix3d R_World2M; 
      R_World2M << cos(_th_cam), sin(_th_cam), 0, -sin(_th_cam), cos(_th_cam), 0, 0, 0, 1; 

      /*************************************** ESTIMATE POSITION *******************************************/ 
      // Position of the camera with respect to C0 in world frame, N
      Eigen::Vector3d p_cam_C0_N = _Cx_cam;

      // Position of the camera frame origin, C0, with respect to the world frame origin, N0, in the world frame, N
      Eigen::Vector3d p_C0_N0_N = Eigen::Vector3d(_mx, _my, _mz); 

      // Position of the camera with respect to the world frame origin, N0, in the world frame, N
      Eigen::Vector3d p_cam_N0_N = p_cam_C0_N + p_C0_N0_N; 
      
      // Position of the camera with respect to the mobile base origin in the mobile base frame, M
      Eigen::Vector3d p_cam_M0_M = Eigen::Vector3d(_mx, _my, _mz); 

      // Position of the camera with respect to the mobile base origin in the world frame, N
      Eigen::Vector3d p_cam_M0_N = R_World2M.transpose() * p_cam_M0_M; 

      // Position of the mobile base origin with respect to the world frame origin, N0, in the world frame
      Eigen::Vector3d p_M0_N0_N = p_cam_N0_N - p_cam_M0_N; 

      _gx_cam =  p_M0_N0_N; 

      /*************************************** ESTIMATE VELOCITY *******************************************/ 
      // Velocity of the camera in world frame, N
      _gxd_cam = _Cxd_cam; 

      /*************************************** ESTIMATE ACCELERATION *******************************************/ 
      // Acceleration of the camera in world frame, N
      _gxdd_cam = _Cxdd_cam; 

      return 1;
  }
  catch (const rs2::error & e)
  {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
      return 0;
  }
  catch (const std::exception& e)
  {
      std::cerr << e.what() << std::endl;
      return 0;
  }
}


/* Get the camera's estimate of the position of the mobile base operational point */
Eigen::Vector3d CameraT265::getCameraPosition() const 
{
  return _gx_cam; 
}


/* Get the camera's estimate of the velocity of the mobile base operational point */
Eigen::Vector3d CameraT265::getCameraVelocity() const
{
  return _gxd_cam; 
}


/* Get the camera's estimate of the acceleration of the mobile base operational point */
Eigen::Vector3d CameraT265::getCameraAcceleration() const
{
  return _gxdd_cam; 
}


/* Get the camera's estimate of the angular position of the mobile base */
Eigen::Vector4d CameraT265::getCameraAngularPosition() const
{
  return _gq_cam; 
}


/* Get the camera's estimate of the angular velocity of the mobile base */
Eigen::Vector3d CameraT265::getCameraAngularVelocity() const
{
  return _gqd_cam; 
}


/* Get the camera's estimate of the angular acceleration of the mobile base */
Eigen::Vector3d CameraT265::getCameraAngularAcceleration() const
{
  return _gqdd_cam; 
}


/* Get the camera's estimate of theta */
float CameraT265::getThCam() const
{
  return _th_cam; 
}


