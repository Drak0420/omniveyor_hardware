#ifndef _PCV_BASE_NODE_H_
#define _PCV_BASE_NODE_H_

/*------------------------------includes--------------------------------------*/
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <signal.h>
#include <string>
#include <unistd.h>
#include <mqueue.h>
#include <map>
#include <ctime>
#include <chrono>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Byte.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "omniveyor_common/electricalStatus.h"

// Alternative software solver for torque control
// #define USING_OTG
#include "hardware.h"
#include "motor.h"
#include "CO_message.h"
#include "CO_objects.h"
#include "CAN_utils.h"
#include "RT_utils.h"
#include "vehicle.h"
#include "event.h"
#include <Eigen/Core>
#include <omniveyor_common/definitions.h>
#ifdef USING_OTG
#include "../include/OTG.h"
#endif
#ifdef CAMERA
#include "../include/pose_t265.h"
#endif
#ifdef JOYSTICK
#include "../include/buttons.h"
#endif

using std::cout;
using std::endl;
typedef void * (*threadFcnPtr)(void *);

class PCVBaseNode{

    public:
    PCVBaseNode(int argc, char *argv[], ros::NodeHandle *rosNH);
    ~PCVBaseNode();

    /*------------------------------defines---------------------------------------*/

    /*------------------------------structs---------------------------------------*/

    /*------------------------static function declarations------------------------*/
    static void perform_startup_tasks (void);
    static void parse_command_args (int argc, char *argv[]);
    static void *control_thread (void *aux);
    //static void sig_handler_int (int);
    static void sig_handler_tstp (int);
    static void sig_handler_cont (int);
    static inline void sleep_until (struct timespec *ts, long delay);
    static int kbhit(void);
    /*------------------------static variable declarations------------------------*/
    
    ros::NodeHandle _rosNH;
    ros::Publisher odomPub;
    ros::Publisher eStatusPub;
    ros::Subscriber cmdSubV;
    ros::Subscriber cmdSubX;
    ros::Subscriber cmdSubA;
    ros::Subscriber ctrlModeSub;
    ros::Subscriber lidarSub;
    float tfCos, tfSin, tfX, tfY;
    float *rayCos, *raySin, *rayX, *rayY;
    double pubRate;
    //bool onDestruction;

    tf::TransformBroadcaster odomBroadcaster;
	pthread_t control;
	
    inline static Vehicle *vehicle = new Vehicle();

    /* state space variables for velocity and torque control mode */
    inline static double gx_des_g[3] = {0.};      // Position desired, global var
    inline static double gxd_des_g_raw[3] = {0.}; // Velocity desired, global var (x_dot, y_dot, theta_dot)
    inline static double gxd_des_g[3] = {0.};     // Velocity desired, global var (x_dot, y_dot, theta_dot)
    inline static double gxdd_des_g[3] = {0.};    // Acceleration desired, global var

    /* for recording data */
    inline static bool dumpData = false;
    inline static std::ofstream file;

    inline static enum ctrl_mode control_mode = VELOCITY;
    // inline enum ctrl_mode control_mode = TORQUE;

#ifdef JOYSTICK
    inline static mqd_t mq_joystick;
    inline static const char *mq_name = "/joystick message queue";
#endif
#ifdef CAMERA
    inline static CameraT265 *cameraT265 = new CameraT265();
#endif

    void cmdPosRecvCallback(const geometry_msgs::Pose::ConstPtr& x_des);
    void cmdVelRecvCallback(const geometry_msgs::Twist::ConstPtr& xd_des);
    void cmdVelRawCallback(const geometry_msgs::Twist::ConstPtr& xd_des);
    void cmdAccRecvCallback(const geometry_msgs::Accel::ConstPtr& xdd_des);
    void ctrlModeRecvCallback(const std_msgs::Byte::ConstPtr& ctrlMode);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    void spin();

};

#endif
