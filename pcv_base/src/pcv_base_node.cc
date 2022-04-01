#include "pcv_base_node.h"

/* these parameters were for testing sine wave inputs, uncomment next line
   to run the platform in a circular motion and ignore controller inputs */

#define X_freq                  (0.1)       /* [hz] */
#define Y_freq                  (0.1)       /* [hz] */
#define THETA_freq              (0.05)      /* [hz] */

// Trajectory modes
// #define SPIN
// #define LINE
// #define SQUARE

/*---------------------------public functions---------------------------------*/
/*
 * The main function for the vehicle program. You can pass a --dump flag to
 * output data to a csv for debugging. This file i/o is performed in the
 * control_thread function if it needs to be changed.
 */

void PCVBaseNode::cmdPosRecvCallback(const geometry_msgs::Pose::ConstPtr& x_des)
{
    gx_des_g[0] = x_des->position.x;
    gx_des_g[1] = x_des->position.y;
    gx_des_g[2] = tf::getYaw(x_des->orientation);
}

void PCVBaseNode::cmdVelRecvCallback(const geometry_msgs::Twist::ConstPtr& xd_des)
{
    gxd_des_g[0] = xd_des->linear.x;
    gxd_des_g[1] = xd_des->linear.y;
    gxd_des_g[2] = xd_des->angular.z;
}

void PCVBaseNode::cmdVelRawCallback(const geometry_msgs::Twist::ConstPtr& xd_des)
{
    gxd_des_g_raw[0] = xd_des->linear.x;
    gxd_des_g_raw[1] = xd_des->linear.y;
    gxd_des_g_raw[2] = xd_des->angular.z;
}

void PCVBaseNode::cmdAccRecvCallback(const geometry_msgs::Accel::ConstPtr& xdd_des)
{
    gxdd_des_g[0] = xdd_des->linear.x;
    gxdd_des_g[1] = xdd_des->linear.y;
    gxdd_des_g[2] = xdd_des->angular.z;
}

void PCVBaseNode::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    uint32_t nSamples = scan->ranges.size();
    float minDist = 1.e6;
    uint32_t ind;
    for(uint32_t i=0; i<nSamples; i++){
        float xlaser = scan->ranges[i]*rayCos[i];
        float ylaser = scan->ranges[i]*raySin[i];
        rayX[i] = tfCos*xlaser - tfSin*ylaser + tfX;
        rayY[i] = tfSin*xlaser + tfCos*ylaser + tfY;
        float iDist = fmax(fabs(rayX[i]), fabs(rayY[i]));
        if (iDist < minDist){
            minDist = iDist;
            ind = i;
        }
    }
    //std::cout << minDist << "  " << ind << std::endl;
    const float hardBound = PC_length*0.5;
    const float softBound = hardBound+0.05;
    if (minDist >= softBound){
        gxd_des_g[0] = gxd_des_g_raw[0];
        gxd_des_g[1] = gxd_des_g_raw[1];
        gxd_des_g[2] = gxd_des_g_raw[2];
        return;
    }
    if (fabs(rayX[ind])>=fabs(rayY[ind])){    // front obstacle, limit frontal velocity
        gxd_des_g[0] = (rayX[ind]>0.)? fmin(0., gxd_des_g_raw[0]) : fmax(0., gxd_des_g_raw[0]);
        gxd_des_g[1] = gxd_des_g_raw[1];
        if (fabs(rayY[ind])<hardBound){
            // left side obstacle. Prevent steer right.
            // right side obstacle. Prevent steer left.
            gxd_des_g[2] = ((rayY[ind]>0.)==(rayX[ind]>0.))?
                            fmax(0., gxd_des_g_raw[2]) : fmin(0., gxd_des_g_raw[2]);
            //std::cout << "FrontSideObstacle" << std::endl;
        } else {
            gxd_des_g[2] = gxd_des_g_raw[2];
            //std::cout << "FrontObstacle" << std::endl;
        }
    } else {                // side obstacle. limit side velocity
        //std::cout << "SideObstacle" << std::endl;
        gxd_des_g[0] = gxd_des_g_raw[0];
        gxd_des_g[1] = (rayY[ind]>0.)? fmin(0., gxd_des_g_raw[1]) : fmax(0., gxd_des_g_raw[1]);
        if (fabs(rayX[ind])<hardBound){
            // side obstacles.
            gxd_des_g[2] = ((rayX[ind]>0.)==(rayY[ind]>0.))?
                            fmin(0., gxd_des_g_raw[2]) : fmax(0., gxd_des_g_raw[2]);
        } else {
            gxd_des_g[2] = gxd_des_g_raw[2];
        }
    }
}

void PCVBaseNode::ctrlModeRecvCallback(const std_msgs::Byte::ConstPtr& ctrlMode)
{
    switch (ctrlMode->data){
        case 0:
               if (!vehicle->isStopped())
                vehicle->stop();
            break;
        case 1:
            if (vehicle->getCtrlMode()!=VELOCITY){      // CtrlMode == Torque
                if (!vehicle->isStopped())              // Vehicle Enabled
                    vehicle->stop();                 	// First disable vehicle
                vehicle->setCtrlMode(VELOCITY);         // Set CtrlMode to Velocity
                vehicle->start();                    	// Reenable vehicle
            }
            else if (vehicle->isStopped())              // CtrlMode == Velocity and vehicle disabled
                vehicle->start();                       // Enables vehicle
            break;
        case 2:
            if (vehicle->getCtrlMode()!=TORQUE){        // CtrlMode == VELOCITY
                if (!vehicle->isStopped())              // Vehicle Enabled
                    vehicle->stop();                    // First disable vehicle
                vehicle->setCtrlMode(TORQUE);           // Set CtrlMode to TORQUE
                vehicle->start();                       // Reenable vehicle
            }
            else if (vehicle->isStopped())              // CtrlMode == Torque and vehicle disabled
                vehicle->start();                       // Enables vehicle
            break;
        default:
            ROS_ERROR("Unknown Control Mode!");
            break;
    }
}


/* 
    Initialize ROS pub/sub:
    Publishes wheel odometry
    Publishes motor electrical status to topic = electricalStatus
    Publish rate = 50Hz.
*/
PCVBaseNode::PCVBaseNode(int argc, char *argv[], ros::NodeHandle *rosNH) : 
    _rosNH(*rosNH)
{
    std::string         laserScanTopic;
    std::string         odomTopic;
    std::string         electricalStatusTopic;
    std::string         positionCommandTopic;
    std::string         velocityCommandTopic;
    std::string         accelerationCommandTopic;
    std::string         controlModeTopic;
    bool                lidarSafeguard;

    _rosNH.param<std::string>("laser_scan_topic",laserScanTopic,"scan");
    _rosNH.param<std::string>("odom_topic",odomTopic,"odom");
    _rosNH.param<std::string>("electrical_status_topic",electricalStatusTopic,"electricalStatus");
    _rosNH.param<std::string>("position_command_topic",positionCommandTopic,"cmd_pos");
    _rosNH.param<std::string>("velocity_command_topic",velocityCommandTopic,"cmd_vel");
    _rosNH.param<std::string>("acceleration_command_topic",accelerationCommandTopic,"cmd_acc");
    _rosNH.param<std::string>("control_mode_topic",controlModeTopic,"control_mode");
    _rosNH.param<bool>("lidar_safeguard",lidarSafeguard,false);
    _rosNH.param<double>("update_rate",this->pubRate,50.0);

    this->odomPub = _rosNH.advertise<nav_msgs::Odometry>(odomTopic, 10);
    this->eStatusPub = _rosNH.advertise<omniveyor_common::electricalStatus>(electricalStatusTopic,1);
    //ros::Publisher statusPub = rosNH.advertise<std_msgs::Float64MultiArray>("dump",10);
    if (!lidarSafeguard){
        this->cmdSubV = _rosNH.subscribe(velocityCommandTopic, 10, &PCVBaseNode::cmdVelRecvCallback, this);
    } else {
        //throw std::runtime_error("Method Not Implemented yet!");
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        sensor_msgs::LaserScan::ConstPtr oneScan = 
                ros::topic::waitForMessage<sensor_msgs::LaserScan>(laserScanTopic,_rosNH);
        uint32_t nSamples = oneScan->ranges.size();
        if ((rayCos=(float *)malloc(nSamples*sizeof(float)))==NULL){
            throw std::runtime_error("Out Of Memory");
        }
        if ((raySin=(float *)malloc(nSamples*sizeof(float)))==NULL){
            throw std::runtime_error("Out Of Memory");
        }
        if ((rayX=(float *)malloc(nSamples*sizeof(float)))==NULL){
            throw std::runtime_error("Out Of Memory");
        }
        if ((rayY=(float *)malloc(nSamples*sizeof(float)))==NULL){
            throw std::runtime_error("Out Of Memory");
        }
        float theta = oneScan->angle_min;
        for(uint32_t i=0; i<nSamples; i++){
            rayCos[i] = cosf(theta);
            raySin[i] = sinf(theta);
            theta += oneScan->angle_increment;
        }
        ros::Time now = ros::Time::now();
        while (!tfBuffer.canTransform(oneScan->header.frame_id, "base_link",
                                ros::Time::now(), ros::Duration(10.0)));
        geometry_msgs::TransformStamped baseToLaser = tfBuffer.lookupTransform(
                            oneScan->header.frame_id, "base_link", ros::Time::now());
        Eigen::Quaterniond q = Eigen::Quaterniond(baseToLaser.transform.rotation.w, 
                                                baseToLaser.transform.rotation.x, 
                                                baseToLaser.transform.rotation.y,
                                                baseToLaser.transform.rotation.z);
        Eigen::Matrix3d qm = q.toRotationMatrix();
        tfCos = (float)qm(0,0);
        tfSin = (float)qm(1,0);
        tfX = (float)baseToLaser.transform.translation.x;
        tfY = (float)baseToLaser.transform.translation.y;
        this->cmdSubV = _rosNH.subscribe(velocityCommandTopic, 10, &PCVBaseNode::cmdVelRawCallback, this);
        this->lidarSub = _rosNH.subscribe(laserScanTopic, 10, &PCVBaseNode::lidarCallback, this);
    }
    this->cmdSubX = _rosNH.subscribe(positionCommandTopic, 10, &PCVBaseNode::cmdPosRecvCallback, this);
    this->cmdSubA = _rosNH.subscribe(accelerationCommandTopic, 10, &PCVBaseNode::cmdAccRecvCallback, this);
    this->ctrlModeSub = _rosNH.subscribe(controlModeTopic, 10, &PCVBaseNode::ctrlModeRecvCallback, this);

    cout << "vehicle startup" << endl;
    
    /* signal rerouting */
    perform_startup_tasks ();

    /* now parse command line args */
    parse_command_args (argc, argv);

    /* Initialize csv file to write to */
    if (dumpData)
    {
        // if (control_mode == TORQUE)
        // {
            file.open("../traces/tuning.csv", std::ios::out);
            file << "t, dt,";
            file << "x, y, theta,";
            file << "x_des, y_des, theta_des,";
            file << "xd, yd, thetad,";
            file << "tq1_des, tq2_des, tq3_des, tq4_des, tq5_des, tq6_des, tq7_des, tq8_des,";
            file << "fx_des, fy_des, th_des,";
            file << "l1, l2, l3, l4, l5, l6, l7, l8, l9,";
            file << "qsteer1, qsteer2, qsteer3, qsteer4,";
            file << "step_x_des, step_y_des, step_theta_des,";
            file << "step_xd_des, step_yd_des, step_thetad_des,";
            file << "x_cam, y_cam, z_cam,";
            file << "th_cam,";
            file << "heading,";
            file << "x_local, y_local, th_local,";
            file << "xd_local, yd_local, thd_local,";
            file << "qd1, qd2, qd3, qd4,";
            file << "Cpinv1, Cpinv2, Cpinv3, Cpinv4, Cpinv5, Cpinv6, Cpinv7, Cpinv8, Cpinv9, Cpinv10,";
            file << "Cpinv11, Cpinv12, Cpinv13, Cpinv14, Cpinv15, Cpinv16, Cpinv17, Cpinv18, Cpinv19,";
            file << "Cpinv20, Cpinv21, Cpinv22, Cpinv23, Cpinv24";
            file << endl;
        // }
    }
    
    /* Initialize vehicle and if successful, launch control thread*/
    if (vehicle->init()){
        throw std::runtime_error("Vehicle Initialization failed!");
    } else {
        cout << "Vehicle Initialization succeeded!" << endl;
        vehicle->setCtrlMode(control_mode);
        
        // Goal position controls
        // double Kp_x  = 60.0;
        // double Kp_y  = 60.0;
        // double Kp_th = 40.0;

        // double Kv_x  = 12.0;
        // double Kv_y  = 12.0;
        // double Kv_th = 12.0;

        // Trajectory tracking controls
        double Kp_x  = 50.0;
        double Kp_y  = 50.0;
        double Kp_th = 50.0;

        double Kv_x  = 14.0;
        double Kv_y  = 14.0;
        double Kv_th = 20.0;

        Eigen::Array3d Kp;
        Eigen::Array3d Kv;

        Kp << Kp_x, Kp_y, Kp_th;
        Kv << Kv_x, Kv_y, Kv_th;

        vehicle->setKp(Kp);
        vehicle->setKv(Kv);
        vehicle->enable();

        cout << "Control gains: " << endl;
        cout << "Kp: " << Kp << endl;
        cout << "Kv: " << Kv << endl;

#ifdef MANUAL_ZERO
        cout << "Press ENTER once the vehicle is at the desired origin." << endl;
        int c;
        c = getchar();

    #ifdef CAMERA
        /* Construct the camera object*/
        //cameraT265 = new CameraT265();

        /* Initialize T265 camera */
        if (cameraT265->init()) {
            cout << "Camera initialization succeeded!" << endl;
        } else {
            throw std::runtime_error("Camera initialization failed!");
        }
        cameraT265->setOrigin();
        cout << "X offset is: " << cameraT265->getXOffset() << ", Y offset is: " << cameraT265->getYOffset() << endl;
    #endif
#endif
    }
}

void PCVBaseNode::spin(){
    /*Launch control thread */
    launch_rt_thread (control_thread, &control, this, MAX_PRIO-1);
    
    /* main loop - receive events from controller */
    tf::TransformBroadcaster odomBroadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    ros::Time current_time;
    robotElectrical_T eStatus_tmp;
    omniveyor_common::electricalStatus eStatus;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 0.001;
    odom.pose.covariance[21] = 0.001;
    odom.pose.covariance[28] = 0.001;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 0.001;
    odom.twist.covariance[21] = 0.001;
    odom.twist.covariance[28] = 0.001;
    odom.twist.covariance[35] = 0.01;

    Eigen::Vector3d gx      = Eigen::Vector3d::Zero();
    Eigen::Vector3d gxd     = Eigen::Vector3d::Zero();
    const float sin_PI_4    = -sqrt(2.)*0.5;
    const float cos_PI_4    = sqrt(2.)*0.5;

    ros::Rate pub_rate(pubRate);
    while (ros::ok())
    {

#ifdef BUMPER_SENSORS
        if (vehicle->isInitialized()){
            //"SAFETY" -- stop vehicle and program if bumper is hit
            if (vehicle->isBumperHit()) {
                cout << "Bumper hit: " << vehicle->getBumperState() << endl; // Todo: remove print statement - not safe
                sig_handler_int(0);
            }
        }
#endif

        // ROS Code Goes Here.
        current_time = ros::Time::now();
        gx = vehicle->getGlobalPosition();
        gxd = vehicle->getGlobalVelocity();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(gx(2));
        vehicle->getElectricalStatus(&eStatus_tmp);
        
        /*
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = gx(0);// *cos_PI_4 + gx(1)*sin_PI_4;
        odom_trans.transform.translation.y = gx(1);// *sin_PI_4 + gx(1)*cos_PI_4;
        odom_trans.transform.rotation = odom_quat;
        odomBroadcaster.sendTransform(odom_trans);
        // if using RF2O+EKF, disable tf publication from the base to avoid multiple parent frame issue.*/

        odom.header.stamp = current_time;
        odom.pose.pose.position.x = gx(0);
        odom.pose.pose.position.y = gx(1);
        odom.pose.pose.orientation = odom_quat;
        
        odom.twist.twist.linear.x = gxd(0);
        odom.twist.twist.linear.y = gxd(1);
        odom.twist.twist.angular.z = gxd(2);
        
        odomPub.publish(odom);
        
        eStatus.stamp = current_time;
        eStatus.steer_1_Volt = eStatus_tmp.steerMotorVoltage[0];
        eStatus.steer_2_Volt = eStatus_tmp.steerMotorVoltage[1];
        eStatus.steer_3_Volt = eStatus_tmp.steerMotorVoltage[2];
        eStatus.steer_4_Volt = eStatus_tmp.steerMotorVoltage[3];
        eStatus.steer_1_Amp = eStatus_tmp.steerMotorCurrent[0];
        eStatus.steer_2_Amp = eStatus_tmp.steerMotorCurrent[1];
        eStatus.steer_3_Amp = eStatus_tmp.steerMotorCurrent[2];
        eStatus.steer_4_Amp = eStatus_tmp.steerMotorCurrent[3];
        eStatus.roll_1_Volt = eStatus_tmp.rollMotorVoltage[0];
        eStatus.roll_2_Volt = eStatus_tmp.rollMotorVoltage[1];
        eStatus.roll_3_Volt = eStatus_tmp.rollMotorVoltage[2];
        eStatus.roll_4_Volt = eStatus_tmp.rollMotorVoltage[3];
        eStatus.roll_1_Amp = eStatus_tmp.rollMotorCurrent[0];
        eStatus.roll_2_Amp = eStatus_tmp.rollMotorCurrent[1];
        eStatus.roll_3_Amp = eStatus_tmp.rollMotorCurrent[2];
        eStatus.roll_4_Amp = eStatus_tmp.rollMotorCurrent[3];
        
        eStatusPub.publish(eStatus);

        ros::spinOnce();
        pub_rate.sleep();

#ifdef JOYSTICK
        struct event e;
        mq_receive (mq_joystick, (char *)&e, sizeof (e), NULL);
        switch (e.type)
        {
            case NEW_Xd_COMMAND:
                cur_x_dot = MAX_X_VEL * ((int32_t)e.param) / 32767;
                break;

            case NEW_Yd_COMMAND:
                cur_y_dot = MAX_X_VEL * ((int32_t)e.param) / 32767;
                break;

            case NEW_THETAd_COMMAND:
                cur_theta_dot = MAX_THETA_VEL * ((int32_t)e.param) / 32767;
                break;

            if (vehicle->isInitialized()) {
                // handle button events
                case BUTTON_PRESSED:
                {
                    switch (e.param)
                    {
                        // "SAFETY" -- convenience stop!
                        case A_BUTTON:
                            cout << "A button pressed" << endl;
                            // "A" button toggles enable/disable of vehicle
                            vehicle->isEnabled() ? vehicle->disable() : vehicle->enable();
                        break;
                        case X_BUTTON:
                            cout << "X button pressed" << endl;
                            // "X" changes the control mode to torque control
                            control_mode = TORQUE;
                            vehicle->disable();
                            vehicle->setCtrlMode(control_mode);
                            vehicle->enable();
                        break;
                        case Y_BUTTON:
                            cout << "Y button pressed" << endl;
                            // "B" changes the control mode to velocity control
                            control_mode = VELOCITY;
                            vehicle->disable();
                            vehicle->setCtrlMode(control_mode);
                            vehicle->enable();
                        break;

                        default:
                        break;
                    }
                }
            }
            default:
                break;
        }
#elif defined KEYBOARD
        // "SAFETY" -- convenience stop!
        if (kbhit()) {
            cout << "Key is hit" << endl; // Todo: remove print statement - not safe
            sig_handler_int(0);
        }
#endif
    }
}

PCVBaseNode::~PCVBaseNode(){
    int status = pthread_kill( control , SIGINT);
    if ( status <  0 )
        perror("pthread_kill control thread failed");
    else
        printf("Control thread stopped\r\n");
    delete vehicle;
    printf("Exiting main thread\r\n");
}

/*------------------------------static functions------------------------------*/
/*
 * This function defines the control thread, it utilizes the SYNC message for
 * sending control commands and receiving data from the drivers. In the first
 * half of the control loop, we send a SYNC message and wait for the data to
 * come back from the driver. Then we use that data to calculate the next
 * control effort and update that with a call to the vehicle::set_velocity
 * method. Nothing happens in the second half of the control loop.
 */
void *
PCVBaseNode::control_thread (void *aux)
{
    struct CO_message msgSync;
    msgSync.type = SYNC;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    /* variable to keep track of number of control loops */
    unsigned long ticks = 0;

    /* Initialize variables */
    //Eigen::Vector3d vel_commands = Eigen::Vector3d::Zero();
    Eigen::Vector3d gx_des	 = Eigen::Vector3d::Zero();
    Eigen::Vector3d gxd_des  = Eigen::Vector3d::Zero();
    Eigen::Vector3d gxdd_des = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d gx       = Eigen::Vector3d::Zero();
    Eigen::Vector3d gxd      = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_local  = Eigen::Vector3d::Zero();
    Eigen::Vector3d xd_local = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 8, 1> qd;
    Eigen::Matrix<double, 8, 1> qd_des;
    Eigen::Matrix<double, 4, 1> joint;
    Eigen::Matrix<double, NUM_MOTORS, 1> tq_des;
    Eigen::Vector3d cf_des_local;
    Eigen::Matrix3d lambda;
    Eigen::MatrixXd C_pinv;
    Eigen::Matrix<double, NUM_CASTERS, 1> q_steer;
    Eigen::Vector3d gx_cam = Eigen::Vector3d::Zero();
    float th_cam = 0.0;

    double max_linear_dist = 0.02;
    double max_ang_dist = 0.1;

#ifdef LINE
    enum ln_case
    {
        STRAIGHT,
        SPIN
    };

    enum ln_case line_case = STRAIGHT;

    gx_des << 0.5, 0.0, 0.0;
#endif

#ifdef SQUARE
    enum sqr_case
    {
        BOTTOM_LEFT,
        BOTTOM_LEFT_TWIST,
        TOP_LEFT,
        TOP_LEFT_TWIST,
        TOP_RIGHT,
        TOP_RIGHT_TWIST,
        BOTTOM_RIGHT,
        BOTTOM_RIGHT_TWIST
    };

    enum sqr_case square_case = BOTTOM_LEFT;
#endif

#ifdef USING_OTG
    Eigen::VectorXd step_desired_position = gx_des;
    Eigen::VectorXd step_desired_velocity = gxd_des;
    OTG* otg = new OTG(gx, CONTROL_PERIOD_s);
    Eigen::Vector3d max_vel;
    max_vel << MAX_VEL_X, MAX_VEL_Y, MAX_VEL_TH;
    otg->setMaxVelocity(max_vel);
    otg->setMaxAcceleration(2.0);
    otg->setMaxJerk(20.0);
    otg->reInitialize(gx);
#endif

    // Disable HB since we are receiving synced status update.
    struct CO_message msg_hb_disable;
    msg_hb_disable.type = SDO_Rx;
    msg_hb_disable.m.SDO = {PROD_HEARTBEAT_TIME, 0x00, 0, 2};
    for (int k = 1; k < 9; k++)
        CO_send_message (vehicle->s, k, &msg_hb_disable);
    usleep(1000);

    /* initial sync message */
    CO_send_message (vehicle->s, 0, &msgSync);
    sleep_until (&next, CONTROL_PERIOD_ns);

    static bool vehicle_spinning = false;
    static unsigned long start_time = 0;

    /* Initialize loop timer */
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_previous_loop_start = t_start;

    //gxd_des << 0.0, 0.0, 0.1;
    //vehicle->setGlobalVelocity(gxd_des);
    //usleep(100000);
    //gxd_des << 0.0, 0.0, -0.1;
    //vehicle->setGlobalVelocity(gxd_des);
    //usleep(100000);
    gxd_des << 0.0, 0.0, 0.0;
    vehicle->setGlobalVelocity(gxd_des);

    // vehicle starts up stopped after calibration.
    vehicle->stop();

    while (vehicle->isInitialized())
    {
    /* ---------- first half of control loop ---------- */
        /* Get loop timestamp */
        auto t_loop_start = std::chrono::high_resolution_clock::now();
        double t_delta = (std::chrono::duration<double, std::milli>(t_loop_start - t_previous_loop_start).count()); // Get time it took to complete one loop to plot
        t_previous_loop_start = t_loop_start;

        /* Update odometry */
        vehicle->updateOdometry();

    #ifdef CAMERA
        /* Update camera odometry */
        cameraT265->updatePose();
    #endif

        /* send sync message */
        //CO_send_message (vehicle->s, 0, &msg);
        //usleep (3500);
        //rate.sleep();

        /***************** Write data to csv file *********************************/
        if (dumpData)
        {
            gx = vehicle->getGlobalPosition();  //Odom
            gxd = vehicle->getGlobalVelocity(); //Odom
            tq_des = vehicle->getDesJointTorques();
            cf_des_local = vehicle->getLocalCommandForces();
            lambda = vehicle->getLambda();
            q_steer = vehicle->getJointSteeringAngles();
            x_local = vehicle->getLocalPosition();
            xd_local = vehicle->getLocalVelocity();
            C_pinv = vehicle->getCPinv();
            qd = vehicle->getJointVelocities();
            file << (ticks*CONTROL_PERIOD_s) << "," << t_delta << ",";
            file << gx(0) << "," << gx(1) << "," << gx(2) << ",";
            file << gx_des(0) << "," << gx_des(1) << "," << gx_des(2) << ",";
            file << gxd(0) << "," << gxd(1) << "," << gxd(2);
            for(int i = 0; i < NUM_MOTORS; i++){
                file << "," << tq_des(i);
            }
            file << "," << cf_des_local(0) << "," << cf_des_local(1) << "," << cf_des_local(2);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    file << "," << lambda(i,j);
                }
            }
            file << "," << q_steer(0) << "," << q_steer(1) << "," << q_steer(2) << "," << q_steer(3) << ",";
            file << step_desired_position(0) << "," << step_desired_position(1) << "," << step_desired_position(2) << ",";
            file << step_desired_velocity(0) << "," << step_desired_velocity(1) << "," << step_desired_velocity(2) << ",";
        #ifdef CAMERA
            gx_cam = cameraT265->getCameraPosition();
            th_cam = cameraT265->getThCam();
            file << gx_cam(0) << "," << gx_cam(1) << "," << gx_cam(2);
            file << "," << th_cam;
        #else
            file << 0 << "," << 0 << "," << 0;
            file << "," << th_cam;
        #endif
            file << "," << vehicle->getHeading();
            file << "," << x_local(0) << "," << x_local(1) << "," << x_local(2);
            file << "," << xd_local(0) << "," << xd_local(1) << "," << xd_local(2);
            file << "," << qd(0) << "," << qd(1) << "," << qd(2) << "," << qd(3);
            for (int i = 0; i < 24; i++){
                file << "," << C_pinv(i);
            }
            file << endl;
        }
        /**************************************************************************/
        
        if (!vehicle->isStopped()){
            switch(control_mode) {
                case TORQUE:
                {
                    auto t_end = std::chrono::high_resolution_clock::now();
                    double t_curr = (std::chrono::duration<double, std::milli>(t_end-t_start).count())/1000;
                    // gx_des << 0.25 * cos(t_curr/4), 0.25*sin(t_curr/4), 0;
                    // gx_des << 2.0* fabs(cos(t_curr/6)), 0, t_curr/3.0; // Follow a line
                    // gx_des << 0.5, 0, 0;

                    gx = vehicle->getGlobalPosition();

                #ifdef SPIN
                    gx_des << 0.0, 0.0, t_curr; // Spin in place
                    // Change directions after 10 seconds
                    // if (t_curr > 10) {
                    // 	gx_des << 0.0, 0.0, 10-t_curr;
                    // }
                #endif

                #ifdef LINE
                    if(vehicle->reachedTarget(gx, gx_des, max_linear_dist, max_ang_dist))
                    {
                        switch(line_case){
                            case STRAIGHT:
                            {
                                // gx_des[2] += M_PI/3.0;
                                line_case = SPIN;
                                break;
                            }
                            case SPIN:
                            {
                                gx_des[0] = -gx_des[0];
                                line_case = STRAIGHT;
                                break;
                            }
                        }
                    }
                #endif

                #ifdef SQUARE
                    if(vehicle->reachedTarget(gx, gx_des, max_linear_dist, max_ang_dist))
                    {
                        switch(square_case) {
                            case BOTTOM_LEFT:
                            {
                                gx_des[2] += M_PI / 2.0;
                                square_case = BOTTOM_LEFT_TWIST;
                                break;
                            }
                            case BOTTOM_LEFT_TWIST:
                            {
                                gx_des[0] += 0.5;
                                square_case = TOP_LEFT;
                                break;
                            }
                            case TOP_LEFT:
                            {
                                gx_des[2] += M_PI / 2.0;
                                square_case = TOP_LEFT_TWIST;
                                break;
                            }
                            case TOP_LEFT_TWIST:
                            {
                                gx_des[1] += 0.5;
                                square_case = TOP_RIGHT;
                                break;
                            }
                            case TOP_RIGHT:
                            {
                                gx_des[2] += M_PI/ 2.0;
                                square_case = TOP_RIGHT_TWIST;
                                break;
                            }
                            case TOP_RIGHT_TWIST:
                            {
                                gx_des[0] -= 0.5;
                                square_case = BOTTOM_RIGHT;
                                break;
                            }
                            case BOTTOM_RIGHT:
                            {
                                gx_des[2] += M_PI / 2.0;
                                square_case = BOTTOM_RIGHT_TWIST;
                                break;
                            }
                            case BOTTOM_RIGHT_TWIST:
                            {
                                gx_des[1] -= 0.5;
                                square_case = BOTTOM_LEFT;
                                break;
                            }
                            default:
                                break;
                        }
                    }
                #endif

                    gx_des << gx_des_g[0], gx_des_g[1], gx_des_g[2];
                    gxd_des << gxd_des_g[0], gxd_des_g[1], gxd_des_g[2];
                    gxdd_des << gxdd_des_g[0], gxdd_des_g[1], gxdd_des_g[2];
                #ifdef USING_OTG
                    otg->setGoalPositionAndVelocity(gx_des, gxd_des);
                    otg->computeNextState(step_desired_position, step_desired_velocity);
                    vehicle->setTargets(step_desired_position,step_desired_velocity,gxdd_des);
                #else
                    vehicle->setTargets(gx_des, gxd_des, gxdd_des);                         // Torque Control Input
                #endif
                    break;
                }

                case VELOCITY:
                {
                    gxd_des << gxd_des_g[0], gxd_des_g[1], gxd_des_g[2];
                    vehicle->setGlobalVelocity(gxd_des);
                    //printf("%f, %f, %f\r\n", gxd_des(0), gxd_des(1), gxd_des(2));
                    break;
                }

                default:
                {
                    //delete vehicle;
                    cout << "Unimplemented control mode: " << control_mode << endl;
                    vehicle->stop();
                    break;
                    //return 0;
                }
            }
        
            //rate.sleep();
            //sleep_until (&next, CONTROL_PERIOD_ns/2); 
            /* --------- second half of control loop --------- */
            /* send sync message */
            //usleep(1500);
            CO_send_message (vehicle->s, 0, &msgSync);          // 100Hz fast query
        }
        else{
            if (ticks%128==0)
                CO_send_message (vehicle->s, 0, &msgSync);  // 1Hz slow query
        }

        ticks++;

        /* do nothing in this half */
        sleep_until (&next, CONTROL_PERIOD_ns);
        //rate.sleep();
    }
    printf("Exiting control thread ... \r\n");
    return 0;
}



/*
 * Helper function to handle accurate sleeping inside of the control
 * thread. This provides an easier abstraction to the clock_nanosleep
 * interface
 */
inline void
PCVBaseNode::sleep_until (struct timespec *ts, long delay)
{
    ts->tv_nsec += delay;
    if (ts->tv_nsec >= 1000*1000*1000)
    {
        ts->tv_nsec -= 1000*1000*1000;
        ts->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}


/*
 * Function for startup tasks. Installs the signal handler for SIGINT signals,
 * and opens a message queue with the controller if we're not testing.
 */
void
PCVBaseNode::perform_startup_tasks (void)
{
    /* install the signal handler for abnormal program exit */
    //struct sigaction sa1 = {0};
    //sa1.sa_handler = sig_handler_int;
    //sigfillset (&sa1.sa_mask);
    //sigaction (SIGINT, &sa1, NULL);
    
    struct sigaction sa2 = {0};
    sa2.sa_handler = sig_handler_tstp;
    sigfillset (&sa2.sa_mask);
    sigaction (SIGTSTP, &sa2, NULL);
    
    struct sigaction sa3 = {0};
    sa3.sa_handler = sig_handler_cont;
    sigfillset (&sa3.sa_mask);
    sigaction (SIGCONT, &sa3, NULL);

#ifdef JOYSTICK
    /* open message queue with joystick */

    const char *name = "/joystick message queue";
    mq_joystick = mq_open (name, 0);

    if (mq_joystick == -1)
    {
        perror ("failed to open message queue in init_mQueue\n");
        exit (-1);
    }
#endif
}


/*
 * Parses command line args and sets various environment variables
 */
void
PCVBaseNode::parse_command_args (int argc, char *argv[])
{
    for (int i = 0; i < argc; i++)
    {
        std::string str (argv[i]);
        if (str == "--dump")
            dumpData = true;
    }
}


/*
 * Signal handler for SIGINT, used to properly destruct the vehicle object
 * if the program aborts abnormally
 */
/*
void
PCVBaseNode::sig_handler_int (int)
{
    cout << endl;
    cout << "SIGINT received, destroying vehicle and exiting\r\n" << std::endl;
    delete vehicle;
    if (dumpData)
    {
        file.close ();
        cout << "Closing file \r\n" << endl;
    }
    exit (0);
}
*/

void
PCVBaseNode::sig_handler_tstp (int)
{
    vehicle->stop();
    /*
    // this method will stop all communications of the motor, causing vast timeouts.
    struct CO_message msg_stop;
    msg_stop.type = NMT;
    msg_stop.m.NMT = {0x02};
    for (int k = 1; k < 9; k++)
        CO_send_message (vehicle->s, k, &msg_stop);
    usleep(1000);
    // re-enabling heartbeat -- untested
    struct CO_message msg_hb_enable;
    msg_hb_disable.type = SDO_Rx;
    msg_hb_disable.m.SDO = {0x1017, 0x00, 50, 2};
    for (int k = 1; k < 9; k++)
        CO_send_message (vehicle->s, k, &msg_hb_enable);
    usleep(1000);
    */
}

void
PCVBaseNode::sig_handler_cont (int)
{
    vehicle->start();	
    /*struct CO_message msg_cont;
    msg_cont.type = NMT;
    msg_cont.m.NMT = {0x01};
    for (int k = 1; k < 9; k++)
        CO_send_message (vehicle->s, k, &msg_cont);
    usleep(1000);
    // re-disabling heartbeat -- untested
    struct CO_message msg_hb_enable;
    msg_hb_disable.type = SDO_Rx;
    msg_hb_disable.m.SDO = {0x1017, 0x00, 0, 2};
    for (int k = 1; k < 9; k++)
        CO_send_message (vehicle->s, k, &msg_hb_enable);
    usleep(1000);
    */
}


/*
 * Registers keystrokes and returns 1 if a key has been hit. Note, this function
 * only works on Linux
 */
#ifdef KEYBOARD
int PCVBaseNode::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if((ch != EOF) && (ch!='\r'))
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
#endif



int
main (int argc, char *argv[])
{
    // if(argc < 3) {
    //     puts("Usage: sudo ./vehicle <Kp_theta> <Kv_theta>");
    //     exit(0);
    // }

    ros::init(argc, argv, "PCV_Base");
    ros::NodeHandle rosNH;

    try{
        PCVBaseNode PCVBaseNode(argc, argv, &rosNH);
        PCVBaseNode.spin();
    }
    catch (const std::runtime_error& e){
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}