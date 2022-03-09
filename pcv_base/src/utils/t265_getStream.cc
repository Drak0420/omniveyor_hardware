// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <math.h>

using namespace std;

int main(int argc, char * argv[]) try
{

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Create string of x data
        string x = to_string(-pose_data.translation.z) + "," + to_string(-pose_data.translation.x) + "," + to_string(2*acos(pose_data.rotation.w));
        string xd = to_string(-pose_data.velocity.z) + "," + to_string(-pose_data.velocity.x) + "," + to_string(pose_data.angular_velocity.y);
        string xdd = to_string(-pose_data.acceleration.z) + "," + to_string(-pose_data.acceleration.x) + "," + to_string(pose_data.angular_acceleration.y);
        cout << "X: " << x << endl; 

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
