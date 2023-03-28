#include <iostream>
#include <queue>
#include <numeric>
#include <math.h>
#include <atomic>
#include <thread>

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define rad2deg(a) ((a)/M_PI * 180.0)
#define deg2rad(a) ((a)/180.0 * M_PI)
#define NUM_JOINTS 7
#define LOOP_RATE 40.0
#define URDF_PATH "/urdf/gen3_robotiq_2f_140.urdf"
#define HOME_ACTION_IDENTIFIER 2
#define ROBOT_NAME = "arm_gen3"
#define INIT_TIME = 5.0

std_msgs::String hand_status_global;

int64_t GetTickUs() {
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

bool send_gripper_command(ros::NodeHandle n, double value)
{
    // Initialize the ServiceClient
    ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/arm_gen3/base/send_gripper_command");
    kortex_driver::SendGripperCommand service_send_gripper_command;

    // Initialize the request
    kortex_driver::Finger finger;
    finger.finger_identifier = 0;
    finger.value = value;
    service_send_gripper_command.request.input.gripper.finger.push_back(finger);
    service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

    if (service_client_send_gripper_command.call(service_send_gripper_command))
    {
//        ROS_INFO("The gripper command was sent to the robot.");
    }
    else
    {
        std::string error_string = "Failed to call SendGripperCommand";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
//    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    return true;
}

void cb_hand_status(const std_msgs::String ::ConstPtr & hand_status) {
    hand_status_global.data = hand_status->data;
}

int main(int argc, char** argv) {


    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);

    //Create subscribers for all joint states
    ros::Subscriber sub_wrist_point = n.subscribe("/hand_status", 10, cb_hand_status);

    //Create publishers to send position commands to all joints

    int64_t start = GetTickUs();
    int64_t last;


    while (ros::ok()) {

        // Get time Tick
        start = GetTickUs();

        if (hand_status_global.data == "grab") {
//            std::cout << hand_status_global.data << std::endl;
            send_gripper_command(n, 1.0);
        } else if (hand_status_global.data == "release") {
            send_gripper_command(n, 0.0);
        }

        // Calculate time tick
        last = GetTickUs();
//        std::cout << "Time : " << last - start << std::endl;



        // spin and sleep
        ros::spinOnce();
        loop_rate.sleep();


    }


    return 0;

}

