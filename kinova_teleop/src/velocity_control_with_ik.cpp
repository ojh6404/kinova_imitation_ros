#include <iostream>
#include <queue>
#include <numeric>
#include <math.h>
#include <atomic>
#include <thread>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PointStamped.h"
#include "kortex_driver/Base_JointSpeeds.h"
#include "kortex_driver/JointSpeed.h"
#include "kortex_driver/BaseCyclic_Feedback.h"
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/Base_JointSpeeds.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

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

//KDL::JntArray jnt_pos_start(NUM_JOINTS);
KDL::JntArray current_joint_pos(NUM_JOINTS);
KDL::JntArray target_joint_pos(NUM_JOINTS);
KDL::JntArray init_joint_pos(NUM_JOINTS);


std::array<float, 3> rpy;
std::array<float, 3> tool_pos;
std::array<float, 3> wrist_point_global;
std::array<float, 3> wrist_point_offset = {0.0f, 0.0f, 0.0f};
std::array<float, 3> init_point;
std::array<float, 3> target_point;

float point_gain[3] = {1.5f, 1.5f, 1.5f};
float joint_gain[7] = {3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f};


std::atomic<int> last_action_notification_event{0};

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

bool wait_for_action_end_or_abort() {
    while (ros::ok()) {
        if (last_action_notification_event.load() ==
            kortex_driver::ActionEvent::ACTION_END) {
            ROS_INFO("Received ACTION_END notification");
            return true;
        } else if (last_action_notification_event.load() ==
                   kortex_driver::ActionEvent::ACTION_ABORT) {
            ROS_INFO("Received ACTION_ABORT notification");
            return false;
        }
        ros::spinOnce();
    }
    return false;
}

bool reset_pose(ros::NodeHandle n) {
    ros::ServiceClient service_client_read_action =
            n.serviceClient<kortex_driver::ReadAction>("/arm_gen3/base/read_action");
    kortex_driver::ReadAction service_read_action;
    last_action_notification_event = 0;

    // The Home Action is used to home the robot. It cannot be deleted and is
    // always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if (!service_client_read_action.call(service_read_action)) {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // We can now execute the Action that we read
    ros::ServiceClient service_client_execute_action =
            n.serviceClient<kortex_driver::ExecuteAction>("/arm_gen3/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;

    if (service_client_execute_action.call(service_execute_action)) {
        ROS_INFO("Send reset pose to the robot");
    } else {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    return true;
//    return wait_for_action_end_or_abort();
}



void cb_wrist_point(const geometry_msgs::PointStamped::ConstPtr & wrist_point) {

//    wrist_point_global = wrist_point->point[0];
    wrist_point_global[0] = wrist_point->point.x;
    wrist_point_global[1] = wrist_point->point.y;
    wrist_point_global[2] = wrist_point->point.z;
}

void cb_base_feedback(const kortex_driver::BaseCyclic_Feedback::ConstPtr & base_feedback) {
    tool_pos[0] = base_feedback->base.tool_pose_x;
    tool_pos[1] = base_feedback->base.tool_pose_y;
    tool_pos[2] = base_feedback->base.tool_pose_z;
    rpy[0] = deg2rad(base_feedback->base.tool_pose_theta_x);
    rpy[1] = deg2rad(base_feedback->base.tool_pose_theta_y);
    rpy[2] = deg2rad(base_feedback->base.tool_pose_theta_z);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        current_joint_pos(i) = deg2rad(base_feedback->actuators[i].position);
    }

}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
    return((q_goal - q_start) * (t/t_max) + q_start);
}



int main(int argc, char** argv) {

    // Get kinova URDF model
    std::string urdf_path = ros::package::getPath("kinova_teleop");
    if(urdf_path.empty()) {
        ROS_ERROR("kinova_teleop package path was not found");
        return 0;
    }
    urdf_path += URDF_PATH;
    std::cout << "Using model file : " << urdf_path << std::endl;

    // Initilize ROS node
    ros::init(argc, argv, "arm_gen3_velocity_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);

    //Create subscribers for all joint states
    ros::Subscriber sub_wrist_point = n.subscribe("/wrist_point/output/wrist_point", 10, cb_wrist_point);
    ros::Subscriber sub_base_feedback = n.subscribe("/arm_gen3/base_feedback", 10, cb_base_feedback);

    //Create publishers to send position commands to all joints
    ros::Publisher pub_joint_vel = n.advertise<kortex_driver::Base_JointSpeeds>("/arm_gen3/in/joint_velocity", 10);

    //Parse URDF model and generate KDL tree
    KDL::Tree kinova_gen3_arm_tree;
    if (!kdl_parser::treeFromFile(urdf_path, kinova_gen3_arm_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return 0;
    }

    //Generate a kinematic chain from the robot base to its tcp
    KDL::Chain kinova_gen3_arm_chain;
    kinova_gen3_arm_tree.getChain("base_link", "kinova_grip_site", kinova_gen3_arm_chain);
    if (kinova_gen3_arm_chain.getNrOfJoints() != NUM_JOINTS){
        ROS_ERROR("Number of Joints doesn't match");
        return 0;
    }

    //Create FK and IK solvers
    KDL::ChainFkSolverPos_recursive fk_solver(kinova_gen3_arm_chain);
    KDL::ChainIkSolverVel_pinv vel_ik_solver(kinova_gen3_arm_chain);
    KDL::ChainIkSolverPos_NR ik_solver(kinova_gen3_arm_chain, fk_solver, vel_ik_solver, 100);

    // Set init joint pos and compute FK
    init_joint_pos(0) = 0.0;
    init_joint_pos(1) = 0.26;
    init_joint_pos(2) = 3.14;
    init_joint_pos(3) = -2.27;
    init_joint_pos(4) = 0.0;
    init_joint_pos(5) = 0.95;
    init_joint_pos(6) = 1.57;
    KDL::Frame init_cartesian_pose;
    fk_solver.JntToCart(init_joint_pos, init_cartesian_pose);

    double roll, pitch, yaw;
    init_cartesian_pose.M.GetRPY(roll,pitch,yaw);
    ROS_INFO("Initial cartesian pose vector : %.3f, %.3f, %.3f, cartesian pose rot %.3f, %.3f, %.3f", init_cartesian_pose.p(0), init_cartesian_pose.p(1), init_cartesian_pose.p(2),roll, pitch, yaw);


    int64_t start = GetTickUs();
    int64_t last;



    reset_pose(n);

    // Initialize and compute offset of wrist point
    std::queue<float> init_wrist_point_x;
    std::queue<float> init_wrist_point_y;
    std::queue<float> init_wrist_point_z;

    ROS_INFO("Start Initilizing...");
    while (GetTickUs()-start < 1000 * 1000 * 5.0) {
        if (init_wrist_point_x.size() >= 100) {
            init_wrist_point_x.pop();
            init_wrist_point_y.pop();
            init_wrist_point_z.pop();
            init_wrist_point_x.push(wrist_point_global[0]);
            init_wrist_point_y.push(wrist_point_global[1]);
            init_wrist_point_z.push(wrist_point_global[2]);

        } else {
            init_wrist_point_x.push(wrist_point_global[0]);
            init_wrist_point_y.push(wrist_point_global[1]);
            init_wrist_point_z.push(wrist_point_global[2]);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    for (int i = 0; i < init_wrist_point_x.size(); ++i) {
        wrist_point_offset[0] += init_wrist_point_x.front();
        wrist_point_offset[1] += init_wrist_point_y.front();
        wrist_point_offset[2] += init_wrist_point_z.front();
        init_wrist_point_x.pop();
        init_wrist_point_y.pop();
        init_wrist_point_z.pop();
    }

    wrist_point_offset[0] /= init_wrist_point_x.size();
    wrist_point_offset[1] /= init_wrist_point_y.size();
    wrist_point_offset[2] /= init_wrist_point_z.size();

    ROS_INFO("Wrist point offset is %.3f, %.3f, %.3f", wrist_point_offset[0], wrist_point_offset[1], wrist_point_offset[2]);

    const float t_step = 1/((float)LOOP_RATE);
    int count = 0;


    ROS_INFO("Velocity control start");
    kortex_driver::Base_JointSpeeds base_jointspeeds_msg;
    kortex_driver::JointSpeed joint_speed_msg;


    KDL::Vector target_cartesian_pose_vec = init_cartesian_pose.p;
    KDL::Rotation target_cartesian_pose_rot = init_cartesian_pose.M;


    while (ros::ok()) {

        // Get time Tick
        start = GetTickUs();

        for (int i = 0; i < 3; ++i) {
            target_point[i] = point_gain[i] * (wrist_point_global[i] - wrist_point_offset[i]);
        }

        target_cartesian_pose_vec.x(init_cartesian_pose.p(0) + target_point[0]);
        target_cartesian_pose_vec.y(init_cartesian_pose.p(1) + target_point[1]);
        target_cartesian_pose_vec.z(init_cartesian_pose.p(2) + target_point[2]);
        KDL::Frame target_cartesian_pose(target_cartesian_pose_rot ,target_cartesian_pose_vec);

//        KDL::JntArray jnt_pos_goal(NUM_JOINTS);
        ik_solver.CartToJnt(current_joint_pos, target_cartesian_pose,target_joint_pos);

        base_jointspeeds_msg.joint_speeds.clear();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_speed_msg.joint_identifier = i;
            joint_speed_msg.duration = 0;
            joint_speed_msg.value = joint_gain[i] * (target_joint_pos(i) - current_joint_pos(i));
            base_jointspeeds_msg.joint_speeds.push_back(joint_speed_msg);
        }

        // Publish velocity
        pub_joint_vel.publish(base_jointspeeds_msg);

        // Calculate time tick
        last = GetTickUs();
//        std::cout << "Time : " << last - start << std::endl;

        // spin and sleep
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;

}
