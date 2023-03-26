#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "kortex_driver/Base_JointSpeeds.h"
#include "kortex_driver/JointSpeed.h"
#include "kortex_driver/BaseCyclic_Feedback.h"
#include <ros/package.h>

#include <iostream>

#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define rad2deg(a) ((a)/M_PI * 180.0)
#define deg2rad(a) ((a)/180.0 * M_PI)

const int Joints = 7;
KDL::JntArray jnt_pos_start(Joints);
const int loop_rate_val = 50;

geometry_msgs::PointStamped wrist_point_global;
sensor_msgs::JointState joint_state_global;
std::array<float, 3> rpy;
//kortex_driver::BaseCyclic_Feedback base_feedback_global;


//void cb_wrist_point(const geometry_msgs::PointStamped::ConstPtr & wrist_point) {
//
//    wrist_point_global = wrist_point->point[0];
//}

void cb_base_feedback(const kortex_driver::BaseCyclic_Feedback::ConstPtr & base_feedback) {
//    for (int i = 0; i < 7; ++i) {
//        base_feedback_global.actuators.data(i)->position = base_feedback->actuators.data(i)->position;
//    }
//    for (int i = 0; i < Joints; ++i) {
////        base_feedback_global.actuators[i].position. = base_feedback->actuators[i].position;
//        std::cout << i << "th joint pos : " << base_feedback->actuators[i].position << std::endl;
//    }

    rpy[0] = deg2rad(base_feedback->base.tool_pose_theta_x);
    rpy[1] = deg2rad(base_feedback->base.tool_pose_theta_y);
    rpy[2] = deg2rad(base_feedback->base.tool_pose_theta_z);

//    for (int i = 0; i < 3; ++i) {
//        std::cout << i << "th rpy : " << rpy[i] << std::endl;
//    }

//    std::cout << base_feedback->actuators.data()->position << std::endl;
}

void cb_joint_state(const sensor_msgs::JointState::ConstPtr & joint_state) {
    for (int i = 0; i < Joints; ++i) {
        jnt_pos_start(i) = joint_state->position[i+1];
//        std::cout << i << "th joint : " << joint_state->position[i];
    }
}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
    return((q_goal - q_start) * (t/t_max) + q_start);
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

    std::cout << "Please define the offset you want to move for each axis and the time in which the motion should be completed:\n";

    //Get user input
    float x,y,z;
    std::cout << "x:";
    std::cin >> x;
    std::cout << "y:";
    std::cin >> y;
    std::cout << "z:";
    std::cin >> z;
    std::cout << "Time:";
    std::cin >> (*t_max);

    //Compute goal position
    (*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
    (*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
    (*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}


int main(int argc, char** argv) {

    std::string urdf_path = ros::package::getPath("kinova_test");
    if(urdf_path.empty()) {
        ROS_ERROR("kinova_test package path was not found");
    }
    urdf_path += "/urdf/gen3_robotiq_2f_140.urdf";

    std::cout << urdf_path << std::endl;


    ros::init(argc, argv, "cart_pos");

    ros::NodeHandle n;

    ros::Rate loop_rate(loop_rate_val);

    //Create subscribers for all joint states
//    ros::Subscriber sub_wrist_point = n.subscribe("/wrist_point/output/wrist_point", 10, cb_wrist_point);
    ros::Subscriber sub_joint_states = n.subscribe("/arm_gen3/joint_states", 10, cb_joint_state);
    ros::Subscriber sub_base_feedback = n.subscribe("/arm_gen3/base_feedback", 10, cb_base_feedback);

    //Create publishers to send position commands to all joints
//    ros::Publisher pub_joint_vel = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
    ros::Publisher pub_test_joint_states = n.advertise<sensor_msgs::JointState>("/test/output", 1);
    sensor_msgs::JointState test_joint_state;

    //Create publishers to send position commands to all joints
    ros::Publisher pub_joint_vel = n.advertise<kortex_driver::Base_JointSpeeds>("/arm_gen3/in/joint_velocity", 10);


    //Parse urdf model and generate KDL tree
    KDL::Tree kinova_gen3_arm_tree;
    if (!kdl_parser::treeFromFile(urdf_path, kinova_gen3_arm_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    //Generate a kinematic chain from the robot base to its tcp
    KDL::Chain kinova_gen3_arm_chain;
//    kinova_gen3_arm_tree.getChain("base_link", "end_effector_link", kinova_gen3_arm_chain);
    kinova_gen3_arm_tree.getChain("base_link", "kinova_grip_site", kinova_gen3_arm_chain);


    //Create solvers
    KDL::ChainFkSolverPos_recursive fk_solver(kinova_gen3_arm_chain);
    KDL::ChainIkSolverVel_pinv vel_ik_solver(kinova_gen3_arm_chain, 0.0001, 100);
    KDL::ChainIkSolverPos_NR ik_solver(kinova_gen3_arm_chain, fk_solver, vel_ik_solver, 100);

    unsigned int nj = kinova_gen3_arm_chain.getNrOfJoints();

    unsigned int ns = kinova_gen3_arm_chain.getNrOfSegments();

    std::cout << "nj : " << nj << "\tns : " << ns << std::endl;

    const float t_step = 1/((float)loop_rate_val);
    int count = 0;

    kortex_driver::Base_JointSpeeds base_jointspeeds_msg;
    kortex_driver::JointSpeed joint_speed_msg;

    std::vector<float> target_joint_pos;
//    std::vector<float> init_joint_pos = {0.0, 0.26, 3.14, -2.27, 0.0, 0.95, 1.57};
    KDL::JntArray init_joint_pos(Joints);
    init_joint_pos(0) = 0.0;
    init_joint_pos(1) = 0.26;
    init_joint_pos(2) = 3.14;
    init_joint_pos(3) = -2.27;
    init_joint_pos(4) = 0.0;
    init_joint_pos(5) = 0.95;
    init_joint_pos(6) = 1.57;

    for (int i = 0; i < nj; ++i) {
        target_joint_pos.push_back(0.0);
    }

    double roll, pitch, yaw;

    KDL::Frame init_tcp_pos;
    fk_solver.JntToCart(init_joint_pos, init_tcp_pos);

    for (int i = 0; i < 3; ++i) {
        std::cout << i << "th cartesian pos" << init_tcp_pos.p(i) << std::endl;
    }

    init_tcp_pos.M.GetRPY(roll,pitch,yaw);

    std::cout << "roll : " << roll << "\tpitch : " << pitch << "\tyaw : " << yaw << std::endl;

    KDL::Vector target_cartesian_vec(0.3, 0.1, 0.2);
    KDL::Frame target_cartesian_pose(init_tcp_pos.M,target_cartesian_vec);
    KDL::JntArray target_jnt_pos(Joints);
    ik_solver.CartToJnt(init_joint_pos, target_cartesian_pose, target_jnt_pos);



    while (ros::ok()) {


//        std::cout << "start" << std::endl;
//        for (int i = 0; i < nj; ++i) {
//            std::cout << i << "th joint : " << jnt_pos_start(i) << std::endl;
//
//        }
//        std::cout << "end" << std::endl;

        KDL::Frame tcp_pos_start;
        fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

//        for (int i = 0; i < 3; ++i) {
//            std::cout << i << "th : " << tcp_pos_start.p(i) << std::endl;
//        }

        KDL::Vector vec_tcp_pos_goal(0.3, 0.1, 0.2);
//        tcp_pos_start.M.RPY(90.0,0,90.0);

        KDL::Frame tcp_pos_goal(KDL::Rotation::RPY(roll, pitch, yaw), vec_tcp_pos_goal);


        KDL::JntArray jnt_pos_goal(Joints);
        ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal,jnt_pos_goal);



//        for (int i = 0; i < 9; ++i) {
//            std::cout << tcp_pos_start.M.data[i] << std::endl;
//        }


//        std::cout << "start" << std::endl;
//        for (int i = 0; i < Joints; ++i) {
////            std::cout << jnt_pos_goal(i) << std::endl;
//            target_joint_pos[i] = jnt_pos_goal(i);
//        }
//        std::cout << "end" << std::endl;

        for (int i = 0; i < Joints; ++i) {
            target_joint_pos[i] = target_jnt_pos(i);
        }


        base_jointspeeds_msg.joint_speeds.clear();
        for (int i = 0; i < 7; ++i) {
            joint_speed_msg.joint_identifier = i;
            joint_speed_msg.duration = 0;
//            joint_speed_msg.value = 0.0f;
            joint_speed_msg.value = target_joint_pos[i] - jnt_pos_start(i);
//            joint_speed_msg.value = - jnt_pos_start(i);
//            joint_speed_msg.value = init_joint_pos(i) - jnt_pos_start(i);
            base_jointspeeds_msg.joint_speeds.push_back(joint_speed_msg);
        }



        pub_joint_vel.publish(base_jointspeeds_msg);


        ros::spinOnce();
        loop_rate.sleep();


    }


    return 0;

}
