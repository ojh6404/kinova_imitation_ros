#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
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


const int Joints = 7;
KDL::JntArray jnt_pos_start(Joints);
const int loop_rate_val = 100;

geometry_msgs::PointStamped wrist_point_global;
sensor_msgs::JointState joint_state_global;

//void cb_wrist_point(const geometry_msgs::PointStamped::ConstPtr & wrist_point) {
//
//    wrist_point_global = wrist_point->point[0];
//}

void cb_joint_state(const sensor_msgs::JointState::ConstPtr & joint_state) {
    for (int i = 0; i < Joints; ++i) {
        jnt_pos_start(i) = joint_state->position[i];
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

    //Create publishers to send position commands to all joints
//    ros::Publisher pub_joint_vel = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
    ros::Publisher pub_test_joint_states = n.advertise<sensor_msgs::JointState>("/test/output", 1);
    sensor_msgs::JointState test_joint_state;

    //Create publishers to send position commands to all joints
    ros::Publisher joint_com_pub[7];
    joint_com_pub[0] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_1_position_controller/command", 1000);
    joint_com_pub[1] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_2_position_controller/command", 1000);
    joint_com_pub[2] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_3_position_controller/command", 1000);
    joint_com_pub[3] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_4_position_controller/command", 1000);
    joint_com_pub[4] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_5_position_controller/command", 1000);
    joint_com_pub[5] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_6_position_controller/command", 1000);
    joint_com_pub[6] = n.advertise<std_msgs::Float64>("/arm_gen3/joint_7_position_controller/command", 1000);




    //Parse urdf model and generate KDL tree
    KDL::Tree kinova_gen3_arm_tree;
    if (!kdl_parser::treeFromFile(urdf_path, kinova_gen3_arm_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    //Generate a kinematic chain from the robot base to its tcp
    KDL::Chain kinova_gen3_arm_chain;
    kinova_gen3_arm_tree.getChain("base_link", "end_effector_link", kinova_gen3_arm_chain);

    //Create solvers
    KDL::ChainFkSolverPos_recursive fk_solver(kinova_gen3_arm_chain);
    KDL::ChainIkSolverVel_pinv vel_ik_solver(kinova_gen3_arm_chain, 0.0001, 1000);
    KDL::ChainIkSolverPos_NR ik_solver(kinova_gen3_arm_chain, fk_solver, vel_ik_solver, 1000);


    const float t_step = 1/((float)loop_rate_val);
    int count = 0;


    while (ros::ok()) {
        KDL::Frame tcp_pos_start;
        fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

        ROS_INFO("Current tcp Position/Twist KDL:");
        ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));
        ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

        float t_max;
        KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
        get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

        KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

        KDL::JntArray jnt_pos_goal(Joints);
        ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

        float t = 0.0;

        for (int i = 0; i < Joints; ++i) {
            std::cout << jnt_pos_goal(i) << std::endl;
        }

        while(t<t_max) {
            std_msgs::Float64 position[7];
            //Compute next position step for all joints
            for(int i=0; i<Joints; i++) {
                position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
                joint_com_pub[i].publish(position[i]);
            }

            ros::spinOnce();
            loop_rate.sleep();
            ++count;
            t += t_step;
        }
//
//        ros::spinOnce();
//        loop_rate.sleep();


    }


    return 0;

}
