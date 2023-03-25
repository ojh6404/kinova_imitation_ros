#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define IP "192.168.1.10"
#define USERNAME "admin"
#define PASSWORD "admin"

float TIME_DURATION = 10.0f; // Duration of the example (seconds)

geometry_msgs::PointStamped wrist_point;
// Get actuator count


// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};


/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
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




// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

/**************************
 * Example core functions *
 **************************/
bool reset_pose(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
                create_event_listener_by_promise(finish_promise),
                k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}


bool compute_inverse_kinematics(Kinova::Api::Base::BaseClient* base)
{
    // get robot's pose (by using forward kinematics)
    Kinova::Api::Base::JointAngles input_joint_angles;
    Kinova::Api::Base::Pose pose;
    try
    {
        input_joint_angles = base->GetMeasuredJointAngles();
        pose = base->ComputeForwardKinematics(input_joint_angles);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        std::cout << "Unable to get current robot pose" << std::endl;
        return false;
    }

    // Object containing cartesian coordinates and Angle Guess
    Kinova::Api::Base::IKData input_IkData;

    // Fill the IKData Object with the cartesian coordinates that need to be converted
    input_IkData.mutable_cartesian_pose()->set_x(pose.x());
    input_IkData.mutable_cartesian_pose()->set_y(pose.y());
    input_IkData.mutable_cartesian_pose()->set_z(pose.z());
    input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
    input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
    input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());

    // Fill the IKData Object with the guessed joint angles
    Kinova::Api::Base::JointAngle* jAngle;
    for(auto joint_angle : input_joint_angles.joint_angles())
    {
        jAngle = input_IkData.mutable_guess()->add_joint_angles();
        // '- 1' to generate an actual "guess" for current joint angles
        jAngle->set_value(joint_angle.value() - 1);
    }

    // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
    Kinova::Api::Base::JointAngles computed_joint_angles;
    try
    {
        std::cout << "Computing Inverse Kinematics using joint angles and pose..." << std::endl;
        computed_joint_angles = base->ComputeInverseKinematics(input_IkData);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        std::cout << "Unable to compute inverse kinematics" << std::endl;
        return false;
    }

    std::cout << "Joint ID : Joint Angle" << std::endl;
    int joint_identifier = 0;
    for (auto joint_angle : computed_joint_angles.joint_angles())
    {
        std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;
        joint_identifier++;
    }

    return true;
}



bool cyclic_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    bool return_status = true;

    unsigned int actuator_count = base->GetActuatorCount().count();


    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }


    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;
    std::array<float, 7> current_joint_pos;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        int first_actuator_device_id = 1;
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        // Initial delta between first and last actuator
        float init_delta_position = base_feedback.actuators(0).position() - base_feedback.actuators(actuator_count - 1).position();

        // Initial first and last actuator torques; avoids unexpected movement due to torque offsets
        float init_last_torque = base_feedback.actuators(actuator_count - 1).torque();
        float init_first_torque = -base_feedback.actuators(0).torque(); //Torque measure is reversed compared to actuator direction
        float torque_amplification = 2.0;

        std::cout << "Running torque control example for " << TIME_DURATION << " seconds" << std::endl;

        Kinova::Api::Base::JointAngles input_joint_angles;
        Kinova::Api::Base::Pose pose;
        Kinova::Api::Base::JointAngle* jAngle;
        Kinova::Api::Base::IKData input_IkData;
//        ros::Rate loop_rate(1000);

        for(auto joint_angle : input_joint_angles.joint_angles()) {
            jAngle = input_IkData.mutable_guess()->add_joint_angles();
            // '- 1' to generate an actual "guess" for current joint angles
            jAngle->set_value(joint_angle.value());
        }


        // Real-time loop
//        while (timer_count < (TIME_DURATION * 1000))
        while (ros::ok())
        {
            now = GetTickUs();

            if (now - last > 1000)
            {

//                std::cout << wrist_point << std::endl;
                std::cout << "time : " << now - last << std::endl;


                for (int i = 0; i < actuator_count; ++i) {
                    current_joint_pos[i] = base_feedback.actuators(i).position();

                }



//                try
//                {
//                    input_joint_angles = base->GetMeasuredJointAngles();
//                    pose = base->ComputeForwardKinematics(input_joint_angles);
//                }
//                catch (Kinova::Api::KDetailedException& ex)
//                {
//                    std::cout << "Unable to get current robot pose" << std::endl;
//                    return false;
//                }
//
//                // Object containing cartesian coordinates and Angle Guess

//
//                // Fill the IKData Object with the cartesian coordinates that need to be converted
//                input_IkData.mutable_cartesian_pose()->set_x(pose.x());
//                input_IkData.mutable_cartesian_pose()->set_y(pose.y());
//                input_IkData.mutable_cartesian_pose()->set_z(pose.z());
//                input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
//                input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
//                input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());
//
//                // Fill the IKData Object with the guessed joint angles
//                Kinova::Api::Base::JointAngle* jAngle;
//                for(auto joint_angle : input_joint_angles.joint_angles())
//                {
//                    jAngle = input_IkData.mutable_guess()->add_joint_angles();
//                    // '- 1' to generate an actual "guess" for current joint angles
//                    jAngle->set_value(joint_angle.value() - 1);
//                }
//
//                // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
//                Kinova::Api::Base::JointAngles computed_joint_angles;
//                try
//                {
//                    std::cout << "Computing Inverse Kinematics using joint angles and pose..." << std::endl;
//                    computed_joint_angles = base->ComputeInverseKinematics(input_IkData);
//                }
//                catch (Kinova::Api::KDetailedException& ex)
//                {
//                    std::cout << "Unable to compute inverse kinematics" << std::endl;
//                    return false;
//                }
//
//                std::cout << "Joint ID : Joint Angle" << std::endl;
//                int joint_identifier = 0;
//                for (auto joint_angle : computed_joint_angles.joint_angles())
//                {
//                    std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;
//                    joint_identifier++;
//                }


                std::cout << "IK computed" << std::endl;




                // Position command to first actuator is set to measured one to avoid following error to trigger
                // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                base_command.mutable_actuators(0)->set_position(base_feedback.actuators(0).position());

                // First actuator torque command is set to last actuator torque measure times an amplification
                base_command.mutable_actuators(0)->set_torque_joint(init_first_torque + (torque_amplification * (base_feedback.actuators(actuator_count - 1).torque() - init_last_torque)));


                // First actuator position is sent as a command to last actuator
                base_command.mutable_actuators(actuator_count - 1)->set_position(base_feedback.actuators(0).position() - init_delta_position);

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < actuator_count; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }

                timer_count++;
                last = GetTickUs();
            }

            ros::spinOnce();
//            loop_rate.sleep();

        }

        std::cout << "Torque control example completed" << std::endl;

        // Set first actuator back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}



void callback_wirst_point(const geometry_msgs::PointStamped& point_msg) {
    wrist_point = point_msg;
}


int main(int argc, char **argv)
{
    // ROS node
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    float hz = 40.0;
    std::string robot_name = "arm_gen3";
    pnh.getParam("hz", hz);
    pnh.getParam("robot_name",robot_name);

//    ros::Publisher output_pub = nh.advertise<kortex_driver::Base_JointSpeeds>("joint_vel", 1);
    ros::Publisher output_pub = nh.advertise<geometry_msgs::PointStamped>("test_output",1);
//    ros::Subscriber sub_wrist_point = nh.subscribe("/wrist_point/output/wrist_point", 10, callback_wirst_point);
    ros::Subscriber sub_wrist_point = nh.subscribe("/wrist_point/output/wrist_point", 1, callback_wirst_point);

//    bool success = true;







    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USERNAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // core
    bool success = true;
    success &= reset_pose(base);
    success &= cyclic_velocity_control(base, base_cyclic, actuator_config);
    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
    }









    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}
