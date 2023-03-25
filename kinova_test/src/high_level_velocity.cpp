#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <BaseClientRpc.h>

#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <Common.pb.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

float TIME_DURATION = 30.0f; // Duration of the example (seconds)

// Actuator speed (deg/s)
const float SPEED = 20.0f;

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
//        pose = base->ComputeForwardKinematics(input_joint_angles);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        std::cout << "Unable to get current robot pose" << std::endl;
        return false;
    }

    // Object containing cartesian coordinates and Angle Guess
    Kinova::Api::Base::IKData input_IkData;

    pose.set_x(0.6599610447883606);
    pose.set_y(0.007769245654344559);
    pose.set_z(0.4319004416465759);
    pose.set_theta_x(90.0);
    pose.set_theta_y(0.0);
    pose.set_theta_z(90.0);

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




bool send_joint_speeds(k_api::Base::BaseClient* base)
{
    std::cout << "Sending the angular velocities to the robot for 10 seconds..." << std::endl;

    k_api::Base::JointSpeeds joint_speeds;

    std::vector<float> speeds;
    // The 7DOF robot will spin in the same direction for 10 seconds
    int actuator_count = base->GetActuatorCount().count();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
//    for (size_t i = 0; i < speeds.size(); ++i) {
//            auto joint_speed = joint_speeds.add_joint_speeds();
//            joint_speed->set_joint_identifier(i);
//            joint_speed->set_value(0.0);
//            joint_speed->set_duration(1);
//    }
    while (timer_count < (TIME_DURATION * 1000)) {

        now = GetTickUs();
        speeds = {-SPEED, 0.0f, SPEED, 0.0f, -SPEED, 0.0f, SPEED};
        joint_speeds.clear_joint_speeds();
        for (size_t i = 0; i < speeds.size(); ++i) {
            auto joint_speed = joint_speeds.add_joint_speeds();
            joint_speed->set_joint_identifier(i);
            joint_speed->set_value(speeds.at(i));
            joint_speed->set_duration(1);
        }
        base->SendJointSpeedsCommand(joint_speeds);
//        std::this_thread::sleep_for(std::chrono::milliseconds(25)); // 40 hz

        timer_count++;
        last = GetTickUs();

        std::cout << "time : " << last - now << std::endl;
    }

    // Wait 10 seconds
//    std::this_thread::sleep_for(std::chrono::milliseconds(10000));


    // Stop the robot
    std::cout << "Stopping the robot" << std::endl;
    base->Stop();

    return true;
}

int main(int argc, char **argv)
{


    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect("192.168.1.10", PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);

    // Example core
    bool success = true;
    success &= reset_pose(base);
    success &= send_joint_speeds(base);

    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0: 1;
};
