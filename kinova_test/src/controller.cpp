#include <atomic>
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/Base_JointSpeeds.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/JointAngle.h>
#include <kortex_driver/JointAngles.h>
#include <kortex_driver/ReadAction.h>

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

#define HOME_ACTION_IDENTIFIER 2
#define PORT 10000
#define IP "192.168.1.10"
#define USERNAME "admin"
#define PASSWORD "admin"

std::atomic<int> last_action_notification_event{0};

geometry_msgs::PointStamped wrist_point;
sensor_msgs::JointState joint_state;

void callback_wirst_point(const geometry_msgs::PointStamped &point_msg) {
  wrist_point = point_msg;
}

void callback_joint_state(const sensor_msgs::JointState &joint_state_msg) {
  joint_state = joint_state_msg;
}

void notification_callback(const kortex_driver::ActionNotification &notif) {
  last_action_notification_event = notif.action_event;
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

bool clear_faults(ros::NodeHandle nh, const std::string &robot_name) {
  ros::ServiceClient service_client_clear_faults =
      nh.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name +
                                                        "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults)) {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool reset_pose(ros::NodeHandle nh, const std::string &robot_name) {
  ros::ServiceClient service_client_read_action =
      nh.serviceClient<kortex_driver::ReadAction>("/" + robot_name +
                                                  "/base/read_action");
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
      nh.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name +
                                                     "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;

  if (service_client_execute_action.call(service_execute_action)) {
    ROS_INFO("The Home position action was sent to the robot.");
  } else {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

void printException(Kinova::Api::KDetailedException &ex) {
  // You can print the error informations and error codes
  auto error_info = ex.getErrorInfo().getError();
  std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

  std::cout << "KError error_code: " << error_info.error_code() << std::endl;
  std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
  std::cout << "KError sub_string: " << error_info.error_sub_string()
            << std::endl;

  // Error codes by themselves are not very verbose if you don't see their
  // corresponding enum value You can use google::protobuf helpers to get the
  // string enum element for every error code and sub-code
  std::cout << "Error code string equivalent: "
            << Kinova::Api::ErrorCodes_Name(
                   Kinova::Api::ErrorCodes(error_info.error_code()))
            << std::endl;
  std::cout << "Error sub-code string equivalent: "
            << Kinova::Api::SubErrorCodes_Name(
                   Kinova::Api::SubErrorCodes(error_info.error_sub_code()))
            << std::endl;
}

bool forward_kinematics(Kinova::Api::Base::BaseClient *base) {
  // Current arm's joint angles
  Kinova::Api::Base::JointAngles input_joint_angles;
  try {
    std::cout << "Getting Angles for every joint..." << std::endl;
    input_joint_angles = base->GetMeasuredJointAngles();
  } catch (Kinova::Api::KDetailedException &ex) {
    std::cout << "Unable to get joint angles" << std::endl;
    printException(ex);
    return false;
  }

  std::cout << "Joint ID : Joint Angle" << std::endl;
  for (auto joint_angle : input_joint_angles.joint_angles()) {
    std::cout << joint_angle.joint_identifier() << " : " << joint_angle.value()
              << std::endl;
  }
  std::cout << std::endl;

  // Computing Foward Kinematics (Angle -> cartesian convert) from arm's current
  // joint angles
  Kinova::Api::Base::Pose pose;
  try {
    std::cout << "Computing Foward Kinematics using joint angles..."
              << std::endl;
    pose = base->ComputeForwardKinematics(input_joint_angles);
  } catch (Kinova::Api::KDetailedException &ex) {
    std::cout << "Unable to compute forward kinematics" << std::endl;
    printException(ex);
    return false;
  }

  std::cout << "Pose calculated : " << std::endl;
  std::cout << "Coordinate (x, y, z)  : (" << pose.x() << ", " << pose.y()
            << ", " << pose.z() << ")" << std::endl;
  std::cout << "Theta (theta_x, theta_y, theta_z)  : (" << pose.theta_x()
            << ", " << pose.theta_y() << ", " << pose.theta_z() << ")"
            << std::endl
            << std::endl;

  return true;
}

bool inverse_kinematics(Kinova::Api::Base::BaseClient *base) {
  // get robot's pose (by using forward kinematics)
  Kinova::Api::Base::JointAngles input_joint_angles;
  Kinova::Api::Base::Pose pose;
  try {
    input_joint_angles = base->GetMeasuredJointAngles();
    pose = base->ComputeForwardKinematics(input_joint_angles);
  } catch (Kinova::Api::KDetailedException &ex) {
    std::cout << "Unable to get current robot pose" << std::endl;
    printException(ex);
    return false;
  }

  // Object containing cartesian coordinates and Angle Guess
  Kinova::Api::Base::IKData input_IkData;

  // Fill the IKData Object with the cartesian coordinates that need to be
  // converted
  input_IkData.mutable_cartesian_pose()->set_x(pose.x());
  input_IkData.mutable_cartesian_pose()->set_y(pose.y());
  input_IkData.mutable_cartesian_pose()->set_z(pose.z());
  input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
  input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
  input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());

  // Fill the IKData Object with the guessed joint angles
  Kinova::Api::Base::JointAngle *jAngle;
  for (auto joint_angle : input_joint_angles.joint_angles()) {
    jAngle = input_IkData.mutable_guess()->add_joint_angles();
    // '- 1' to generate an actual "guess" for current joint angles
    jAngle->set_value(joint_angle.value() - 1);
  }

  // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's
  // current pose and joint angles guess
  Kinova::Api::Base::JointAngles computed_joint_angles;
  try {
    std::cout << "Computing Inverse Kinematics using joint angles and pose..."
              << std::endl;
    computed_joint_angles = base->ComputeInverseKinematics(input_IkData);
  } catch (Kinova::Api::KDetailedException &ex) {
    std::cout << "Unable to compute inverse kinematics" << std::endl;
    printException(ex);
    return false;
  }

  std::cout << "Joint ID : Joint Angle" << std::endl;
  int joint_identifier = 0;
  for (auto joint_angle : computed_joint_angles.joint_angles()) {
    std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;
    joint_identifier++;
  }

  return true;
}

int main(int argc, char **argv) {
  float hz = 40.0f;
  std::string robot_name = "arm_gen3";

  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("hz", hz);
  pnh.getParam("robot_name", robot_name);

  //    ros::Publisher output_pub =
  //    nh.advertise<kortex_driver::Base_JointSpeeds>("joint_vel", 1);
  ros::Publisher output_pub =
      nh.advertise<sensor_msgs::JointState>("test_output_joint", 1);
  //    ros::Subscriber sub_wrist_point =
  //    nh.subscribe("/wrist_point/output/wrist_point", 10,
  //    callback_wirst_point);
  ros::Subscriber sub_joint_state =
      nh.subscribe("/" + robot_name + "/joint_states", 1, callback_joint_state);

  bool success = true;
//  success = reset_pose(nh, robot_name);

  std::cout << "is this work ?" << std::endl;

  //---------------------------------------------------------------
  auto error_callback = [](Kinova::Api::KError err) {
    cout << "_________ callback error _________" << err.toString();
  };
  auto transport = new Kinova::Api::TransportClientTcp();
  auto router = new Kinova::Api::RouterClient(transport, error_callback);
  transport->connect(IP, PORT);

  // Set session data connection information
  auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
  create_session_info.set_username(USERNAME);
  create_session_info.set_password(PASSWORD);
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  std::cout << "Creating session for communication" << std::endl;
  auto session_manager = new Kinova::Api::SessionManager(router);
  session_manager->CreateSession(create_session_info);
  std::cout << "Session created" << std::endl;

  // Create BaseClient
  auto base = new Kinova::Api::Base::BaseClient(router);

  // example core
  success &= forward_kinematics(base);
  success &= inverse_kinematics(base);

  //------------------------------------------------

  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    //        geometry_msgs::PointStamped point_msg;
    //        kortex_driver::Base_JointSpeeds joint_speed_msg;
    sensor_msgs::JointState output_joint_state_msg;

    output_joint_state_msg = joint_state;

    //        point_msg.point.x = input1_last.point.x + input2_last.point.x;
    output_pub.publish(output_joint_state_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // ----------------------------------------
  session_manager->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router->SetActivationStatus(false);
  transport->disconnect();

  // Destroy the API
  delete base;
  delete session_manager;
  delete router;
  delete transport;

  return success ? 0 : 1;
}
