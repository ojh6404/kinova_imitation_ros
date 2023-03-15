#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys

# ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from kortex_driver.msg import *
from kortex_driver.srv import *
from mediapipe_ros.msg import Hand

# kortex api
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

import cv2
import mediapipe as mp

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils import utilities

class Publishers(object):
    def __init__(self):
        # self.publisher = rospy.Publisher('/control_input', TwistCommand ,Twist,queue_size=10)
        self.publisher = rospy.Publisher('/arm_gen3/in/cartesian_velocity', TwistCommand ,Twist,queue_size=1)

        # {reference_frame: 0, twist: {linear_x: 0.0, linear_y: 0.0, linear_z: 0.01, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}, duration: 0}
        #

        # Import the utilities helper module

        # Parse arguments
        args = utilities.parseConnectionArguments()

        self.robot_name = rospy.get_param('/arm_gen3/arm_gen3_driver/robot_name')

        self.hand_message = Hand()

        self.input_message = TwistCommand()
        self.input_message.reference_frame = 0


        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)


    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            # time.sleep(0.5)
            return True


    def make_msg(self, hand_msg):

        # x_pos = hand_msg.right_x
        # y_pos = hand_msg.right_y
        # x_vel = 0.
        # y_vel = 0.

        # x_pos_ref = 1280.
        # y_pos_ref = 720.
        # x_vel_ref = 0. # position control
        # y_vel_ref = 0.

        # Kp = 5.
        # Kd = 0.1

        # x_pos_diff = x_ref - x
        # y_pos_diff = y_ref - y
        # x_vel_diff = x_vel_ref - x_vel
        # y_vel_diff = y_vel_ref - y_vel

        # control_input = Kp * x_diff + Kd *

        # center is 360
        # thre is 120

        if hand_msg.right_gesture == "grab":
            self.example_send_gripper_command(1.0)
            # self.input_message.twist.linear_z = -0.05
        elif hand_msg.right_gesture == "release":
            self.example_send_gripper_command(0.0)
            # self.input_message.twist.linear_z = 0.05


        # if hand_msg.right_y > 480.: # hand is above center
        #     # self.input_message.twist.linear_x = 0.
        #     # self.input_message.twist.linear_y = 0.
        #     self.input_message.twist.linear_z = -0.05
        #     # self.input_message.twist.angular_x = 0.
        #     # self.input_message.twist.angular_y = 0.
        #     # self.input_message.twist.angular_z = 0.
        # elif hand_msg.right_y < 240.:
        #     self.input_message.twist.linear_z = 0.05
        # else:
        #     self.input_message.twist.linear_z = 0.0

    def send_msg(self):
        self.publisher.publish(self.input_message)



class Subscribers(object):
    def __init__(self, pub):
        self.pub = pub
        self.subscriber = rospy.Subscriber('/hand_status',Hand, self.callback,queue_size=1)

    def callback(self, message):
        # print("right hand xy", message.right_x, message.right_y)

        self.pub.make_msg(message)
        self.pub.send_msg()

if __name__=="__main__":
    rospy.init_node('test_node')
    rospy.Rate(100)

    pub = Publishers()
    sub = Subscribers(pub)

    rospy.spin()


#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import time
import sys
import os
import threading

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications
    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_twist_command(base):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0.03
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 5

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    from utils import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)

        # Example core
        success = True
        success &= example_move_to_home_position(base)
        success &= example_twist_command(base)

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
