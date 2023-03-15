#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from kortex_driver.msg import TwistCommand
from mediapipe_ros.msg import Hand

import cv2
import mediapipe as mp

class Publishers(object):
    def __init__(self):
        # self.publisher = rospy.Publisher('/control_input', TwistCommand ,Twist,queue_size=10)
        self.publisher = rospy.Publisher('/arm_gen3/in/cartesian_velocity', TwistCommand ,Twist,queue_size=1)

        # {reference_frame: 0, twist: {linear_x: 0.0, linear_y: 0.0, linear_z: 0.01, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}, duration: 0}
        #

        # self.robot_name = rospy.get_param('/arm_gen3/arm_gen3_driver/robot_name')
        # print(self.robot_name)

        self.hand_message = Hand()

        self.input_message = TwistCommand()
        self.input_message.reference_frame = 0


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
        print(hand_msg.right_gesture)

        if hand_msg.right_gesture == "grab":

            self.input_message.twist.linear_z = -0.05
        elif hand_msg.right_gesture == "release":
            self.input_message.twist.linear_z = 0.05
        

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
