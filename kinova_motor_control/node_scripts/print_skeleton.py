#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from kortex_driver.msg import *
from kortex_driver.srv import *
# from mediapipe_ros.msg import Hand
from kinova_imitation_common_msgs.msg import HumanSkeletonArray
import time

import cv2
# import mediapipe as mp

class Publishers(object):
    def __init__(self):
        # self.publisher = rospy.Publisher('/control_input', TwistCommand ,Twist,queue_size=10)
        # self.publisher = rospy.Publisher('/arm_gen3/in/cartesian_velocity', TwistCommand ,Twist,queue_size=1)
        #
        rospy.loginfo('debug')



    # def make_msg(self, hand_msg):


    #     if hand_msg.right_gesture == "grab":
    #         self.example_send_gripper_command(1.0)
    #         # self.input_message.twist.linear_z = -0.05
    #     elif hand_msg.right_gesture == "release":
    #         self.example_send_gripper_command(0.0)
    #         # self.input_message.twist.linear_z = 0.05


    # def send_msg(self):
    #     self.publisher.publish(self.input_message)



class Subscribers(object):
    def __init__(self, pub):
        self.pub = pub
        print('debug')
        # self.subscriber = rospy.Subscriber('/skeleton_with_depth/output/pose', HumanSkeletonArray, self.callback, queue_size=10)
        #
        self.subscriber = rospy.Subscriber('/xtion/rgb/image_rect_color', Image, self.callback, queue_size=1)
        print('sdfdebug')

    def callback(self, message):
        # print("right hand xy", message.right_x, message.right_y)
        # print(message)

        print('callback')
        # self.pub.make_msg(message)
        # self.pub.send_msg()

if __name__=="__main__":
    rospy.init_node('test_node')
    # rospy.Rate(100)

    # pub = Publishers()
    pub = 1
    sub = Subscribers(pub)

    rospy.spin()
