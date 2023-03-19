#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import division
from __future__ import print_function

import cv_bridge
from image_geometry import PinholeCameraModel
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


class WristPoint(ConnectionBasedTransport):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.wrist_point_pub = self.advertise(
            "~output/wrist_point", PointStamped, queue_size=10
        )
        self.wrist_point_msg = PointStamped()

        self.point_offset = [0.0, 0.0, 0.5]

        self.point_queue_size = 10  # 30 Hz
        self.wrist_point_array = np.array(
            [0.0] * self.point_queue_size, dtype=np.float32
        )

    def subscribe(self):
        queue_size = rospy.get_param("~queue_size", 10)
        sub_skeleton = rospy.Subscriber(
            "/skeleton_with_depth/output/pose",
            HumanSkeletonArray,
            self.callback,
            queue_size=queue_size,
            buff_size=2**24,
        )
        self.sub = sub_skeleton

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, skeleton_array_msg: HumanSkeletonArray):
        try:
            skeleton = skeleton_array_msg.skeletons[0]

            bones = skeleton.bones
            bone_names = skeleton.bone_names

            right_arm_bones = ["right_shoulder->right_elbow", "right_elbow->left_wrist"]
            if set(right_arm_bones) < set(bone_names):
                # print("detected")
                right_arm_bones_index = [
                    bone_names.index(right_arm_bones[0]),
                    bone_names.index(right_arm_bones[1]),
                ]
                right_shoulder_start_x = bones[right_arm_bones_index[0]].start_point.x
                right_shoulder_start_y = bones[right_arm_bones_index[0]].start_point.y
                right_shoulder_start_z = bones[right_arm_bones_index[0]].start_point.z
                # right_shoulder_end_x = bones[right_arm_bones_index[0]].end_point.x
                # right_shoulder_end_y = bones[right_arm_bones_index[0]].end_point.y
                # right_shoulder_end_z = bones[right_arm_bones_index[0]].end_point.z
                # right_elbow_start_x = bones[right_arm_bones_index[1]].start_point.x
                # right_elbow_start_y = bones[right_arm_bones_index[1]].start_point.y
                # right_elbow_start_z = bones[right_arm_bones_index[1]].start_point.z
                right_elbow_end_x = bones[right_arm_bones_index[1]].end_point.x
                right_elbow_end_y = bones[right_arm_bones_index[1]].end_point.y
                right_elbow_end_z = bones[right_arm_bones_index[1]].end_point.z

                self.wrist_point_msg = PointStamped(header=skeleton_array_msg.header)
                self.wrist_point_msg.point.x = self.point_offset[0] - (
                    right_elbow_end_z - right_shoulder_start_z
                )
                self.wrist_point_msg.point.y = self.point_offset[1] - (
                    right_elbow_end_x - right_shoulder_start_x
                )
                self.wrist_point_msg.point.z = self.point_offset[2] - (
                    right_elbow_end_y - right_shoulder_start_y
                )

            else:
                pass
                # print("no bone detected")

            self.wrist_point_pub.publish(self.wrist_point_msg)
        except:
            print("no human detected")


if __name__ == "__main__":
    rospy.init_node("wrist_pose_node")
    WristPoint()
    rospy.spin()
