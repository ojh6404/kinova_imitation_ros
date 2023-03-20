#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import division
from __future__ import print_function

import numpy as np
import rospy

from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import PointStamped


class WristPoint(ConnectionBasedTransport):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.wrist_point_pub = self.advertise(
            "~output/wrist_point", PointStamped, queue_size=10
        )
        self.wrist_point_msg = PointStamped()

        self.point_offset = [0.0, 0.0, 0.5]

        self.point_queue_size = 30  # 30 Hz
        self.wrist_point = np.zeros(3, dtype=np.float32)
        self.wrist_point_queue = np.zeros((3, self.point_queue_size), dtype=np.float32)

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
            skeleton = skeleton_array_msg.skeletons[
                0
            ]  # TODO : what if multiple people?

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

                # zxy to xyz with offset
                self.wrist_point[0] = self.point_offset[0] - (
                    right_elbow_end_z - right_shoulder_start_z
                )
                self.wrist_point[1] = self.point_offset[1] - (
                    right_elbow_end_x - right_shoulder_start_x
                )
                self.wrist_point[2] = self.point_offset[2] - (
                    right_elbow_end_y - right_shoulder_start_y
                )

                # IQR outlier remove
                # for i in range(3):
                # q25, q75 = np.quantile(self.wrist_point[i], 0.25), np.quantile(
                #     self.wrist_point[i], 0.75
                # )
                # iqr = q75 - q25
                # cutoff = iqr * 1.5
                # lower, upper = q25 -cut_off, q75 + cut_off
                # if self.wrist_point[i] < lower or self.wrist_point[i] > upper:
                #     return

                # detect outlier
                outlier = np.abs(self.wrist_point - self.wrist_point_queue[:, -1]) > 0.3
                if np.all(outlier):
                    print(
                        "outlier",
                        outlier,
                        "\t delta",
                        self.wrist_point - self.wrist_point_queue[:, -1],
                    )
                else:
                    # fill wrist point queue queue
                    self.wrist_point_queue[:, :-1] = self.wrist_point_queue[:, 1:]
                    self.wrist_point_queue[:, -1] = self.wrist_point

                    # prepare message
                    self.wrist_point_msg = PointStamped(
                        header=skeleton_array_msg.header
                    )
                    # raw data (no moving average)
                    # self.wrist_point_msg.point.x = self.wrist_point[0]
                    # self.wrist_point_msg.point.y = self.wrist_point[1]
                    # self.wrist_point_msg.point.z = self.wrist_point[2]

                    # processed data (moving average)
                    self.wrist_point_msg.point.x = np.mean(self.wrist_point_queue[0])
                    self.wrist_point_msg.point.y = np.mean(self.wrist_point_queue[1])
                    self.wrist_point_msg.point.z = np.mean(self.wrist_point_queue[2])

            else:
                # print("no bone detected")
                pass

            self.wrist_point_pub.publish(self.wrist_point_msg)
        except:
            print("no human detected")


if __name__ == "__main__":
    rospy.init_node("wrist_pose_node")
    WristPoint()
    rospy.spin()
