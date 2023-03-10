#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import os
import sys
import time

import numpy as np

import rospy
import rospkg

from sensor_msgs.msg import Image

# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    from cv_bridge import CvBridge
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(ws_python3_paths[0])
    else:
        from cv_bridge import CvBridge


class ImageSaver:
    def __init__(self, class_id):
        self.class_id = class_id
        self.save_dir = os.path.join('data', str(class_id))
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        filenums = [int(os.path.splitext(filename)[0])  for filename in os.listdir(self.save_dir)]
        self.img_id = max(filenums) + 1 if os.path.exists(self.save_dir) and (len(filenums) != 0) else 1

        self.img = None
        self.save_image_flag = False

        rospy.init_node('image_saver')
        # self.sub = rospy.Subscriber('/docker/detic_segmentor/instance_segmented_image', Image, self.image_callback)
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.rate = rospy.Rate(10) # set the rate to 10 Hz

    def save_image(self, img, class_id, img_id):
        self.save_path = os.path.join(self.save_dir, f'{self.img_id}.jpg')
        cv2.imwrite(self.save_path, img)
        print(f'Saved image {self.save_path}')

    def image_callback(self, msg):
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.save_image_flag:
            self.save_image(self.img, self.class_id, self.img_id)

            self.img_id += 1
            self.save_image_flag = False

    def run(self):
        while not rospy.is_shutdown():
            if self.img is not None:
                cv2.imshow('image', self.img)
                key = cv2.waitKey(1)
                if key == ord('s'):
                    self.save_image_flag = True
                if key == ord('q'):
                    return
            self.rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', type=int, help='class ID for the images', required=True)
# label to str, int yaml
    args = parser.parse_args()

    saver = ImageSaver(args.label)
    saver.run()
