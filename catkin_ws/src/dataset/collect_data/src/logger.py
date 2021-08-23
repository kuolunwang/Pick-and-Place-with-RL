#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

class Collect(object):
    def __init__(self, number):

        self.cv_bridge = CvBridge()
        r = rospkg.RosPack()
        self.path = os.path.join(r.get_path('collect_data'), "log")

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        # ros service
        start = rospy.Service("/start_collect", Trigger, self.start)
        stop = rospy.Service("/stop_collect", Trigger, self.stop)

        # ros subscriber
        img_rgb = message_filters.Subscriber('/camera/color/image_raw', Image)
        img_depth = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([img_rgb, img_depth], 5, 5)
        self.ts.registerCallback(self.register)

        traj = rospy.Subscriber('/joint_states', JointState, self.traj_callback)
        grasped = rospy.Subscriber('/gripper/state', Int8, self.grasped_callback)

        # parameter
        self.number = number

        # save data
        if self.trigger:
            self.save(self.number)

    def start(self, req):

        res = TriggerResponse()

        try:
            res.success = True
            self.trigger = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def stop(self, req):

        res = TriggerResponse()

        try:
            res.success = True
            self.trigger = False
            self.number += 1
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def save(self, number):

        rospy.loginfo('Start collect data!')

        log_path = os.path.join(self.path, "log_{:.03}".format(number))
        img_path = os.path.join(log_path, "img")
        dep_path = os.path.join(log_path, "dep")

        if not os.path.exists(log_path):
            os.makedirs(log_path)

        if not os.path.exists(img_path):
            os.makedirs(img_path)

        if not os.path.exists(dep_path):
            os.makedirs(dep_path)

        self.writer_csv(log_path, "trajectory_info", self.traj_info)
        self.writer_csv(log_path, "grasped_info", self.grasped_info)
        
        depth = np.array(self.depth) / 1000.0

        ti = time.time()

        img_name = os.path.join(img_path, ti + "_img")
        depth_name = os.path.join(dep_path, ti + "_dep")
        
        cv2.imwrite(img_name, self.rgb)
        np.save(depth_name, depth)

    def writer_csv(self, path, file_name, data):

        with open(os.path.join(path, file_name + '.csv'), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([data])

    def grasped_callback(self, msg):

        self.grasped_info = msg

    def traj_callback(self, msg):

        self.traj_info = msg.position[0:5]

    def register(self, rgb, depth):

        self.rgb = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
        self.depth = self.cv_bridge.imgmsg_to_cv2(depth, "16UC1")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Set up some initial parameter')

    parser.add_argument('--number', type=int, default = 0)
    args = parser.parse_args()

    rospy.init_node("collect_data_node")
    collecter = Collect(args.number)
    rospy.spin()