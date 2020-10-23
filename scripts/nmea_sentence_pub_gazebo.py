#!/usr/bin/env python
# coding:utf-8

# 通过两个rtk得到航向角

import os
import csv
import math
import tf
import rospy
import time
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path, Odometry
from nmea_msgs.msg import Sentence


class GetNmeaSentence(object):

    def __init__(self):
        rospy.init_node('GetFakeNmeaSentence')
        self.odom = None
        self.yaw = None
        self.ori_yaw = None
        self.pub_tag = True
        self.rate = rospy.Rate(1)
        rospy.Subscriber('/wheel_odom', Odometry, self.odom_cb, queue_size=1)
        self.pub_path = rospy.Publisher('/navsat/nmea_sentence', Sentence, queue_size=1, latch=True)
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def cal_yaw(self, quat):
        quanternion = (quat.x, quat.y, quat.z, quat.w)
        eular = tf.transformations.euler_from_quaternion(quanternion)
        eular = list(eular)
        return eular[2]

    def odom_cb(self, data):
        self.ori_yaw = data.pose.pose.orientation
        self.yaw = (-self.cal_yaw(self.ori_yaw)) * 180 / math.pi
        self.pub_sentence(self.yaw)
        self.rate.sleep()

    def pub_sentence(self, yaww):
        sen = Sentence()
        sen.header.frame_id = 'utm'
        sen.header.stamp = rospy.Time(0)
        sen.sentence = str(self.yaw)
        self.pub_path.publish(sen)

    def shutdown(self):
        time.sleep(0.3)
        print('\033[32m   node shutdown done !   \033[0m')


if __name__ == '__main__':
    try:
        GetNmeaSentence()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start GetNmeaSentence node.')
