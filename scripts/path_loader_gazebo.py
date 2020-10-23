#!/usr/bin/env python
#coding:utf-8

import os
import csv
import math
import tf
import rospy
import time
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path,Odometry
from nmea_msgs.msg import Sentence


##  ------------- config -----------------

utm_topic = '/gps_meas'							#utm话题 pkg="gps_common" type="utm_odometry_node"
odom_topic = '/wheel_odom'						#odom话题
angle_topic = '/navsat/nmea_sentence'			#方位角信息
path_frame_id = 'utm' 							#rviz中frame
way_file_name =	'waypoints.csv'					#打开的路径文件位置
base_path_topic = '/base_path'					#发布的路径话题
origin_point_topic = '/origin_point'			#发布的原点话题（rviz中不可见）
## ---------------------------------------


## unchangeable
CSV_HEADER = ['x', 'y', 'yaw']
MAX_DECEL = 1.0
pose_init_state = False
yaw_init_state = False
origin_point=[0,0,0]							# [x,y,yaw]  -节点启动时utm绝对坐标 x,y 以及小车的初始角度（需要在下一个updater节点将方位角归0 作为odom的0角度,此方位角并非实际真值）

## 

class WaypointLoader(object):

	def __init__(self):
		rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)
		if rospy.has_param('~gnss_using_file_path'): 
			self.way_file_path = rospy.get_param('~gnss_using_file_path')
		else:
			self.way_file_path=("")
		rospy.Subscriber(utm_topic, Odometry, self.robot_init_pose, queue_size=1)
		self.pub_path = rospy.Publisher(base_path_topic, Path, queue_size=1, latch=True)
		self.pub_origin = rospy.Publisher(origin_point_topic, PoseStamped, queue_size=1, latch=True)
		rospy.spin()


	def yaw_cb(self,data):
		global yaw_init_state
		if(not yaw_init_state):
			global origin_point
			str = data.sentence
			yaw_new = float(str)
			yaw_new = yaw_new*(math.pi)/180
			origin_point[2] = yaw_new
			#print("yaw time is %.2f  !  ----:"%time.time())
			yaw_init_state = True

	def robot_init_pose(self,data):
		global pose_init_state
		global origin_point
		if (not pose_init_state):
			rospy.Subscriber(angle_topic, Sentence, self.yaw_cb, queue_size=1)
			self.init_data = data
			if(math.isnan(self.init_data.pose.pose.position.x) or math.isnan(self.init_data.pose.pose.position.y)):
				rospy.loginfo("GNSS signal not available at current frame,retrying...")
			else:
				if(yaw_init_state):
					origin_point[0] = self.init_data.pose.pose.position.x
					origin_point[1] = self.init_data.pose.pose.position.y
					pose_init_state = True
					#print("xy time is %.2f  !  ----:"%time.time())
					print("Origin point init completely !  :[%.2f,%.2f,%.2f]"%(origin_point[0], origin_point[1], origin_point[2]))
					os_path = os.path.join(self.way_file_path,'path',way_file_name)
					self.new_waypoint_loader(os_path)
			

	def new_waypoint_loader(self, path):
		global pose_init_state
		global origin_point
		if os.path.isfile(path):
			base_path = self.load_waypoints(path)
			rospy.loginfo('Path point transform completely')
			print("origin_point topic publishing        name: %s"%origin_point_topic) # rviz中不可见  该原点值为节点启动是gps转换的莫卡托坐标系的值
			print("base_path    topic publishing        name: %s"%base_path_topic)    # 每个路径点减去原点的值所求出相对utm坐标 显示在 rviz中
			print("")
			print("")
			print("                                    ---  node working ---")
			print("")
			while(True):
				global origin_point
				rate = rospy.Rate(1)
				ori = PoseStamped()
				ori.pose.position.x = origin_point[0]
				ori.pose.position.y = origin_point[1]
				list_angle = tf.transformations.quaternion_from_euler(0,0,origin_point[2])
				ori.pose.orientation.x =list_angle[0]
				ori.pose.orientation.y =list_angle[1]
				ori.pose.orientation.z =list_angle[2]
				ori.pose.orientation.w =list_angle[3]
				self.publish(base_path)
				self.ori_publish(ori)
				rate.sleep()

		else:
			rospy.logerr('%s is not a file,please check waypoints file', path)

	def load_waypoints(self, fname):
		base_path = Path()
		base_path.header.frame_id = path_frame_id  #rviz   frame
		global origin_point
		with open(fname) as wfile:
			reader = csv.DictReader(wfile, CSV_HEADER)
			for wp in reader:
				path_element = PoseStamped()
				path_element.pose.position.x = float(wp['x']) - origin_point[0]
				path_element.pose.position.y = float(wp['y']) - origin_point[1]
				base_path.poses.append(path_element)
		print("Waypoint   load   completely ! ")
		return base_path

	def publish(self,base_path):
		self.pub_path.publish(base_path)

	def ori_publish(self,ori):
		self.pub_origin.publish(ori)


if __name__ == '__main__':
	try:
		WaypointLoader()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint node.')
