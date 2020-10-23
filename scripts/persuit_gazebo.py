#!/usr/bin/env python
# coding:utf-8
import os
import rospy
import math,time
import numpy
import tf

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
## -----------------------  congig  --------------------------------------------------
# 订阅的话题
base_path_topic = '/base_path'								
utm_robot_pose = '/utm_pose'							#utm坐标系下实时位姿 

# 发布的话题
final_path = '/final_path'
cmd_vel = '/cmd_vel'
# 参数
ARR_LEN = 0.1    # 离下一个目标点距离小于多少认为已经到达
MIN_POI_NUM = 5
MAX_POI_NUM = 20
LOOK_ANGLE = 0.07  
vk = 5 


## ---unchangeable----------------------------------------------------------------------


STOP_CMD =False



## -------------------------------------------------------------------------------------
##class
class Ruban_Persuit(object):

	def __init__(self):
		rospy.init_node('Ruban_Persuit', log_level=rospy.INFO)
		self.waypoints_2d = None
		self.next_index = 0
		self.curr_pose = None
		self.path_array_size = None
		self.new_poi_arr = None
		self.is_all_arrived = False
		self.STOP_CMD =False

		rospy.Subscriber(base_path_topic, Path, self.got_path_cb,queue_size=1)
		rospy.Subscriber(utm_robot_pose, PoseStamped, self.robot_pose_cb, queue_size=1)

		self.pub_final_path = rospy.Publisher(final_path, Path, queue_size=1, latch=True)
		self.pub_cmd_vel = rospy.Publisher(cmd_vel, Twist, queue_size=1, latch=True)

		rospy.on_shutdown(self.shutdown) 
		rospy.spin()


	def got_path_cb(self, data):
		if not self.waypoints_2d:
			self.waypoints_2d = [[waypoint.pose.position.x, waypoint.pose.position.y] for waypoint in data.poses]
			self.path_array_size = len(self.waypoints_2d)
			


	def robot_pose_cb(self,data):
		if not self.STOP_CMD:
			if self.waypoints_2d:
				self.curr_pose = data
				tag= self.is_arrived(self.curr_pose,self.next_index)
				if not tag:
					if(self.next_index < self.path_array_size ):
						self.cal_and_pub_cmd(self.curr_pose,self.next_index,self.waypoints_2d)
						#self.cal_and_pub_path(self.curr_pose,self.next_index,self.waypoints_2d) 
						next_poi = [self.waypoints_2d[self.next_index][0],self.waypoints_2d[self.next_index][1]]
				else:
					self.next_index = self.next_index + 1



	def cal_and_pub_path(self,pose,nextindex,waypoints):
		while(nextindex<self.path_array_size-1):
			len_1 = self.target_dist_yaw(pose,waypoints[nextindex])[0]
			if len_1>0.15:
				self.next_index = nextindex
				break


	def cal_and_pub_cmd(self,pose,nextindex,waypoints):
		currentX = pose.pose.position.x
		currentY = pose.pose.position.y
		targetX = waypoints[nextindex][0]
		targetY = waypoints[nextindex][1]
		if nextindex <(self.path_array_size - 1):
			p1 = [currentX,currentY]
			p2 = [targetX,targetY]
			p3 = waypoints[nextindex+1]
			print(p1,p2,"index:",nextindex)
			re = self.cal_ang(p1,p2,p3)
			# v = vk*(1-math.cos(re[2]/2))
			quanternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quanternion)
			yaw = euler[2]
			alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw
			l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
			theta = math.atan( math.sin(alpha))/1
				# #get twist command
			print("alpha : %f"%alpha)
			if math.cos(alpha)<0.707:
				v=0
			else :

				t = math.pi/4-math.acos(math.cos(alpha))
				print(t)
				v = max(t,0.05)*0.8
			twistCmd = Twist()
			twistCmd.linear.x = v
			twistCmd.angular.z = theta*1.5		
		else:
			twistCmd = Twist()
			twistCmd.linear.x = 0
			twistCmd.angular.z = 0
		self.pub_cmd_vel.publish(twistCmd)


	def cal_ang(self,point_1, point_2, point_3):
		a = math.sqrt(
			(point_2[0] - point_3[0]) * (point_2[0] - point_3[0]) + (point_2[1] - point_3[1]) * (point_2[1] - point_3[1]))
		b = math.sqrt(
			(point_1[0] - point_3[0]) * (point_1[0] - point_3[0]) + (point_1[1] - point_3[1]) * (point_1[1] - point_3[1]))
		c = math.sqrt(
			(point_1[0] - point_2[0]) * (point_1[0] - point_2[0]) + (point_1[1] - point_2[1]) * (point_1[1] - point_2[1]))
		B = math.acos((b * b - a * a - c * c) / (-2 * a * c))
		result = [c, a, B]
		return result





	def is_arrived(self,pose,next_index):
		if next_index > self.path_array_size-1:
			return True
		else:
			length = ((pose.pose.position.x-self.waypoints_2d[next_index][0])**2+(pose.pose.position.y-self.waypoints_2d[next_index][1])** 2)**0.5
			# print(self.waypoints_2d[next_index][0],self.waypoints_2d[next_index][1])
			# print(pose.pose.position.x,pose.pose.position.y)
			if length < ARR_LEN:
				# if(next_index < self.path_array_size):
				# 	print("   Arrived at the %d / %d waypoint , going to the next point"%(next_index,self.path_array_size))
				# else:
				# 	print("   All waypoints have arrived  , path following is done ! ")
				return True
			else:
				#print("index: %d is arrived : false  , length is  %f "%(next_index,length))
				return False 


	def cal_yaw(self,quat):
		quanternion = (quat.x, quat.y, quat.z, quat.w)
		eular = tf.transformations.euler_from_quaternion(quanternion)
		eular=list(eular)
		return eular[2]

	def target_dist_yaw(self,pos,point):
		x1 = pos.pose.position.x
		y1 = pos.pose.position.y
		x2 = point[0]
		y2 = point[1]
		if (x1 != x2):
			alpha = math.atan2((y2 - y1), (x2 - x1))
			delta = self.cal_yaw(pos.pose.orientation)
			temp_yaw = delta - alpha
			distence = ((x1-x2)**2+(y1+y2)**2)**0.5
			return [distence,temp_yaw]
		else:
			return [abs(y2 - y1),temp_yaw]





	def shutdown(self):
		time.sleep(0.3)
		self.STOP_CMD =True
		twistCmd = Twist()
		twistCmd.linear.x = 0
		twistCmd.angular.z = 0
		rospy.loginfo("Stopping the robot...")
		#self.twist_pub.publish(Twist())
		self.pub_cmd_vel.publish(twistCmd)
		rospy.loginfo('\033[32mnode shutdown done !   \033[0m')
		rospy.sleep(0.3)





if __name__ == '__main__':
	try:
		Ruban_Persuit()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start ruban persuit node.')
