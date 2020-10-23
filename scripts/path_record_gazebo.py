#!/usr/bin/env python
# coding:utf-8
# 本程序录制gps原始数据/gps转换的来的utm坐标数据/odom里程计原始数据 ，计算路径点以utm为主
import csv
import math
import rospy
import os
import time
import numpy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


#  节点将生成两个路径点文件， waypoints_时间_origin.csv ：保留估计录制原始数据， 第一列序号0代表经纬度信息
#  第一列序号为1的点是经纬度信息转换为大地坐标系的xy坐标，序号2的点是odom的xy。 
#  生成的第二个waypoints_时间.csv只保留原始数据中的大地坐标。
#

##  -----------------  config ------------
utm_publish_frame_id = 'utm'							#会实时将当前utm相对坐标现实在rviz上，此为frame
relative_utm_pose = 'record_utm_pose'					#此为要现实显示utm相对坐标的话题
gps_topic = 'fix' #'/sensor_msgs/NavSatFix'				#gnss话题
utm_topic = '/gps_meas'									#utm话题 pkg="gps_common" type="utm_odometry_node"  
odom_topic = '/wheel_odom' 								#odom话题
data_process_tag = True									#默认保存原始数据前进行数据处理生成坐标点
min_len = 0.2  											#当前点与上一个点距离小于此数值时忽略该点 单位 米
## ---------------------------------------


odom_count = 0
total_num = 0
RECORD_TAG =  True


class GPS_Recorder(object):
	def __init__(self):
		rospy.init_node('GPS_Recorder')
		global RECORD_TAG
		global gps_topic
		global utm_topic
		global odom_topic
		global origin_utm

		self.father_path = rospy.get_param('~gnss_record_file_path')
		self.target_csv_path = None
		self.final_result_path = None
		self.target_csv = None
		self.writer = None
		self.FILE_READY = False
		self.utm_origin = None
		self.gnss_data = None
		self.utm_data = None
		self.file_oper()
		time.sleep(1)
		print(' ')
		print(' ')
		print('\033[32m   GNSS points recording  ...   \033[0m')
		print(' ')
		print(' ')
		print('           -press CTRL-C to stop recording data when record is done')
		print(' ')
		print(' ')
		rospy.Subscriber(gps_topic, NavSatFix, self.gnss_fix,queue_size = 1)
		rospy.Subscriber(utm_topic, Odometry, self.utm_cb, queue_size=1)
		rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=1)
		self.publish_relative_utm = rospy.Publisher(relative_utm_pose, Odometry, queue_size=1, latch=True)
		rospy.on_shutdown(self.shutdown) 
		rospy.spin()

	def file_oper(self):
		filename = time.strftime("waypoints_%m_%d_%H_%M_%S_origin",time.localtime(time.time()))+".csv"
		self.target_csv_path=os.path.join(self.father_path,'path_repository',filename)
		final_filename=time.strftime("waypoints_%m_%d_%H_%M_%S",time.localtime(time.time()))+".csv"
		self.final_result_path=os.path.join(self.father_path,'path_repository',final_filename)
		self.target_csv = open(self.target_csv_path, 'wb')
		self.writer = csv.writer(self.target_csv)
		self.FILE_READY = True



	def gnss_fix(self,data):
		self.gnss_data = data
		if(math.isnan(self.gnss_data.latitude) or math.isnan(self.gnss_data.longitude)):
			rospy.loginfo("GNSS signal not available at current frame,Skip and record the next one  ...")
		else:
			global RECORD_TAG
			if RECORD_TAG and self.FILE_READY:
				gnss_xy = []
				gnss_xy.append(0)        # tag 0 means gps points data
				gnss_xy.append(self.gnss_data.header.stamp)
				gnss_xy.append(self.gnss_data.latitude)
				gnss_xy.append(self.gnss_data.longitude)
				self.writer.writerow(gnss_xy)


	def utm_cb(self, data):
		self.utm_data = data
		global RECORD_TAG
		global first_utm_tag
		if RECORD_TAG and self.FILE_READY:
			if(math.isnan(self.utm_data.pose.pose.position.x) or math.isnan(self.utm_data.pose.pose.position.y)):
				rospy.loginfo("gps_to_utm node not available at current frame,Skip and record the next point")
			else:
				x = 0
				y = 0 
				if(not self.utm_origin):
					x = self.utm_data.pose.pose.position.x
					y = self.utm_data.pose.pose.position.y
					self.utm_origin = [x,y]
				else:
					x=self.utm_data.pose.pose.position.x
					y=self.utm_data.pose.pose.position.y
					x_rviz = x - self.utm_origin[0]
					y_rviz = y - self.utm_origin[1]
					self.publish_utm(x_rviz,y_rviz)
				utm_xy = []
				utm_xy.append(1)      # tag 1 means utm points data
				utm_xy.append(self.utm_data.header.stamp)
				utm_xy.append(x)
				utm_xy.append(y)
				self.writer.writerow(utm_xy)


	def odom_cb(self, data):
		global RECORD_TAG
		if RECORD_TAG and self.FILE_READY:
			self.odom_data = data
			odom_xy = []
			odom_xy.append(2)      # tag 2 means odom points data
			odom_xy.append(self.odom_data.header.stamp)
			odom_xy.append(self.odom_data.pose.pose.position.x)
			odom_xy.append(self.odom_data.pose.pose.position.y)
			self.writer.writerow(odom_xy)

	def publish_utm(self,x,y):
		utm_odometry = Odometry()
		utm_odometry.header.frame_id = utm_publish_frame_id
		utm_odometry.header.stamp = rospy.Time(0)
		utm_odometry.pose.pose.position.x = x
		utm_odometry.pose.pose.position.y = y
		utm_odometry.pose.pose.position.z = 0
		utm_odometry.pose.pose.orientation.x = 0
		utm_odometry.pose.pose.orientation.y = 0.707
		utm_odometry.pose.pose.orientation.z = 0
		utm_odometry.pose.pose.orientation.w = 0.707
		self.publish_relative_utm.publish(utm_odometry)



	def shutdown(self):
		global RECORD_TAG
		RECORD_TAG = False
		print('\033[32m Stopping the record node, please dont close this Terminal until save process is done  \033[0m')
		time.sleep(0.3)
		self.target_csv.close()
		print(' ')
		print('   data_processing')
		global data_process_tag
		if(data_process_tag):
			self.data_processing()
		
		

	def data_processing(self):
		num=0
		self.target_csv.close()
		array_ori = numpy.genfromtxt(open(self.target_csv_path,"rb"),delimiter=",")
		result_list = []
		utm_index_arr = []
		array_ori_len=len(array_ori)
		if(array_ori_len>0):
			# build utm array
			for index, item in enumerate(array_ori):
				if (item[0] == 1):
					utm_index_arr.append(index)
			# drop point which length is too small ,min_length should change in line-21  $ changeable  config $
			len_utm = len(utm_index_arr)
			if(len_utm>0):
				point1 = [array_ori[utm_index_arr[0]][2], array_ori[utm_index_arr[0]][3], 0]
				result_list.append(point1)
				num = num + 1
				for item in utm_index_arr:
					point2 = [array_ori[item][2], array_ori[item][3], 0]
					length = ((point2[1] - point1[1]) ** 2 + (point2[0] - point1[0]) ** 2) ** 0.5
					if (length > min_len):
						point1=point2
						result_list.append(point1)
						num = num + 1
			else:
				print("   got no utm data , please check topic :'/gps_meas'  or  rosnode : 'gps_to_umd'  ")  
			print("   got %d effective gnss -> utm points"%num)
			result = numpy.array(result_list)
			for index,item in enumerate(result):
				if index==0:
					if (index + 1 )< len(result):
						result[index][2] =  math.atan2(result[index+1][1]-result[index][1],result[index+1][0]-result[index][0])
					else:
						result[index][2] = 0
				elif (index == len(result)-1):
					result[index][2] =  math.atan2(result[index][1]-result[index-1][1],result[index][0]-result[index-1][0])
				else :
					temp1 =  math.atan2(result[index+1][1]-result[index][1],result[index+1][0]-result[index][0])
					temp2 =  math.atan2(result[index][1]-result[index-1][1],result[index][0]-result[index-1][0])
					temp3 = ((temp1-temp2)+math.pi*2)%(math.pi*2)
					result[index][2] =  temp1-temp3

			numpy.savetxt(self.final_result_path,result,delimiter = ',')
			print("   final waypoints transformed done !")
			print(' ')
			print('\033[32m   Save process is done !   \033[0m')
		else:
			print("   there has no gps point record ! ")

		

		
		





if __name__ == '__main__':
	try:
		GPS_Recorder()
	except rospy.ROSInterruptException:
		rospy.logerr('   Could not start gps record node.')

