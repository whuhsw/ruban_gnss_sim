#!/usr/bin/env python
# coding:utf-8
# -----------------------------
#  Author  :  hao 
#  Version :  1.2.0 
#  date    :  19.8.1.11
#  status  :  Done
#  update  :  yaw calculation
# -----------------------------
import os
import rospy
import math, time
import numpy
import tf
import threading
from geometry_msgs.msg import Quaternion,PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence

# -----------------------  congig  --------------------------------------------------
# 订阅的话题
utm_topic = '/gps_meas'  						# utm话题 pkg="gps_common" type="utm_odometry_node"
odom_topic = '/wheel_odom'  					# odom话题
origin_point_topic = '/origin_point'  			# utm原点话题
base_path_topic = '/base_path'  				# 全局路线
rad_topic = '/navsat/nmea_sentence'  			# 获得角度

# 发布的话题
pose_frame_id = 'utm' 							# 发布的话题在rviz的frame id
new_utm_odom_topic = '/utm_odom'  				# utm坐标系下里程计
new_utm_pose_topic = '/utm_pose'  				# utm坐标系下实时位姿

# -------------------  config  -------------------------------------------------------
'''
代表相邻rtk方位角帧误差一致假设
以低频的gnss定位上一帧的gnss位置作为基准点， 高频的odom上一帧的xy作第二个顶点
当前帧的gnss位置作为第三个顶点，求出夹角即为此gnss帧间误差，以此修正下一帧误差
这是建立在相邻gnss帧间，车辆运动情况是类似的假设上。如果要关闭这个模式，请在源
码config区 修改near_heading_error_same_assume = False 即可。
near_heading_error_k  误差可设定衰减比例 [0,1]之间
关闭后，需要每换一次区域后重新校正误差，修改 yaw_correct
相关方法 def  cal_near_yaw_error（）
手动矫正方法 小车直行一段距离，选择一段运行稳定的轨迹，以gnss相邻两点构成的直线角度yaw1，
此两点内估计的odom轨迹所在的直线yaw2，二者角度差即为与真实gnss的角度差
yaw_correct = yaw2 - yaw1  （弧度）
'''
near_heading_error_same_assume = True  
near_heading_error_k = 1  

# ------------------------------------------------------------------------------------
ODOM_TO_UTM_XY_K = 1                    # odom里程计测得长度与utm测得长度比例修正系数
ODOM_TO_UTM_YAW_K = 1                   # odom里程计测得长度与utm测得角度比例修正系数
yaw_correct = -0.047    # rviz utm方位角与小车方位角角度误差 ，直行时，小车的utm_pose话题指向- 
                        # -应该与小车的行驶轨迹utm_odom 话题）一致 ，如不一致 修改此值 
# ------------------------------------------------------------------------------------
'''   
  注释中的符号说明

  u_x  u_y  u_yaw           gnss经纬度转换莫卡托   x y yaw   
  m_ox m_oy m_oyaw          得到gnss时的里程计x y    -(millstone_odom_x / y / yaw)
  o_x  o_y  o_yaw           odom里程计实时 x y yaw   
  l_ox l_oy                 上一帧的里程计数据以积分得到累计值   -(last_odom_x / y) 
  new_x  new_y new_yaw      高频odom+低频gnss差值融合后的实时 x y yaw 值
  pose_update_tag  yaw_update_tag   每收到有效信号就置为1 在真值基础上估计实时位姿

'''
class Utm_pose_updater(object):

    def __init__(self):
        time.sleep(2)
        self.mutex = threading.Lock()
        rospy.init_node('Utm_pose_updater', log_level=rospy.INFO)
        self.origin_point = None
        self.pose_utm = None
        self.pose_odom = None
        self.pose_rad = None
        self.every_time_data = [0, 0, 0, 0, 0, 0,0,0]   # [u_x,u_y,u_yaw,o_x,o_y,o_yaw,pose_update_tag,yaw_update_tag] 记录每一时刻的数据
        self.pose_temp_arr = [0, 0, 0, 0, 0, 0]         # [m_ox,m_oy,l_ox,l_oy,new_x,new_y] 临时存储xy变量
        self.yaw_temp_arr = [0, 0]                      # [m_oyaw new_yaw]  临时存储 yaw用到的变量
        self.err_corr = 0                               # 假定相邻fix帧间角度误差一致，利用上一帧误差修正下一帧的角度
        self.pose_correct_temp_arr =[0,0,0,0,0,0]       # 存储上一帧的gnss坐标 上一帧dodm x y 最终 x y
        rospy.Subscriber(origin_point_topic, PoseStamped, self.got_origin_cb, queue_size=1)
        rospy.Subscriber(rad_topic, Sentence, self.rtk_yaw_cb, queue_size=1)
        rospy.Subscriber(utm_topic, Odometry, self.robot_pose_utm_cb, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.robot_pose_odom_cb, queue_size=1)

        self.pub_utm_odom = rospy.Publisher(new_utm_odom_topic, Odometry, queue_size=1, latch=True)
        self.pub_utm_pose = rospy.Publisher(new_utm_pose_topic, PoseStamped, queue_size=1, latch=True)

        print("   publishing utm odom topic              :  %s" % new_utm_odom_topic)
        print("   publishing utm pose topic              :  %s" % new_utm_pose_topic)
        print("")
        print("                                 ---  node working ---")
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def got_origin_cb(self, data):
        if not self.origin_point:
            self.origin_point = data

    def rtk_yaw_cb(self, data):
        if self.origin_point:
            self.mutex.acquire()
            str = data.sentence
            self.pose_rad = float(str)
            yaw_new = self.pose_rad * math.pi / 180
            self.every_time_data[2] =(-1)* (yaw_new - math.pi/2)      # 注意负号 rtk方位角顺时针增多，与天线放置位置有关
            self.every_time_data[7] = 1                               # + math.pi 是rtk测的角度在xoy坐标系存在90度偏差
            self.mutex.release()

    def robot_pose_utm_cb(self, data):
        if self.origin_point:
            self.pose_utm = data
            print(data)
            if self.pose_correct_temp_arr[0] != 0 and self.pose_correct_temp_arr[1] !=0:
                self.pose_correct_temp_arr[0]= self.every_time_data[0]  # 记录上一次的gnss相对坐标
                self.pose_correct_temp_arr[1]= self.every_time_data[1]
            self.every_time_data[0] = self.pose_utm.pose.pose.position.x - self.origin_point.pose.position.x
            self.every_time_data[1] = self.pose_utm.pose.pose.position.y - self.origin_point.pose.position.y
            self.every_time_data[6] = 1

    def robot_pose_odom_cb(self, data):
        if self.pose_utm and self.pose_rad:
            self.mutex.acquire()
            self.pose_odom = data
            self.every_time_data[3] = self.pose_odom.pose.pose.position.x 
            self.every_time_data[4] = self.pose_odom.pose.pose.position.y 
            self.every_time_data[5] = self.cal_yaw(self.pose_odom.pose.pose.orientation)
            self.cal_orientation()   
            print(near_heading_error_same_assume)
            if near_heading_error_same_assume:
                self.cal_position_try_correct_error()
            else:        
                self.cal_position()
            self.publish_new_pose()
            self.mutex.release()



    def cal_position_try_correct_error(self):
        if self.every_time_data[6] == 1:
            self.pose_correct_temp_arr[2] = self.pose_temp_arr[4]
            self.pose_correct_temp_arr[3] = self.pose_temp_arr[5]

            last_fix=[self.pose_correct_temp_arr[0],self.pose_correct_temp_arr[1]]
            now_est_odom = [self.pose_correct_temp_arr[2],self.pose_correct_temp_arr[3]]
            now_fix = [self.every_time_data[0],self.every_time_data[1]]
            self.err_corr = self.cal_near_yaw_error(last_fix,now_est_odom,now_fix)

            self.pose_temp_arr[2] = self.every_time_data[3]
            self.pose_temp_arr[3] = self.every_time_data[4]
            self.pose_temp_arr[4] = self.every_time_data[0]  
            self.pose_temp_arr[5] = self.every_time_data[1]  
            self.every_time_data[6] = 0
        else: 
            x1 = self.pose_temp_arr[2]   # last odom
            y1 = self.pose_temp_arr[3]
            x2 = self.every_time_data[3] 
            y2 = self.every_time_data[4]
            self.pose_temp_arr[2] = self.every_time_data[3]  # 当前odom存入temp以便下次运算
            self.pose_temp_arr[3] = self.every_time_data[4]
            length = ((y2 - y1) ** 2 + (x2 - x1) ** 2) ** 0.5 / ODOM_TO_UTM_XY_K
            part_angle = math.atan2((y2 - y1), (x2 - x1))
            length_x = length * math.cos(part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] - yaw_correct)
            length_y = length * math.sin(part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] - yaw_correct)

            self.pose_temp_arr[4] = self.pose_temp_arr[4] + length * math.cos(
                part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] + yaw_correct + self.err_corr)
            self.pose_temp_arr[5] = self.pose_temp_arr[5] + length * math.sin(
                part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] + yaw_correct + self.err_corr)
            print(self.err_corr)

    def cal_position(self):
        print("-----------------------------------------------------------------------")
        if self.every_time_data[6] == 1:
            self.pose_temp_arr[2] = self.every_time_data[3]
            self.pose_temp_arr[3] = self.every_time_data[4]
            self.pose_temp_arr[4] = self.every_time_data[0]  # true realtime new x
            self.pose_temp_arr[5] = self.every_time_data[1]  # true realtime new y
            self.every_time_data[6] = 0
        else:
            x1 = self.pose_temp_arr[2]   # last odom
            y1 = self.pose_temp_arr[3]
            x2 = self.every_time_data[3] 
            y2 = self.every_time_data[4]
            self.pose_temp_arr[2] = self.every_time_data[3]  # 当前odom存入temp以便下次运算
            self.pose_temp_arr[3] = self.every_time_data[4]
            length = ((y2 - y1) ** 2 + (x2 - x1) ** 2) ** 0.5 / ODOM_TO_UTM_XY_K
            part_angle = math.atan2((y2 - y1), (x2 - x1))
            self.pose_temp_arr[4] = self.pose_temp_arr[4] + length * math.cos(
                part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] + yaw_correct)
            self.pose_temp_arr[5] = self.pose_temp_arr[5] + length * math.sin(
                part_angle + self.yaw_temp_arr[1] - self.yaw_temp_arr[0] + yaw_correct)

    def cal_orientation(self):
        # mutex.acquire()
        if self.every_time_data[7] == 1:
            self.yaw_temp_arr[0] = self.every_time_data[5]
            self.yaw_temp_arr[1] = self.every_time_data[2]
            self.every_time_data[7] = 0

        else:
            self.yaw_temp_arr[1] = self.every_time_data[2] + (self.every_time_data[5] - self.yaw_temp_arr[0]) * ODOM_TO_UTM_YAW_K


    # mutex.release()

    def publish_new_pose(self):
        # mutex.acquire()
        new_utm_odom = Odometry()
        new_utm_odom.header.frame_id = pose_frame_id
        new_utm_odom.header.stamp = rospy.Time(0)
        new_utm_odom.pose.pose.position.x = self.pose_temp_arr[4]
        new_utm_odom.pose.pose.position.y = self.pose_temp_arr[5]
        new_utm_odom.pose.pose.position.z = 0.02
        # 下一行 + math.pi 是gps方位角xy坐标系与odom的xy坐标系存在90度偏差  yaw_correct 角度误差修正
        orientation_xyzw = tf.transformations.quaternion_from_euler(0, 0, self.yaw_temp_arr[1] + yaw_correct + self.err_corr)
        new_utm_odom.pose.pose.orientation.x = orientation_xyzw[0]
        new_utm_odom.pose.pose.orientation.y = orientation_xyzw[1]
        new_utm_odom.pose.pose.orientation.z = orientation_xyzw[2]
        new_utm_odom.pose.pose.orientation.w = orientation_xyzw[3]
        new_utm_pose = PoseStamped()
        new_utm_pose.header = new_utm_odom.header
        new_utm_pose.pose = new_utm_odom.pose.pose
        self.pub_utm_odom.publish(new_utm_odom)
        self.pub_utm_pose.publish(new_utm_pose)

    def cal_yaw(self, quat):

        quanternion = (quat.x, quat.y, quat.z, quat.w)
        eular = tf.transformations.euler_from_quaternion(quanternion)
        eular = list(eular)
        return eular[2]


    '''
    以低频的gnss定位上一帧的gnss位置作为基准点， 高频的odom上一帧的xy作第二个顶点
    当前帧的gnss位置作为第三个顶点，求出夹角即为此gnss帧间误差，以此修正下一帧误差
    这是建立在相邻gnss帧间，车辆运动情况是类似的假设上，如果要关闭这个模式，请在源
    码config区 修改near_heading_error_same_assume = False 即可。
    关闭后，建议每换一次区域后重新校正误差，修改 yaw_correct
    '''
    def cal_near_yaw_error(self,last_fix,now_est_odom,now_fix): 
        x1 = last_fix[0]
        y1 = last_fix[1]
        x2 = now_est_odom[0]
        y2 = now_est_odom[1]
        x3 = now_fix[0]
        y3 = now_fix[1]
        l1 = ((y3-y1)**2+(x3-x1)**2)**0.05
        l2 = ((y2-y1)**2+(x2-x1)**2)**0.05
        alpha_ture = math.atan2(y3-y1,x3-x1) #真值
        alpha_esti = math.atan2(y2-y1,x2-x1) #估计值
        if l1 > 0.2 and l2 > 0.2:   #行驶距离太短无需修正 
            error_correct = (alpha_ture -alpha_esti)*near_heading_error_k
        else:
            error_correct = 0
        return error_correct


    def shutdown(self):
        time.sleep(0.3)
        print('\033[32m   node shutdown done !   \033[0m')



if __name__ == '__main__':
    try:

        Utm_pose_updater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start utm pose updater node.')
