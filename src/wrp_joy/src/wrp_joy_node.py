#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 14:51:54 2021

@author: jianhuang
"""

import rospy
import threading
import message_filters
from collections import deque
from statistics import mean

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from human_robot_interaction.src.human_robot_system import HRS

lock = threading.Lock()
MIN_RANGE = 0.2000
MAX_RANGE = 0.9000

velo_limit = 0.5
acc_limit = 0.5
dec_default = 0.1
threshold = -0.5
acc_idx = 2
angular_idx = 0
l_scale = 0.5
a_scale = 0.3
dec_idx = 0
safe_dist = MAX_RANGE

class TeleopWR():

    def __init__(self):

        self.dec_idx_ = 0
        self.flag_ = False
        self.safe_scale_ = 0.05
        self.velo_x_ = 0.0

        self.ultra0_ = message_filters.Subscriber('/ultrasonic1', Range)
        self.ultra1_ = message_filters.Subscriber('/ultrasonic3', Range)
        self.ultra2_ = message_filters.Subscriber('/ultrasonic2', Range)
        self.ultra3_ = message_filters.Subscriber('/ultrasonic6', Range)

        self.threshold_ = rospy.get_param('threshold', threshold)
        self.vel_limit_ = rospy.get_param('velo_limit', velo_limit)
        self.dec_default_ = rospy.get_param('dec_default', dec_default)
        self.acc_limit_ = rospy.get_param('acc_limit', acc_limit)
        self.acc_idx_ = rospy.get_param('vel_acc', acc_idx)
        self.dec_idx_ = rospy.get_param('vel_dec', dec_idx)
        self.angular_idx_ = rospy.get_param('vel_angular', angular_idx)
        self.l_scale_ = rospy.get_param('scale_linear', l_scale)
        self.a_scale_ = rospy.get_param('scale_angular', a_scale)
        self.safe_dist_ = rospy.get_param('safe_distance', safe_dist)
        self.twist_linear_buffer_ = deque(maxlen=40)
        self.twist_angular_buffer_ = deque(maxlen=40)

        self.ats_ = message_filters.ApproximateTimeSynchronizer([self.ultra0_, self.ultra1_, self.ultra2_, self.ultra3_], queue_size=10, slop=0.1)
        self.ats_.registerCallback(self.ultraCallBack)

        # publisher
        self.wr_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.adas_trigger_pub_ = rospy.Publisher('/cmd_vel_driver', Twist, queue_size=1)

        self.hrs = HRS()

        # suscriber
        self.cmd_adas_sub_ = rospy.Subscriber('/cmd_vel_adas', Twist, self.adas_CB)
        self.joy_sub_ = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.odom_sub_ = rospy.Subscriber('/odom', Odometry, self.odom_CB)

        self.attention_states_sub_ = rospy.Subscriber('/attention_states', Float64, self.states_CB)

    def adas_CB(self, twist):
        #rospy.loginfo("Received a /cmd_vel_adas message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(twist.linear.x, twist.linear.y, twist.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(twist.angular.x, twist.angular.y, twist.angular.z))
        self.hrs.SetParameter(twist)
    
    def states_CB(self, attention):
        self.hrs.SetAttention(attention)
        
    def odom_CB(self, odom):
        # rospy.loginfo("Oscar::Received message from hunter. %f, %f", odom->twist.twist.linear.x, odom->twist.twist.angular.z)
        self.velo_x_ = odom.twist.twist.linear.x

    

    def joyCallback(self, joy):

        twist_data = Twist()
        if(joy.buttons[5] == 1 and joy.axes[self.acc_idx_] == -1 and joy.axes[self.dec_idx_] == -1):
            print('7253')
            self.flag_ = True
        if (self.flag_):
            twist_data.angular.z = self.a_scale_ * joy.axes[self.angular_idx_]
            if (joy.buttons[4] != 1):

                if (joy.axes[self.dec_idx_] > -1):
                    with lock:
                        # rospy.logwarn("The brake velocity is:%f", self.velo_x_)
                        twist_data.linear.x = max(self.velo_x_ - min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), 0.0)
                elif (joy.axes[self.acc_idx_] > self.threshold_):
                    with lock:
                        # rospy.logwarn("The acc velocity is:%f", self.velo_x_)
                        twist_data.linear.x = min(self.velo_x_ + min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), self.vel_limit_)
                elif (joy.axes[self.acc_idx_] > -1 and joy.axes[self.acc_idx_] <= self.threshold_):
                    with lock:
                        # rospy.logwarn("The maintain velocity is:%f", self.velo_x_)
                        twist_data.linear.x = min(self.velo_x_, self.vel_limit_)
                else:
                    with lock:
                    # rospy.logwarn("The non-op velocity is:%f", self.velo_x_)
                        twist_data.linear.x = max(self.velo_x_ - min(self.dec_default_, self.acc_limit_), 0.0)
            else:
                if (joy.axes[self.dec_idx_] > -1):
                    with lock:
                        # rospy.logwarn("THHHHE brake velocity is:%f", self.velo_x_)
                        twist_data.linear.x = min(self.velo_x_ + min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.dec_idx_] + 1), self.acc_limit_), 0.0)

                elif (joy.axes[self.acc_idx_] > -1):
                    with lock:
                        # rospy.logwarn("THHHHE acc velocity is:%f", self.velo_x_)
                        twist_data.linear.x = max(self.velo_x_ - min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), -self.vel_limit_)

                else:
                    with lock:
                    # rospy.logwarn("THHHHE non-op velocity is:%f", self.velo_x_)
                        twist_data.linear.x = min(self.velo_x_ + min(self.dec_default_, self.acc_limit_), 0.0)

            self.twist_linear_buffer_.append(twist_data.linear.x)
            self.twist_angular_buffer_.append(twist_data.angular.z)

        if (len(self.twist_linear_buffer_) != 0):
            twist_data.linear.x = mean(self.twist_linear_buffer_)
            twist_data.angular.z = mean(self.twist_angular_buffer_)

        # for t in self.twist_buffer_:
        #     cmd_vel_lon_sum += t.linear.x
        #     cmd_vel_rot_sum += t.angular.z

        # twist_data.linear.x = (cmd_vel_lon_sum /
        #                        max((len(self.twist_buffer_)), 1))
        # twist_data.angular.z = (cmd_vel_rot_sum /
        #                         max((len(self.twist_buffer_)), 1))

        self.adas_trigger_pub_.publish(twist_data)
        # rospy.logwarn("Oscar::THE driver velocity is: %f, %f", twist_data.linear.x, twist_data.angular.z)\

        self.hrs.CalFinalVelocityCmd(twist_data)
        # rospy.logwarn("Oscar::THE final velocity is: %f, %f", twist_data.linear.x, twist_data.angular.z)

        twist_data.angular.y = self.hrs.weight_driver_cmd_lon_
        twist_data.angular.x = self.hrs.weight_adas_cmd_lon_

        self.wr_pub_.publish(twist_data)

    def ultraCallBack(self, ultra0, ultra1, ultra2, ultra3):
        if ((ultra0.range > MIN_RANGE and ultra0.range < self.safe_dist_) or
            (ultra1.range > MIN_RANGE and ultra1.range < self.safe_dist_) or
            (ultra2.range > MIN_RANGE and ultra2.range < self.safe_dist_) or
                (ultra3.range > MIN_RANGE and ultra3.range < self.safe_dist_)):
            self.safe_scale_ = 0.05
        else:
            self.safe_scale_ = 1.0


if __name__ == '__main__':
    try:
        rospy.init_node('teleop_wr', anonymous=True)
        TR = TeleopWR()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
