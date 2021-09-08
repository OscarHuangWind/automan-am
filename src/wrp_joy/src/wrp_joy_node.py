#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 14:51:54 2021

@author: jianhuang
"""

import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Range
from geometry_msgs.msg import Twist
import threading
import rospy
from collections import deque
import sys
sys.path.append("../..")
from  src.human_robot_interaction.src.human_robot_system import HRS
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
dec_idx = 0.0
safe_dist = MAX_RANGE

joy_data = message_filters.Subscriber('joy', Joy)
cmd_vel_adas_data = message_filters.Subscriber("/cmd_vel_adas", Twist)
odom_data = message_filters.rospy.Suscriber('/odom', Odometry)


class TeleopWR():

    def __init__(self):

        self.dec_idx_ = 0
        self.flag_ = False
        self.safe_scale_ = 0.05
        self.velo_x_ = 0.0

        self.ultra0_ = message_filters.Subscribe('/ultrasonic1', Range)
        self.ultra1_ = message_filters.Subscribe('/ultrasonic3', Range)
        self.ultra2_ = message_filters.Subscribe('/ultrasonic2', Range)
        self.ultra3_ = message_filters.Subscribe('/ultrasonic6', Range)

        self.ats_ = message_filters.appoximateTimeSynchronizer(
            [self.ultra0_, self.ultra1_, self.ultra2_, self.ultra3_], queue_size=10, slop=0.1)
        self.ats_.registerCallback(
            self.ultra0_, self.ultra1_, self.ultra2_, self.ultra3_)

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
        self.twist_buffer_ = deque(maxlen=10)

        self.hrs = HRS()

        # suscriber
        self.cmd_adas_sub_ = rospy.Suscriber(
            '/cmd_vel_adas', Twist, self.adas_CB)
        self.joy_sub_ = rospy.Suscriber('/joy', Joy, self.joyCallback)
        self.odom_sub_ = rospy.Suscriber('/odom', Odometry, self.odom_CB)

        # publisher
        self.wr_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.adas_trigger_pub_ = rospy.Publisher(
            'cmd_vel_driver', Twist, queue_size=100)

    def adas_CB(self, twist):
        #rospy.loginfo("Received a /cmd_vel_adas message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(twist.linear.x, twist.linear.y, twist.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(twist.angular.x, twist.angular.y, twist.angular.z))
        self.hrs.SetParameter(twist)

    def odom_CB(self, odom):
        # rospy.loginfo("Oscar::Received message from hunter. %f, %f", odom->twist.twist.linear.x, odom->twist.twist.angular.z)
        self.velo_x_ = odom.twist.twist.linear.x

    def joyCallback(self, joy):

        twist_data = Twist()
        if(joy.buttons[5] == 1 and joy.axes[self.acc_idx_] == -1 and joy.axes[self.dec_idx_] == -1):
            self.flag_ = True

        if (self.flag_):
            twist_data.angular.z = self.a_scale_ * joy.axes[self.angular_idx_]
            if (joy.buttons[4] != 1):

                if (joy.axes[self.dec_idx] > -1):
                    with lock:
                        #ROS_WARN("The brake velocity is:%f", velo_x_)
                        twist_data.linear.x = max(self.velo_x_ - min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), 0.0)
                elif (joy.axes[self.acc_idx_] > self.threshold_):
                    with lock:
                        #ROS_WARN("The acc velocity is:%f", velo_x_)
                        twist_data.linear.x = min(self.velo_x_ + min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), self.velo_limit_)
                elif (joy.axes[self.acc_idx_] > -1 and joy.axes[self.acc_idx_] <= self.threshold_):
                    with lock:
                        #ROS_WARN("The maintain velocity is:%f", velo_x_)
                        twist_data.linear.x = min(
                            self.velo_x_, self.velo_limit_)
                else:
                    with lock:
                        #ROS_WARN("The non-op velocity is:%f", velo_x_)
                        twist_data.linear.x = max(
                            self.velo_x_ - min(self.dec_default_, self.acc_limit_), 0.0)

            else:
                if (joy.axes[self.dec_idx_] > -1):
                    with lock:
                        #ROS_WARN("THHHHE brake velocity is:%f", velo_x_)
                        twist_data.linear.x = min(self.velo_x_ + min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.dec_idx_] + 1), self.acc_limit_), 0.0)

                elif (joy.axes[self.acc_idx_] > -1):
                    with lock:
                        #ROS_WARN("THHHHE acc velocity is:%f", velo_x_)
                        twist_data.linear.x = max(self.velo_x_ - min(self.safe_scale_ * self.l_scale_ * (
                            joy.axes[self.acc_idx_] + 1), self.acc_limit_), -self.velo_limit_)

                else:
                    with lock:
                        #ROS_WARN("THHHHE non-op velocity is:%f", velo_x_)
                        twist_data.linear.x = min(
                            self.velo_x_ + min(self.dec_default_, self.acc_limit_), 0.0)

            self.twist_buffer_.append(twist_data)

        cmd_vel_lon_sum = 0.0
        cmd_vel_rot_sum = 0.0

        for t in self.twist_buffer_:
            cmd_vel_lon_sum += t.linear.x
            cmd_vel_rot_sum += t.angular.z

        twist_data.linear.x = (cmd_vel_lon_sum /
                               max((len(self.twist_buffer_)), 1))
        twist_data.angular.z = (cmd_vel_lon_sum /
                                max((len(self.twist_buffer_)), 1))

        self.adas_trigger_pub_.publish(twist_data)
        #ROS_WARN("Oscar::the flattened driver velocity is: %f, %f", twist.linear.x, twist.angular.z);

        self.hrs.CalFinalVelocityCmd(twist_data)
        #ROS_WARN("Oscar::THE final velocity is: %f, %f", twist.linear.x, twist.angular.z);
        self.wr_pub_.publish(twist_data)

    def ultraCallBack(self):
        if ((self.ultra0_.range > MIN_RANGE and self.ultra0_.range < self.safe_dist_) or
            (self.ultra1_.range > MIN_RANGE and self.ultra1_.range < self.safe_dist_) or
            (self.ultra2_.range > MIN_RANGE and self.ultra2_.range < self.safe_dist_) or
                (self.ultra3_.range > MIN_RANGE and self.ultra3_.range < self.safe_dist_)):
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
