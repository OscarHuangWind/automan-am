#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  3 15:41:17 2021

@author: oscar
"""

import numpy as np
import rospy
from .control_authority import Authority
from geometry_msgs.msg import Twist


class HRS():
    def __init__(self):
        self.weight_adas_cmd_lon_ = float(0.0)
        self.weight_adas_cmd_rot_ = float(0.0)
        self.weight_driver_cmd_lon_ = float(1.0)
        self.weight_driver_cmd_rot_ = float(1.0)
        self.vel_adas_ = Twist()
        self.vel_adas_.linear.x = 999
        self.vel_adas_.angular.z = 999
        self.authority = Authority()

    def SetParameter(self, cmd_vel_adas):
        if (cmd_vel_adas == None):
            self.weight_adas_cmd_lon_ = 0.0
            self.weight_adas_cmd_rot_ = 0.0
            # rospy.logwarn("Oscar::No command from ADAS.")

        self.vel_adas_ = cmd_vel_adas

    def CalFinalVelocityCmd(self, vel_cmd_final):
        flag = ((abs(self.vel_adas_.linear.x) > 100.0) and
                (abs(self.vel_adas_.angular.z) > 100.0))
        if (flag == True):
            self.weight_adas_cmd_lon_ = 0.0
            self.weight_adas_cmd_rot_ = 0.0
            self.weight_driver_cmd_lon_ = 1.0 - self.weight_adas_cmd_lon_
            self.weight_driver_cmd_rot_ = 1.0 - self.weight_adas_cmd_rot_
            # rospy.logwarn("Oscar::Driver is doing good.")
        else:
            if (self.vel_adas_.linear.z < 10):
                #### Safe Stop ####
                self.weight_adas_cmd_lon_ = 1.0
                self.weight_adas_cmd_rot_ = 0.0
                self.weight_driver_cmd_lon_ = 1.0 - self.weight_adas_cmd_lon_
                self.weight_driver_cmd_rot_ = 1.0 - self.weight_adas_cmd_rot_
            else:
                ##### Power Steering #####
                lat_risk_human = self.vel_adas_.angular.x
                # lat_risk_auto = self.vel_adas_.angular.y
                attention = 0.0
                # rospy.logwarn("Oscar::The x:%f, y:%f, risk_human:%f, attention:%f" %
                #               self.vel_adas_.angular.x, self.vel_adas_.angular.y, lat_risk_human, attention)
                self.authority.SetInput(lat_risk_human, attention)
                self.authority.ComputeAuthority()
                self.weight_driver_cmd_rot_ = self.authority.GetAuthority()

                if (self.weight_driver_cmd_rot_ < 0.01):
                    self.weight_driver_cmd_rot_ = 0

                self.weight_driver_cmd_lon_ = self.weight_driver_cmd_rot_

                # self.weight_adas_cmd_lon_ = lat_risk_human / (lat_risk_human + lat_risk_auto)
                # self.weight_adas_cmd_rot_ = lat_risk_human / (lat_risk_human + lat_risk_auto)
                self.weight_adas_cmd_lon_ = 1.0 - self.weight_driver_cmd_lon_
                self.weight_adas_cmd_rot_ = 1.0 - self.weight_driver_cmd_rot_

            if (self.vel_adas_.linear.x < 0):
                self.vel_adas_.linear.x = 0.0

        self.CalculateVelocity(vel_cmd_final)
        return True

    def Reset(self):
        self.weight_adas_cmd_lon_ = 0.0
        self.weight_adas_cmd_rot_ = 0.0
        self.weight_driver_cmd_lon_ = 1.0 - self.weight_adas_cmd_lon_
        self.weight_driver_cmd_rot_ = 1.0 - self.weight_adas_cmd_rot_

    def CalculateVelocity(self, vel_cmd):

        # rospy.logwarn("Oscar::the weight of adas is: [%f, %f]" %
        #               self.weight_adas_cmd_lon_, self.weight_adas_cmd_rot_)

        vel_cmd.linear.x = (self.weight_adas_cmd_lon_ * max(0.0, self.vel_adas_.linear.x)) +\
                                (self.weight_driver_cmd_lon_ * vel_cmd.linear.x)
        vel_cmd.angular.z = (self.weight_adas_cmd_rot_ * self.vel_adas_.angular.z) +\
                                 (self.weight_driver_cmd_rot_ *
                                  vel_cmd.angular.z)

    def GetVelocityCmd(self):
        return self.vel_adas_
