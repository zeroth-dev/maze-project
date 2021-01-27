#!/usr/bin/env python3

import rospy
import math
from pid import PID
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist


class AttitudeControl:
    """
    Class implements attitude control (roll, pitch, yaw).
    """
    def __init__(self):
        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles reference values
        self.x_mv = 0                       # x-position measured value
        self.x_sp = 0                       # x-position set point
        self.y_mv = 0                       # y-position measured value
        self.y_sp = 0                       # y-position set point

        self.pid_roll = PID(1, 0, 1.45, 1, -1)     # roll controller
        self.pid_pitch = PID(1, 0, 1.45, 1, -1)    # pitch controller
        self.pid_yaw = PID(2, 0, 0, 5, -5)         # yaw controller

        self.vel_msg = Twist()


    def runPose(self, data):
        self.vel_msg.linear.x = self.pid_roll.compute(self.x_sp, data.position.x)
        self.vel_msg.linear.y = self.pid_pitch.compute(self.x_sp, data.position.y)
        self.vel_msg.angular.z = self.pid_yaw.compute(self.euler_sp.z, data.orientation.z)



    # -- ODOMETRY FUNCTIONS -- NOT USED ANYMORE
    #def run(self, data):
    #    """
    #    Computes PID algorithms for attitude control.
    #    "Input" in roll PID controller is x-position.
    #    "Input" in pitch PID controller is y-position.
    #    "Input" in yaw PID controller is z-angle.
    #    "Output" of roll PID controller is x-angle.
    #    "Output" of pitch controller is y-angle.
    #    "Output" of yaw PID controller is yaw rate.
    #    :return: Type geometry_msgs/Twist
    #    """
    #    self.odometry_cb(data)
    #    self.vel_msg.linear.x = self.pid_roll.compute(self.x_sp, self.x_mv)
    #    self.vel_msg.linear.y = self.pid_pitch.compute(self.y_sp, self.y_mv)
    #    #self.vel_msg.angular.z = self.pid_yaw.compute(self.euler_sp.z, self.euler_mv.z) OVO ODKOMENTIRAJ ZA ROT
    #    #print(self.vel_msg)

    #def odometry_cb(self, msg):
    #    """
    #    Odometry callback. Used to extract roll, pitch, yaw and their rates.
    #    We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
    #    :param msg: Type nav_msgs/Odometry
    #    """
    #    
    #    self.x_mv = msg.pose.pose.position.x
    #    self.y_mv = msg.pose.pose.position.y

    #    qx = msg.pose.pose.orientation.x
    #    qy = msg.pose.pose.orientation.y
    #    qz = msg.pose.pose.orientation.z
    #    qw = msg.pose.pose.orientation.w

    #    # conversion quaternion to euler (yaw - pitch - roll)
    #    self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
    #    self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
    #    self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
