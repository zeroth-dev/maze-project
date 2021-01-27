#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from attitude_ctl import AttitudeControl
from height_ctl import HeightControl
from sensor_msgs.msg import Joy
import math

# -- Start drone driver with roslaunch bebop_driver bebop_node.launch


class DroneController:
    """
    Class that does the control.
    """
    def __init__(self):
        """
        Constructor.
        """
        # Publishers and subscribers.
        self.takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)              # -- Publishes a single Empty message, signaling the drone for takeoff
        self.land_pub = rospy.Publisher('bebop/land', Empty, queue_size=1)                         # -- Publishes a message to land, not used right now.
        self.speedPublisher = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)          # -- Publishes speed
        
        self.currentPointInfo = rospy.Subscriber('currentPose', Pose, self.curr_pose_cb)      # -- Sets current pose if odometry is not used
        self.setPointInfo = rospy.Subscriber('pose', Pose, self.set_point_cb)                 # -- Sets desired position
        self.joystick = rospy.Subscriber("bebop/joy", Joy, self.joystick_ctl)                            # -- Gets joystick info
        
        #self.odometryInfo = rospy.Subscriber('bebop/odom', Odometry, self.runControl)        # -- Gets odometry from the drone. Frequency is 5 Hz
        
        self.joystickTriggerPressed = False
        self.att_ctl = AttitudeControl()
        self.hgt_ctl = HeightControl()

    # -- calls regulators when allowed
    def curr_pose_cb(self, data):
        if self.joystickTriggerPressed:
            print("Data received!")
            self.att_ctl.runPose(data)
            self.hgt_ctl.runPose(data)  # -- Check data is being proccesed correctly TODO
            
            # -- merge both Twists into one and send it to the drone
            att_info = self.att_ctl.vel_msg
            att_info.linear.z = -self.hgt_ctl.vel_msg.linear.z / 2
            att_info.linear.x = -att_info.linear.x / 10
            att_info.linear.y = -att_info.linear.y / 10
            print("Published att_info!")
            self.speedPublisher.publish(att_info)

    # -- disables regulators while RT is pressed
    def joystick_ctl(self, data):
        if data.buttons[6] == 1.0    :       # -- 7 should be RT, check! TODO
            self.joystickTriggerPressed = True
        else:
            #print("LT for regulator output")
            self.joystickTriggerPressed = False

    def set_point_cb(self, data):
        """
        Setpoint callback.
        :param data: geometry_msgs/Pose
        """
        print("Got desired pose")
        self.hgt_ctl.z_sp = data.position.z
        self.att_ctl.x_sp = data.position.x
        self.att_ctl.y_sp = data.position.y
        self.att_ctl.euler_sp.z = data.orientation.z; # expects euler rotation, not quternion!        

    # -- ODOMETRY FUNCTIONS -- NOT USED ANYMORE
    #def runControl(self, data):
    #    """
    #    Odometry callback.
    #    :param data: nav_msgs/Odometry
    #    """
    #    if not self.joystickTriggerPressed:
    #        #print(data)
    #        print("Data received!")
    #        self.att_ctl.run(data)                           # -- gets roll, yaw, pitch
    #        self.hgt_ctl.run(data)                           # -- gets z speed
            
    #        # -- merge both Twists into one and send it to the drone
    #        att_info = self.att_ctl.vel_msg
    #        att_info.linear.z = self.hgt_ctl.vel_msg.linear.z  
    #        att_info.linear.x = 0;
    #        att_info.linear.y = 0;      
    #        print("Published att_info!")
    #        #print(att_info)
    #        self.speedPublisher.publish(att_info)

if __name__ == "__main__":
    rospy.init_node("drone_controller")
    DC = DroneController()
    #DC.takeoff_pub.publish(Empty())
    #rospy.sleep(0.5)
    #DC.takeoff_pub.publish(Empty())
    #rospy.sleep(0.5)
    #DC.takeoff_pub.publish(Empty())
    print("Ready for odometry info")
    rospy.spin()
