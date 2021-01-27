#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from matplotlib import pyplot as plt
import position as testing
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class sphero_test():
    def __init__(self):

        rospy.init_node("Sphero_test")
        self.sphero_pub = rospy.Publisher("/sphero_0/cmd_vel", Twist, queue_size=10)

        self.camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        
        self.drone_currpoint = rospy.Publisher("currentPose", Pose, queue_size = 1)
        self.drone_setpoint = rospy.Publisher("pose", Pose, queue_size = 1)

        t = Twist()
        t.angular.y = -90
        current_time = rospy.get_time()
        while rospy.get_time()-current_time<5.:
            self.camera_pub.publish(t)
        
        self.camera_sub = rospy.Subscriber("bebop/image_raw", Image, self.get_image)
        self.mode = True
        #cv2.namedWindow('test')

        self.debug_variable_x = 0.0
        self.debug_variable_y = 50.0

        #Set current drone point to (0,0,0)
        currpoint = Pose()
        currpoint.position.x = 0
        currpoint.position.y = 0
        currpoint.position.z = 0
        currpoint.orientation.x = 0
        currpoint.orientation.y = 0
        currpoint.orientation.z = 0
        self.drone_setpoint.publish(currpoint)
        self.drone_setpose = Pose()
        self.drone_positioned = False
        self.detected = 0
        
        self.desired_z = 60
        self.desired_x = 500    # TODO set actual values values
        self.desired_y = 460

        rospy.spin()

    def get_image(self, img_msg):
        begin=rospy.Time.now().to_sec()
        self.__cv_bridge = CvBridge()
        
        imageFrame=self.__cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8") # Needs to be checked
        #cv2.imshow('test', imageFrame)
        #cv2.waitKey(40)
            
        diagonal, posx, posy = testing.dron_QRKod(imageFrame)
        print(diagonal)
        if diagonal == 0 and posx == 0 and posy == 0:
            self.drone_setpose.position.z = 0
            self.drone_setpose.position.x = 0   
            self.drone_setpose.position.y = 0
        else:
            self.drone_setpose.position.z = (diagonal - self.desired_z) / 100
            self.drone_setpose.position.x = (self.desired_x - posx) / 100
            self.drone_setpose.position.y = (self.desired_y - posy) / 100
        if abs(self.drone_setpose.position.z) < 0.2 and abs(self.drone_setpose.position.x) < 0.2 and abs(self.drone_setpose.position.y) < 0.2:
            self.drone_positioned = True
        #    print("POSITIONED")
        print(f"z:{self.drone_setpose.position.z} x: {self.drone_setpose.position.x} y: {self.drone_setpose.position.y} ")
            
        if diagonal != -100.00:
            self.drone_currpoint.publish(self.drone_setpose)

        #print("time passed: " + str(end-begin))
        #print()
if __name__ == "__main__":
#    if rospy.is_shutdown("Sphero_test"):
#        rospy.signal_shutdown("Sphero_test")
    x = sphero_test()
