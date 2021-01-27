#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from matplotlib import pyplot as plt
import maze_solver as ms
import PPPKod as testing
import Color_recognition_and_tracking_speed_distance as CRATS
import sphero_ctl
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class sphero_test():
    def __init__(self):

        rospy.init_node("Sphero_test")
        self.sphero_pub = rospy.Publisher("/sphero_0/cmd_vel", Twist, queue_size=10)

        self.camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        #self.speed_class_green = CRATS.position_and_speed('green')
        self.speed_class_blue = CRATS.position_and_speed('blue', self.camera_pub)
        #self.speed_class_red = CRATS.position_and_speed('red')
        self.dbg_img = rospy.Publisher("dbg_img", Image, queue_size=1)

        self.sphero_control = sphero_ctl.SpheroControl(self.sphero_pub)
        rospy.Rate(4)
        t = Twist()
        t.angular.y = -90
        current_time = rospy.get_time()
        while rospy.get_time()-current_time<5.:
            self.camera_pub.publish(t)
        
        self.camera_sub = rospy.Subscriber("/bebop/image_raw", Image, self.get_image)
        self.mode = True
        #cv2.namedWindow('test')

        self.debug_variable_x = 0.0
        self.debug_variable_y = 50.0

        rospy.spin()
    def get_image(self, img_msg):
        begin=rospy.Time.now().to_sec()
        self.__cv_bridge = CvBridge()
        
        imageFrame=self.__cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8") # Needs to be checked
        #cv2.imshow('test', imageFrame)
        #cv2.waitKey(40)

        pts1 = np.array(0)
        try:
            pts1 = testing.izracunaj_koordinate_vrhova(imageFrame)
            pts1 = np.array(pts1)
        
        except Exception as e:
            #print(e)
            return
        if pts1.shape !=(4,2):
            return
        
             
            #plt.imshow(imageFrame)
            #plt.show()
            
        pts1 = np.array(pts1)
        #print(f"Points to transform: {pts1}")
        pts2 = np.float32([[0, 0], [0, 299], [299, 0],  [299, 299] ])
        M = cv2.getPerspectiveTransform(pts1.astype(np.float32),pts2)

        new_img = cv2.warpPerspective(imageFrame, M, (300, 300))
        #new_img = cv2.flip(new_img, 1)

        #speed_green = self.speed_class_green.get_sphero_speed(imageFrame)
        location_blue_x, location_blue_y = self.speed_class_blue.get_sphero_speed(new_img)
        #location_green_x, location_green_y = self.speed_class_green.get_sphero_speed(new_img)


        if self.debug_variable_x == 0:
            self.debug_variable_x = location_blue_x

        #speed_red = self.speed_class_red.get_sphero_speed(imageFrame)
        if self.mode:
            error = 100
            error =self.sphero_control.run_y((self.debug_variable_x, self.debug_variable_y), (location_blue_x, location_blue_y))
            #error_2 = self.sphero_control.run_x(0, speed_blue_x)
            print("error = " + str(error))
            if abs(error) < 10:
                self.mode = False
        else:
            zero = self.sphero_control.run_y(0.0, speed_blue_y)
        end=rospy.Time.now().to_sec()
        print("time passed: " + str(end-begin))
        print()
if __name__ == "__main__":
#    if rospy.is_shutdown("Sphero_test"):
#        rospy.signal_shutdown("Sphero_test")
    x = sphero_test()