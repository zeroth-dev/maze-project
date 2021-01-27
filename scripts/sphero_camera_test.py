#! /usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import sphero_ctl
import time
import PPPKod as testing
import maze_solver as ms
import Color_recognition_and_tracking_speed_distance as CRATS
#import maze_solver as ms

class DroneControl:
    def __init__(self):
        self.__cv_bridge=CvBridge()
        rospy.init_node("Drone_controller", anonymous=True)
        self.camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        self.sphero_pub = rospy.Publisher("sphero_0/cmd_vel", Twist, queue_size=10)

        self.speed_class = CRATS.position_and_speed()
        self.sphero_control = sphero_ctl.SpheroControl(self.sphero_pub)

        self.img=None
        #cv2.startWindowThread()
        #cv2.namedWindow('drone image')
        rospy.Rate(25)
        t = Twist()
        t.angular.y = -90
        current_time = rospy.get_time()
        while rospy.get_time()-current_time<5.:
            self.camera_pub.publish(t)
            
        self.camera_sub = rospy.Subscriber("/bebop/image_raw", Image, self.get_image)
        self.mode = True
        rospy.spin()

    def get_image(self, img_msg):

        
        imageFrame=self.__cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8") # Needs to be checked
        
        

        cv2.imshow('drone image', imageFrame)
        cv2.waitKey(40)
        '''
        pts1 = np.array(0)
        while pts1.shape !=(4,2):
            try:
                pts1 = testing.izracunaj_koordinate_vrhova(imageFrame)
               
            except Exception as e:
                print(e)
                return

             
            pts1 = np.array(pts1)
            #plt.imshow(imageFrame)
            #plt.show()
            
        pts1 = np.array(pts1)
        #print(f"Points to transform: {pts1}")
        pts2 = np.float32([[0, 0], [0, 650], [650, 0],  [650, 650] ])
        M = cv2.getPerspectiveTransform(pts1.astype(np.float32),pts2)

        new_img = cv2.warpPerspective(imageFrame, M, (650, 650))
        new_img = cv2.flip(new_img, 1)
        cv2.imshow("transformirano", new_img)

        
        grayImage = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage,90, 255, cv2.THRESH_BINARY)
        maze_image = new_img
        edges = cv2.Canny(blackAndWhiteImage,0, 255,apertureSize = 3)
        minLineLength = 100
        maxLineGap = 5
        lines = cv2.HoughLinesP(edges,1,np.pi/180,10, minLineLength, maxLineGap)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(maze_image, (x1, y1), (x2, y2), (0, 0, 0), 4)
        #plt.figure(figsize=(7,7))
        #plt.imshow(blackAndWhiteImage) # show the image on the screen 
        #plt.show()
        '''
        # TODO Maze solving
        #simplified_maze = ms.simplify(blackAndWhiteImage)
        if self.mode:
            error = 100
            error =self.sphero_control.run(0.5, imageFrame)
            if abs(error) < 0.1:
                self.mode = False
        else:
            zero = self.sphero_control.run(0.0, imageFrame)
        



    
  
if __name__ == "__main__":
    x = DroneControl()
    