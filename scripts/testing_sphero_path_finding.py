#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from matplotlib import pyplot as plt
import maze_solver as ms
import position as testing 
import Color_recognition_beztrackera as sphero_tracking
import sphero_ctl
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class sphero_test():
    def __init__(self):

        rospy.init_node("Sphero_test")
        self.sphero_pub = rospy.Publisher("/sphero_0/cmd_vel", Twist, queue_size=1)

        self.camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        self.dbg_pub = rospy.Publisher("dbg_img", Image, queue_size=1)
        self.position_class_green = sphero_tracking.Position('green', pub=self.dbg_pub)
        self.position_class_blue = sphero_tracking.Position('blue')
        self.position_class_red = sphero_tracking.Position('red')

        self.sphero_control = sphero_ctl.SpheroControl(self.sphero_pub)
        rospy.Rate(20)
        t = Twist()
        t.angular.y = -90
        current_time = rospy.get_time()
        while rospy.get_time()-current_time<5.:
            self.camera_pub.publish(t)
        

        self.drone_mode = 0
        self.past_error = 0
        self.camera_sub = rospy.Subscriber("/bebop/image_raw", Image, self.get_image,queue_size=1)
        self.mode = True
        self.first_pass = True
        self.path_iterator=0
        #cv2.namedWindow('test')
        self.stop = False
        self.debug_variable_x = 0.0
        self.debug_variable_y = 50.0
        self.first_time_segment = True
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
        pts2 = np.float32([[0, 0], [0, 649], [649, 0],  [649, 649] ])
        M = cv2.getPerspectiveTransform(pts1.astype(np.float32),pts2)

        new_img = cv2.warpPerspective(imageFrame, M, (650, 650))
        #new_img = cv2.flip(new_img, 1)
  
        # plt.imsave('Labyrinth_orig.png', imageFrame)
        location_blue_x, location_blue_y = self.position_class_blue.get_sphero_position(new_img)
        location_red_x, location_red_y = self.position_class_red.get_sphero_position(new_img)
        location_green_x, location_green_y = self.position_class_green.get_sphero_position(new_img)

        if self.first_pass == True:
            self.first_pass = False
            return
        if self.drone_mode == 0:
            no_sphero = testing.remove_sphero(new_img)
            grayImage = cv2.cvtColor(no_sphero, cv2.COLOR_BGR2GRAY)
            #plt.imshow(no_sphero)
            #plt.show()
        
            (thresh, blackAndWhiteImage) = cv2.threshold(grayImage,140, 255, cv2.THRESH_BINARY)
            maze_image = new_img
            edges = cv2.Canny(blackAndWhiteImage,0, 255,apertureSize = 3)
            minLineLength = 100
            maxLineGap = 5
            lines = cv2.HoughLinesP(edges,1,np.pi/180,10, minLineLength, maxLineGap)
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(maze_image, (x1, y1), (x2, y2), (0, 0, 0), 4)

            # TODO normal style
            

            #plt.imshow(blackAndWhiteImage)
            #plt.show()

            self.path=ms.find_path(blackAndWhiteImage, (location_blue_x, location_blue_y))
            self.drone_mode=1


        if self.debug_variable_x == 0:
            self.debug_variable_x = location_blue_x
        #print("Tracker output: ")
        #print("x: " + str(location_blue_x))
        #print("y: " +  str(location_blue_y))
        #print("")
        #print("path y: " + str(self.path[self.path_iterator][1]))
        #speed_red = self.speed_class_red.get_sphero_speed(imageFrame)
        if not self.stop:    
            error = 100
            error = self.sphero_control.run_y(self.path[self.path_iterator], (location_blue_x, location_blue_y), self.first_time_segment, [location_green_x, location_green_y], [location_red_x, location_red_y])
            self.first_time_segment = False
            if self.past_error == 0:
                self.past_error = error
            #error_2 = self.sphero_control.run_x(0, speed_blue_x)
            #print("error = " + str(error))
            
            if abs(error) < 15:
                self.first_time_segment = True
                self.path_iterator+=1
                if self.path_iterator == len(self.path):
                    self.stop = True
                self.past_error = 0
        end=rospy.Time.now().to_sec()
        print("time passed: " + str(end-begin))
        #print()
if __name__ == "__main__":
#    if rospy.is_shutdown("Sphero_test"):
#        rospy.signal_shutdown("Sphero_test")
    x = sphero_test()