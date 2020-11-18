#!usr/bin/ python3
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import maze_solver as ms
class DroneControl:
    def __init__(self):
        self.__cv_bridge=CvBridge()
        rospy.init_node("Drone controller", anonymous=True)
        self.camera_sub = rospy.Subscriber("image_raw", Image, self.get_image)
        self.camera_pub = rospy.Publisher("camera_control", Twist, queue_size=10)
        self.img=None

    def get_image(self, img_msg):
        image=self.__cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough") # Needs to be checked
        self.img=image
    
    def run(self):
        path=ms.find_shortest_path(self.img, src, dist=width)