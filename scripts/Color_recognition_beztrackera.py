import numpy as np
import cv2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
class Position:
        def __init__(self, color, pub=None):
            
            self.color=color
            if self.color == 'green':
                self.lower = [(50,50,50),(0,0,0)]
                self.upper = [(80,255,255),(0,0,0)]
                self.img_pub=pub
            elif self.color == 'blue':
                self.lower = [(100,70,60),(0,0,0)]
                self.upper = [(130, 255, 255),(0,0,0)]
            elif self.color == 'red':
                self.lower = [(0,70,50),(161,100,100)]
                self.upper = [(10,255,255),(179,255,255)]
            self.img_pub=pub
            self.colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0)}            #adding parameters for color circles
            self.mask_color = {}
            
            self.position = [0, 0]                  #Position [x,y]
            self.distance = 0                       #self.distance between enemy and friend
            self.pastPosition = [0, 0]              #enemy past coordinates
            self.speed = 0                          #speed in pixels per second
            self.speed_x = 0
            self.speed_y = 0
            self.frame = 0                          #self.frame counter
            self.first_pass = False
            self.first_speed = True

            self.sphero_location = Point()
        def get_sphero_position(self, imageFrame):
            imageFrame_clear = imageFrame.copy()
            self.frame += 1
            hsvFrame = cv2.cvtColor(imageFrame,cv2.COLOR_BGR2HSV)             #transfering image from BGR to HSV

        

            lower1 = np.array(self.lower[0], dtype="uint8")             #defining first lower mask
            upper1 = np.array(self.upper[0], dtype="uint8")             #defining first higher mask
            lower2 = np.array(self.lower[1], dtype="uint8")             #defining second lower mask
            upper2 = np.array(self.upper[1], dtype="uint8")             #defining second higher mask

            mask1 = cv2.inRange(hsvFrame, lower1, upper1)
            mask2 = cv2.inRange(hsvFrame, lower2, upper2)
            mask = mask1 + mask2
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))
            mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)
            self.mask_color = mask

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if (len(cnts) > 0):
                c = max(cnts, key=cv2.contourArea)      
                ((x, y), radius) = cv2.minEnclosingCircle(c)                #defining a circle with a minimal radius around the found object
                if radius > 10:
                    cv2.circle(imageFrame_clear, (int(x), int(y)), int(radius), self.colors[self.color], 2)      #drawing a circle round the found object
                    pass
                self.sphero_location.x = int(x)
                self.sphero_location.y = int(y)
                
            if self.color =='green':
                bridge = CvBridge()
                img_msg = bridge.cv2_to_imgmsg(imageFrame_clear, encoding="bgr8")
                self.img_pub.publish(img_msg)

            #mask_all = mask_color['red'] + mask_color['green'] + mask_color['blue']
            output = cv2.bitwise_and(imageFrame_clear, imageFrame_clear, mask=mask)  # isolating preffered color from the image frame

            #cv2.imshow("images", np.hstack([imageFrame, output]))       #only able to shut down using q
            #key = cv2.waitKey(40)
            
            return (self.sphero_location.x, self.sphero_location.y)
