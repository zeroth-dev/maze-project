import numpy as np
import cv2
from math import pow, sqrt
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt


#dajes mu nutra boju, trackers objekt sa trackanim objektima, te index ovog objekta u trackersima!
class position_and_speed:
    def __init__(self, color):
        self.color = color
        self.tracker = cv2.TrackerCSRT_create()
        
        if color == 'green':
            self.lower = [(40,30,40),(0,0,0)]
            self.upper = [(70,255,255),(0,0,0)]
        elif color == 'blue':
            self.lower = [(100,100,60),(0,0,0)]
            self.upper = [(130, 255, 255),(0,0,0)]
        elif color == 'red':
            self.lower = [(0,100,100),(161,100,100)]
            self.upper = [(10,255,255),(179,255,255)]

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

    def get_sphero_speed(self, imageframe):
        imageframe_clear = imageframe.copy()
        self.frame += 1
        hsvframe = cv2.cvtColor(imageframe,cv2.COLOR_BGR2HSV)             #transfering image from BGR to HSV
        (success, box) = self.tracker.update(imageframe)


        lower1 = np.array(self.lower[0], dtype="uint8")             #defining first self.lower mask
        upper1 = np.array(self.upper[0], dtype="uint8")             #defining first higher mask
        lower2 = np.array(self.lower[1], dtype="uint8")             #defining second self.lower mask
        upper2 = np.array(self.upper[1], dtype="uint8")             #defining second higher mask
        mask1 = cv2.inRange(hsvframe, lower1, upper1)
        mask2 = cv2.inRange(hsvframe, lower2, upper2)
        mask = mask1 + mask2
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)

        #plt.imshow(mask)
        #plt.show()

        if self.first_pass == True:
            self.position[0] = box[0]
            self.position[1] = box[1]
            #print("Position = [" + str(self.position[0]) + "," + str(self.position[1]) + "]")
            self.sphero_location.x = box[0] + box[2]//2
            self.sphero_location.y = box[1] + box[3]//2
            if self.first_speed == False:
                #print("calcs---")
                #print(str(pow((self.position[0] - self.pastPosition[0]), 2)))
                #print(str(pow((self.position[1] - self.pastPosition[1]), 2)))
                self.speed = 20 * int(sqrt(pow((self.position[0] - self.pastPosition[0]), 2) + pow((self.position[1] - self.pastPosition[1]), 2)))
                self.speed_x = 20*(self.position[0] - self.pastPosition[0])
                self.speed_y = 20*(self.position[1] - self.pastPosition[1])
                print(self.color + " speed_x:" + str(self.speed_x/600))
                print(self.color + " speed_y:" + str(self.speed_y/600))
            self.pastPosition[0] = self.position[0] 
            self.pastPosition[1] = self.position[1]
            self.first_speed = False

    
        #print("PRINTING BOX ON SCREEN")
        (x, y, w, h) = box
        #cv2.circle(imageframe, (int(x)+(int(w)//2), int(y)+(int(h)//2)), int(w)//2, (0, 255, 0), 2)
        #cv2.imshow("trackers", imageframe)
        #cv2.waitKey(40)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if (len(cnts) > 0):
            c = max(cnts, key=cv2.contourArea)      
            ((x, y), radius) = cv2.minEnclosingCircle(c)                #defining a circle with a minimal radius around the found object
            if radius > 10:
            #    cv2.circle(imageframe, (int(x), int(y)), int(radius), self.colors[key], 2)      #drawing a circle round the found object
            #    cv2.line(imageframe, (320, 240), (int(x), int(y)), self.colors[key], 2)         #drawing a line from the center of the self.frame to the circle

                if self.first_pass == False:
                    a = int(x) - int(radius)
                    b = int(y) - int(radius)
                    c = 2 * int(radius)
                    d = 2 * int(radius)
                    box = (a, b, c, d)
                    ok = self.tracker.init(imageframe, box)
                    self.first_pass = True

        #output = cv2.bitwise_and(imageframe_clear, imageframe_clear, mask = self.mask_color['blue'])
        #mask_all = self.mask_color['red'] + self.mask_color['green'] + self.mask_color['blue']
        #output = cv2.bitwise_and(imageframe_clear, imageframe_clear, mask=mask_all)  # isolating preffered color from the image self.frame
        #plt.imshow(output)
        #plt.show()
        return (self.speed_x*0.5/300, self.speed_y*0.5/300)

