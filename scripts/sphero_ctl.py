#!/usr/bin/env python3

import rospy
import cv2
#import zoomy
import numpy as np
from pid import PID
from geometry_msgs.msg import Twist
#import Color_recognition_and_tracking_speed_distance as CRATS
class SpheroControl:

    def __init__(self, velocity_pub):
        #rospy.init_node('SpheroControl2')
        self.velocity_pub = velocity_pub
        #self.sub = rospy.Subscriber('zoom_speed', Twist, self.run)
        self.white_width = 70
        self.black_width = 11
        self.distance_covered = 0.0
        #self.PAS = position_and_speed()
        self.pid_x = PID(0.55, 0, 0, 1, -1)
        self.pid_y = PID(0.55, 0, 0, 1, -1)
        self.old_time = rospy.get_time()
        #self.run(100)
        #rospy.Rate(20)
        #rospy.spin()
        self.firstPass=True

    def run_y(self, destination, current_position, first_Time, enemy_green_pos, enemy_red_pos,):
        
        velocity = Twist()
        
        # d = down - positive x axis
        # r = right - positive y axis
        # l = left - negative y axis
        # u = up - negative x axis

        """
            if direction != self.old_direction:
                self.old_direction = direction
        """    

        #self.is_path_clear(current_position, enemy_green_pos, enemy_red_pos, destination[2])

        velocity.linear.y = -120*self.pid_y.compute((destination[1])/650, current_position[1]/650)
        velocity.linear.x = 120*self.pid_x.compute((destination[0])/650, current_position[0]/650)
        
        
        if not self.is_path_clear(current_position, enemy_green_pos, enemy_red_pos, destination[2]):
            velocity.linear.x = 0
            velocity.linear.y = 0
        
        if destination[2] == 'u' or destination[2] == 'd':
            #velocity.linear.x = 0
            error = destination[1]-current_position[1]
        else:
            #velocity.linear.y = 0
            error = destination[0]-current_position[0]

        '''
        if first_Time == True:
            if destination[2] == 'u':
                velocity.linear.y=50
            elif destination[2] == 'd':
                velocity.linear.y=-50
            elif destination[2] == 'r':
                velocity.linear.x=50
            elif destination[2] == 'l':
                velocity.linear.x=-50
        '''     

        #print("sphero command y: {}".format(velocity.linear.y))
        #print("sphero command x: {}".format(velocity.linear.x))
        #print("Distance covered: {}".format(self.distance_covered))
        #print()


        """
            if direction == 'forward':                     
                velocity.linear.x = self.pid.compute(path_length, distance_covered)

            elif direction == 'right':
                velocity.linear.y = -self.pid.compute(path_length, distance_covered)

            elif direction == 'left':
                velocity.linear.y = self.pid.compute(path_length, distance_covered)

            elif direction == 'backward':
                velocity.linear.x = -self.pid.compute(path_length, distance_covered)
        """
        #print("current position y: " + str(current_position[0]))
        self.velocity_pub.publish(velocity)
        return error



    def is_path_clear(self, current_position, enemy_green_pos, enemy_red_pos, direction):
        clear = True
        if (direction == "u" or direction == "d"):
            if( (enemy_green_pos[0] < current_position[0] + 35 and enemy_green_pos[0] > current_position[0] - 35) or (enemy_red_pos[0] < current_position[0] + 35 and enemy_red_pos[0] > current_position[0] - 35) ):
                if direction == "u":
                    if ( (enemy_green_pos[1] < current_position[1] - 70 and enemy_green_pos[1] > current_position[1]) or (enemy_red_pos[1] < current_position[1] - 70 and enemy_red_pos[1] > current_position[1]) ):
                        clear = False
                if direction == "d":
                    if ( (enemy_green_pos[1] < current_position[1] + 70 and enemy_green_pos[1] > current_position[1]) or (enemy_red_pos[1] < current_position[1] + 70 and enemy_red_pos[1] > current_position[1]) ):
                        clear = False

        if (direction == "l" or direction == "r"):
            if( (enemy_green_pos[1] < current_position[1] + 35 and enemy_green_pos[1] > current_position[1] - 35) or (enemy_red_pos[1] < current_position[1] + 35 and enemy_red_pos[1] > current_position[1] - 35) ):
                if direction == "l":
                    if ( (enemy_green_pos[0] < current_position[0] - 70 and enemy_green_pos[0] > current_position[0]) or (enemy_red_pos[0] < current_position[0] - 70 and enemy_red_pos[0] > current_position[0]) ):
                        clear = False
                if direction == "r":
                    if ( (enemy_green_pos[0] < current_position[0] + 70 and enemy_green_pos[0] > current_position[0]) or (enemy_red_pos[0] < current_position[0] + 70 and enemy_red_pos[0] > current_position[0]) ):
                        clear = False
        print(clear)
        return clear

        """
            if direction == 'forward':                     
                velocity.linear.x = self.pid.compute(path_length, distance_covered)

            elif direction == 'right':
                velocity.linear.y = -self.pid.compute(path_length, distance_covered)

            elif direction == 'left':
                velocity.linear.y = self.pid.compute(path_length, distance_covered)

            elif direction == 'backward':
                velocity.linear.x = -self.pid.compute(path_length, distance_covered)
        """
        print("current position y: " + str(current_position[0]))
        self.velocity_pub.publish(velocity)
        return error


        #self.webcam.release()
    

if __name__ == "__main__":
    
    ne = SpheroControl(10000.0)