#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import smbus
import RPi.GPIO as GPIO
import sys

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)
time.sleep(1)
        
# Set GPIO mode to BCM so it refferences GPIO pin number instead of PI-pin number
GPIO.setmode(GPIO.BCM)

#Victim LED
GPIO_LED = 24 
GPIO.setup(GPIO_LED,GPIO.OUT)

# Lighting up the RGB
LED = 26 
GPIO.setup(LED,GPIO.OUT)


LINEAR_VEL = 0.17 
ANGULAR_VEL = 2
STOP_DISTANCE = 0.27
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
SOFT_DISTANCE = 0.3 + LIDAR_ERROR
MEDIUM_DISTANCE = 0.25 + LIDAR_ERROR
HARD_DISTANCE = 0.2 + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
       # Filters the data
        samples = len(scan.ranges)  
        samples_view = 360          
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        # Eliminates bad bad scans
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            if scan_filter[i] < 0.01: #if an angle is not working - set it to 1 instead.
                scan_filter[i] = 1
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True
        obstacle_counter = 0
        global victim_counter
        victim_counter = 0
        collision_radius = 0.05

        # Function to quit the program
        def quit_program():
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            turtlebot_moving = False
            print("2 minutes passed - stopping\n")
            print("_______________________________")
            print("Obstacles hit:", obstacle_counter)
            print("Victims detected:", victim_counter)
            time.sleep(2)
            print("quitting..")
            sys.exit()
            
        
        # stops program after certain time
        runtime_start = time.time()


        # for victim counter
        start_time_victim = 10 

        while not rospy.is_shutdown():

            # makes sure the led for RGB is always on
            GPIO.output(LED, GPIO.HIGH)
            
            
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            # stops program er 2 minuts
            if((time.time() - runtime_start) >= 120):
                quit_program()


            # Obstacle in front - do a u-turn
            if min(lidar_distances[160:200]) < SAFE_STOP_DISTANCE / 1.5:
                twist.linear.x = LINEAR_VEL * 0.1
                twist.angular.z = ANGULAR_VEL
                self._cmd_pub.publish(twist)
                print("Obstacle in front - very close")
                in_front_close += 1
                
            
            # Obstacle to the left 
            elif min(lidar_distances[180:220]) < SAFE_STOP_DISTANCE:
                
                # Obstacle is very close - do a sharp turn
                if min(lidar_distances[180:220]) < SAFE_STOP_DISTANCE / 2:
                    twist.linear.x = 0
                    twist.angular.z =  - ANGULAR_VEL 
                    self._cmd_pub.publish(twist)
                    print("Obstacle on the left - very close")
                    right_close += 1
            
                else:
                    # Obstacle is less close - do a light turn
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = - ANGULAR_VEL * 0.5
                    self._cmd_pub.publish(twist)
                    print("Obstacle on the left - less close")
                    right += 1
            
            # Obstacle to the right
            elif min(lidar_distances[140:180]) < SAFE_STOP_DISTANCE:
                # Obstacle is very close - do a sharp turn
                if min(lidar_distances[140:180]) < SAFE_STOP_DISTANCE / 2:
                    twist.linear.x = 0
                    twist.angular.z =  ANGULAR_VEL 
                    self._cmd_pub.publish(twist)
                    print("Obstacle on the right - very close")
                    left_close += 1
            
                else:
                    # Obstacle is less close - do a light turn
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = ANGULAR_VEL * 0.5
                    self._cmd_pub.publish(twist)
                    print("Obstacle on the right - less close")
                    left += 1
                    
            else:
                # Move foward
                twist.linear.x = LINEAR_VEL 
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)
            
            # collision counter
            if (min_distance < collision_radius):
                if(time.time() - start_time_victim >= 10.0):
                    obstacle_counter += 1
                    start_time_obstacle = time.time()
                    print("collision detected")


            # defines the different colors
            data = bus.read_i2c_block_data(0x44, 0x09, 6) #Selects the right registers
            time.sleep(0.001)
            green = data[1] + data[0] / 256 # Calculates the levels of each color [0, 255]
            red = data[3] + data[2] / 256
            blue = data[5] + data[4] / 256
        
            # Determines the color based on which has the higher value
            color = ""
            
            # blue was too weak
            blue = blue * 1.2
            
            # green was too strong
            green = green * 0.8
            
            # makes sure no random objects gets mistaken for a victim
            thresholder_red = 100
            
            if green > red and green > blue: 
                color = "Green"
                print("green")
                GPIO.output(GPIO_LED, GPIO.LOW)

            elif blue > red and blue > green:
                color = "Blue"
                print("blue")
                GPIO.output(GPIO_LED, GPIO.LOW)

            elif red > thresholder_red and red > blue and red > green:
                color = "Red" 
                print("RGB(%d %d %d)" % (red, green, blue))
                #stops the robot and LED blinks when victim is detected
                if(time.time() - start_time_victim >= 5.0):
                    print("_________________")
                    print("!!! Victim Detected !!!")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False 
                    victim_counter = victim_counter + 1
                    start_time_victim = time.time()
                    #LED blinks
                    GPIO.output(GPIO_LED, GPIO.HIGH)
                    time.sleep(0.5)
                    GPIO.output(GPIO_LED, GPIO.LOW)
                    time.sleep(0.5)
                    GPIO.output(GPIO_LED, GPIO.HIGH)
                    time.sleep(0.5)
                    GPIO.output(GPIO_LED, GPIO.LOW)
            
            else:
                print(" ")

            print("RGB(%d %d %d)" % (red, green, blue))

        
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
