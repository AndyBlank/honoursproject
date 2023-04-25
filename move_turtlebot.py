#!/usr/bin/env python3

import time
from itertools import count
import itertools
from typing import Counter
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import serial.tools.list_ports

global twist  
global command
global serialInst

def move_forward():
        global twist
        twist = Twist()
        twist.linear.x =1
        twist.linear.y=0
        twist.linear.z=0

        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=-0.05
def turn_right():
        global twist
        twist = Twist()
        twist.linear.x=0
        twist.linear.y=0

        twist.linear.z=0
        
        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=-0.8
def turn_left():
       
        global twist
        twist = Twist()
        twist.linear.x=0
        twist.linear.y=0
        twist.linear.z=0
        
        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=0.8

def turn_90_degrees(command):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
     #global twist
     #twist = Twist()
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle<float(1.98)):         #1.745
        if command == "right":
            turn_right()
        elif command == "left":
             turn_left()
        pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        current_angle = float(1*(t1-t0))
    move_stop()
    pub.publish(twist)
          
def move_distance():
     pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
     t0 = rospy.Time.now().to_sec()
     current_length = 0
     while(current_length< 2): #moves about 30 cm
           move_forward()
           pub.publish(twist)
           t1 = rospy.Time.now().to_sec()
           current_length = 1*(t1-t0)
     move_stop()
     pub.publish(twist)  


def move_stop():
        global twist
        twist = Twist()
        twist.linear.x=0
        twist.linear.y=0
        twist.linear.z=0

        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=0


def comm_with_arduino():
    ports = serial.tools.list_ports.comports()
    global serialInst 
    serialInst = serial.Serial()
    portsList=[]

    for onePort in ports:
        portsList.append(str(onePort))
        print(str(onePort))

    val = input("Select Port : ttyACM")

    for x in range(0,len(portsList)):
        if portsList[x].startswith("/dev/ttyACM"+str(val)):
            portVar = "/dev/ttyACM" + str(val)
            print(portVar)

    serialInst.baudrate = 9600
    serialInst.port = portVar
    serialInst.open()

    #while True:
      #  command = input("Arduino Command: UP/DOWN  or FW/BCK or OPN/CLS ")
       # serialInst.write(command.encode('utf-8'))

       # if command == 'exit':
       #      exit()


def move_from_coords(L,W):
    change_direction = "left"
    for index in range(0,L,2):
        move_distance()
        time.sleep(2)
        ###PLANTING OPERATION
        arduino_instruction("FW")
        arduino_instruction("DOWN")
        arduino_instruction("OPN")
        arduino_instruction("UP")
        arduino_instruction("CLS")
        if change_direction == "left":
             turn_90_degrees("left")
             time.sleep(2)
        elif change_direction =="right":
            turn_90_degrees("right")
            #publisher.publish(twist)
            time.sleep(2)
        for index2 in range (0,W,2):
             move_distance()
             time.sleep(2)
             ##PLANTING OPERATION
             arduino_instruction("FW")
             arduino_instruction("DOWN")
             arduino_instruction("OPN")
             arduino_instruction("UP")
             arduino_instruction("CLS")
             
        if change_direction =="right":
            turn_90_degrees("left")
            time.sleep(2)
            change_direction = "left"
        elif change_direction =="left":
             turn_90_degrees("right")
             time.sleep(2)
             change_direction= "right"

def arduino_instruction(command):
     serialInst.write(command.encode('utf-8'))
     time.sleep(4)



def main():
    global command  
    rospy.init_node("controller", anonymous=True)
    #pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    comm_with_arduino()
    length = int(input("Length of area:"))
    width = int(input("Width of area:"))
    time.sleep(2)
    move_from_coords(length,width)
    #while not rospy.is_shutdown():
    #    turn_90_degrees("left") 
    #    rospy.sleep(1)
    #    move_distance()
    #command = "DOWN"
    #serialInst.write(command.encode('utf-8'))
    
        
    #    rospy.sleep(4)
       
       
        
    rospy.Rate(500).sleep()



        
if __name__=="__main__":
       main()
    
       
