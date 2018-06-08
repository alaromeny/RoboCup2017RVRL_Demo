#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from scipy.ndimage import gaussian_filter
import numpy as np
import math


number_of_robots = 3

stigmergyMap_width = 100
stigmergyMap_height = 100
x_transform = 50
y_transform = 50

localResolution_x = 2
localResolution_y = 2

x_transform, y_transform

stigmergyMap = np.zeros((stigmergyMap_width, stigmergyMap_height), dtype=np.uint8)

def getLocalArea( x_robot, y_robot):
    x = int(math.floor(x_robot))
    y = int(math.floor(y_robot))
    lower_x = x-localResolution_x
    upper_x = x+localResolution_x+1
    lower_y = y-localResolution_y
    upper_y = y+localResolution_y+1

    if upper_y >= stigmergyMap_height:
        upper_y = stigmergyMap_height -1
    if upper_x >= stigmergyMap_width:
        upper_x = stigmergyMap_width -1
    if lower_x < 0:
        lower_x = 0
    if lower_y < 0:
        lower_y = 0

    localStigmergyMap = stigmergyMap[lower_x:upper_x, lower_y:upper_y]
    return localStigmergyMap

def printLocalArea( x_robot, y_robot):
    localStigmergyMap = getLocalArea( x_robot, y_robot)
    print localStigmergyMap


def transformRoboPosToPheromoneMap( x_robot, y_robot):
    newX = x_robot + x_transform
    newY = y_robot + y_transform
    return newX, newY

def leaveTrail( x_robot, y_robot):
    x = int(math.floor(x_robot))
    y = int(math.floor(y_robot))
    stigmergyMap[x, y] = np.uint8(255)
    # print("CLEAN DATA: " + str(x) + " " + str(y))

def callback(data):
    rawX = data.pose.pose.position.x
    rawY = data.pose.pose.position.y

    mapX, mapY = transformRoboPosToPheromoneMap( rawX, rawY)
    leaveTrail( mapX, mapY)
    printLocalArea( mapX, mapY)

    # a = data.range


    # print("RAW DATA: \n")
    # print data.pose.pose.position

    # print("CLEAN DATA: " + str(mapX) + " " + str(mapY))

def callbackDummy(data):
    rawX = data.pose.pose.position.x
    rawY = data.pose.pose.position.y

    mapX, mapY = transformRoboPosToPheromoneMap( rawX, rawY)
    leaveTrail( mapX, mapY)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    j = 3
    robotName = 'robot' + str(j) + '/odom'

    rospy.Subscriber(robotName, Odometry, callback)

    for i in range(0,number_of_robots):
        robotName = 'robot' + str(i) + '/odom'
        rospy.Subscriber(robotName, Odometry, callbackDummy)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
