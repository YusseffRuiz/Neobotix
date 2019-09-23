#!/usr/bin/env python


import rospy
import rospkg
import os
import json
import numpy as np
import random
import time
import sys

# from environment_stage import Env

from sensor_msgs.msg import LaserScan

dataF = None
dataB = None

def callback(msg):
    dataF = None
    dataB = None

    dataF = msg
    # print("min: ", max(dataF.ranges))

    obstacle_angle, minF_range, scanRangeF = getObstacles(dataF)
    print("Obstacle Distance front: ", minF_range, ", obstacle Angle: ", obstacle_angle, ", Matrix Size: ",len(scanRangeF))
    # time.sleep(1)

def getObstacles(dataF):
    scanF_range = []
    # scanB_range = []
    numberOfReadings = len(dataF.ranges)/28

    for i in range(len(dataF.ranges)):
        if(i%numberOfReadings == 0):
            if dataF.ranges[i] == float('Inf'):
                scanF_range.append(3.5)
            elif np.isnan(dataF.ranges[i]):
                scanF_range.append(0)
            else:
                scanF_range.append(dataF.ranges[i])


    # for i in range(len(dataB.ranges)):
    #     if(i%numberOfReadings == 0):
    #         if dataB.ranges[i] == float('Inf'):
    #             scanB_range.append(3.5)
    #         elif np.isnan(dataB.ranges[i]):
    #             scanF_range.append(0)
    #         else:
    #             scanB_range.append(dataB.ranges[i])
    possibleObstacles = []
    for i in range(len(scanF_range)):
        if scanF_range[i]>0.25 and scanF_range[i]<1:
            possibleObstacles.append(scanF_range[i])
        else:
            possibleObstacles.append(3.5)

    obstacle_min_rangeF = round(min(possibleObstacles), 2)
    # obstacle_min_rangeF = round(min(scanF_range), 2)
    obstacle_angle = np.argmin(scanF_range)
    return obstacle_angle, obstacle_min_rangeF, scanF_range


if __name__ == '__main__':
    rospy.init_node('mpo_700_get_Obstacles', anonymous=True)
    rospy.loginfo("Initializing node reading Scanners")

    sub = rospy.Subscriber('/sick_s300_front/scan', LaserScan, callback)
    rospy.spin()
    #
    # state_size = 32
    # action_size = 4
    # env = Env(action_size)


    # while not rospy.is_shutdown():
    #     # dataF, dataB = env.readScanners()
    #     # obstacle_angle, minB_range, minF_range, scanRangeF = env.getObstacles(dataF, dataB)
    #     # rospy.loginfo("Obstacle Distance front: %d, obstacle Angle: %d, Matrix Size: %d", minF_range, obstacle_angle, len(scanRangeF))
    #     # state, done, crash = env.getState(dataF, dataB)
    #     #
    #     # rospy.loginfo("Crash: %d", crash)
    #     time.sleep(1)