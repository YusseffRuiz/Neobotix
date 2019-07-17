#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np


x = 0.0
y = 0.0
theta = 0.0


def newPosition(msg):
    # global x
    # global y
    # global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    dataLaser = None

    while dataLaser is None:
        try:
            dataLaser = rospy.wait_for_message('/sick_front/scan', LaserScan, timeout=5)
        except:
            pass

    # readLaser(dataLaser)
    print(x, y, theta)


def readLaser(scan):
    scan_range = []
    min_range = 0.4  #### change to detect collision
    done = False

    for i in range(len(scan.ranges)):
        if scan.ranges[i] == float('Inf'):
            scan_range.append(3.5)
        elif np.isnan(scan.ranges[i]):
            scan_range.append(0)
        else:
            scan_range.append(scan.ranges[i])

    obstacle_min_range = round(min(scan_range), 2)
    obstacle_angle = np.argmin(scan_range)

    if min_range > min(scan_range) > 0:
        done = True
        print("Collision!")

    print ("Distance: ", min(scan_range), "Obstacle: ", obstacle_min_range, "Angle: ", obstacle_angle)


if __name__ == "__main__":
    rospy.init_node("get_position", anonymous=True)

    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, newPosition)



    # readLaser(dataLaser)


    rospy.spin()
