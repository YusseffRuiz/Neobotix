#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import Empty
import time



class RobotActions:

    def __init__(self):

        # Creates a node with name 'speed_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node("robotActions", anonymous=True)
        # Publisher which will publish to the topic '/cmd_vel'.

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # A subscriber to the topic '/amcl_pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.r = rospy.Rate(10)
        self.speed = Twist()


    def publish_vel(self, x, y, z):
        self.speed.linear.x = x
        self.speed.linear.y = y
        self.speed.angular.z = z
        self.velocity_publisher.publish(self.speed)


    def resetWorld(self):
        self.publish_vel(0.0, 0.0, 0.0)
        self.r.sleep()
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        print("Reset World")
        reset_world()
        rospy.spin

    def moveFront(self):
        self.publish_vel(0.8, 0.0, 0.0)

    def moveLeft(self):
        self.publish_vel(0.0, 0.8, 0.0)

    def moveRight(self):
        self.publish_vel(0.0, -0.8, 0.0)

    def moveBack(self):
        self.publish_vel(-0.8, 0.0, 0.0)

    def spinRight(self):
        self.publish_vel(0.0, 0.0, -0.7)

    def spinLeft(self):
        self.publish_vel(0.0, 0.0, 0.7)

    def stopMove(self):
        self.publish_vel(0.0, 0.0, 0.0)




if __name__ == "__main__":
    try:
        x = RobotActions()

        for y in range(7):
            if y == 1:
                x.moveFront()
            elif y == 2:
                x.moveLeft()
            elif y == 3:
                x.moveRight()
            elif y == 4:
                x.moveBack()
            elif y == 5:
                x.spinLeft()
            elif y == 6:
                x.spinRight()
            else:
                x.stopMove()
            print("move: ", y)
            time.sleep(2)

        x.spinRight()
        time.sleep(4)

        x.resetWorld()
    except rospy.ROSInterruptException:
        pass