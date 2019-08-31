#! /usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class RobotActions:



    def __init__(self):

        # Creates a node with name 'speed_controller' and make sure it is a
        # unique node (using anonymous=True).
        # rospy.init_node("robotActions", anonymous=True)
        # Publisher which will publish to the topic '/cmd_vel'.
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getPosition)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # A subscriber to the topic '/amcl_pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.r = rospy.Rate(10)
        self.speed = Twist()
        self.myPose = Point()
        self.moveDistance = 1
        self.moveAngle = (2 * math.pi) / 36



    def getPosition(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y


        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.myPose.x = x
        self.myPose.y = y
        self.myPose.z = theta

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
        end_flag = False
        end_distance = self.myPose.x + self.moveDistance
        self.publish_vel(0.8, 0.0, 0.0)
        while not end_flag:
            if self.myPose.x >= end_distance:
                end_flag = True
        self.stopMove()

    def moveLeft(self):
        end_flag = False
        end_distance = self.myPose.y + self.moveDistance/2
        self.publish_vel(0.0, 0.8, 0.0)
        while not end_flag:
            if self.myPose.y >= end_distance:
                end_flag = True
        self.stopMove()

    def moveRight(self):
        end_flag = False
        end_distance = self.myPose.y - self.moveDistance/2
        self.publish_vel(0.0, -0.8, 0.0)
        while not end_flag:
            if self.myPose.y <= end_distance:
                end_flag = True
        self.stopMove()


    def moveBack(self):
        end_flag = False
        end_distance = self.myPose.x - self.moveDistance/2
        self.publish_vel(-0.8, 0.0, 0.0)
        while not end_flag:
            if self.myPose.x <= end_distance:
                end_flag = True
        self.stopMove()

    def spinRight(self):
        end_flag = False
        end_distance = self.myPose.z - self.moveAngle
        self.publish_vel(0.0, 0.0, -0.7)
        while not end_flag:
            if self.myPose.z <= end_distance:
                end_flag = True
        self.stopMove()

    def spinLeft(self):
        end_flag = False
        end_distance = self.myPose.z + self.moveAngle
        self.publish_vel(0.0, 0.0, 0.7)
        while not end_flag:
            if self.myPose.z >= end_distance:
                end_flag = True
        self.stopMove()

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

        time.sleep(4)

        x.resetWorld()
    except rospy.ROSInterruptException:
        pass