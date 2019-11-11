#!/usr/bin/env python


import rospy
import numpy as np
import math
import random
import time ## for testing
from math import pi
from geometry_msgs.msg import Twist #, Point, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from RespawnNewGoal import Respawn  ### Done maybe check
# from scripts.mpo_700_actions import RobotActions


class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = PoseWithCovarianceStamped().pose.pose.position  ##Pose()
        self.r = rospy.Rate(10)

        # rospy.loginfo("Starting speed the environment")
        # time.sleep(4)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=2)

        # rospy.loginfo("Finishing speed the environment")

        # self.sub_odom = rospy.Subscriber("/odom", Odometry, self.getOdometry) ##Training Stage
        self.pub_init = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 10)
        self.sub_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getOdometry) ## Running Stage
        self.initPoint = PoseWithCovarianceStamped()
        # rospy.loginfo("Finishing publishers")
        # time.sleep(4)
        # self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        # self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn() #rospy.loginfo("State: %d", len(dataF.ranges))# substitute
        # rospy.loginfo("Reset World")
        # time.sleep(4)
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.0
        # time.sleep(4)
        self.pub_cmd_vel.publish(self.vel_cmd)
        # rospy.loginfo("Publishing 0 speed")

        # self.initPoint.pose.pose.position.x = 0.254
        # self.initPoint.pose.pose.position.y = -13.540357265
        # [x, y, z, w] = quaternion_from_euler(0.0, 0.0, 0.0)
        # self.initPoint.pose.pose.orientation.x = x
        # self.initPoint.pose.pose.orientation.y = y
        # self.initPoint.pose.pose.orientation.z = z
        # self.initPoint.pose.pose.orientation.w = w
        # self.robotActions = RobotActions()



    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)



        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)
        # rospy.loginfo('pose x: %d, y: %d', self.position.x, self.position.y)

    def getState(self, scanF, scanB): ### needs to modify it
        heading = self.heading
        min_range = 0.5 #### change to detect collision
        done = False
        crash = False

        obstacle_angle, minB_range, minF_range, scanF_range = self.getObstacles(scanF, scanB)


        if min_range > minF_range > 0:
            done = True
            crash = True
        if (min_range) > minB_range > 0:
            done = True
            crash = True
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        if current_distance < 1:
            self.get_goalbox = True
        # rospy.loginfo('Goal Distance: %d', current_distance)

        return scanF_range + [heading, current_distance, minF_range, obstacle_angle], done, crash

    def setReward(self, state, done, crash, action):
        yaw_reward = []
        goal = False
        current_distance = state[-3]
        heading = state[-4]

        for i in range(self.action_size):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        if heading <= 0.4 and heading >= -0.4:
            reward = 10 - (self.goal_distance - current_distance)
        else:
            distance_rate = 2 ** (current_distance / self.goal_distance)
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)


        # rospy.loginfo("Reward: %d", reward)

        if done:
            if crash:
                rospy.loginfo("Collision!!")
                reward -= 50
            else:
                reward -= 20
                rospy.loginfo("Done, not crash")

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward += 300
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False
            goal = True

        return reward, goal

    def calculateVelocity(self, obstacleDistance, action):
        max_angular_vel = 1.0
        min_range = 2
        dist_temp = (round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2))
        ang_vel = ((self.action_size - 1) / 2 - action) * max_angular_vel * 0.5

        if (self.heading < 0.5 and self.heading > -0.5):

            if (dist_temp > 5):
                dist_temp = 5
            vel_temp = dist_temp * 1.0 / 5

            if vel_temp <= 0.4:
                vel_temp = 0.5

            if obstacleDistance < min_range:
                vel_temp = (obstacleDistance)
                if obstacleDistance < 1:
                    vel_temp = 0.4-2*obstacleDistance


        else:

            if self.heading <= (pi/2) and self.heading >= (-pi/2):
                # rospy.loginfo("Angle: %d", self.heading)
                vel_temp = 0.4
                if ang_vel == 1:
                    ang_vel = max_angular_vel
                else:
                    if ang_vel == -1:
                        ang_vel = -max_angular_vel
                    else:
                        vel_temp = dist_temp * 1.0 / 5
            else:
                vel_temp = 0.0
                if ang_vel == 0:
                    ang_vel = max_angular_vel
                else:
                    if ang_vel == -1:
                        ang_vel = -max_angular_vel
                    else:
                        vel_temp = -0.6





        return vel_temp, ang_vel


    def readScanners(self):

        dataF = None
        dataB = None

        while dataF is None:
            try:
                dataF = rospy.wait_for_message('/sick_s300_front/scan', LaserScan, timeout=5)
            except:
                pass

        while dataB is None:
            try:
                dataB = rospy.wait_for_message('/sick_s300_back/scan', LaserScan, timeout=5)
            except:
                pass

        return dataF, dataB

    def getObstacles(self, dataF, dataB):
        scanF_range = []
        scanB_range = []
        numberOfReadings = round(len(dataF.ranges))

        auxCont = 0

        for i in range(len(dataF.ranges)):
            if (i % numberOfReadings == 0):
                auxCont += 1
                if(auxCont > 1):
                    if dataF.ranges[i] == float('Inf'):
                        scanF_range.append(3.5)
                    elif np.isnan(dataF.ranges[i]):
                        scanF_range.append(0)
                    else:
                        scanF_range.append(dataF.ranges[i])
        auxCont = 0
        for i in range(len(dataB.ranges)):
            if (i % numberOfReadings == 0):
                auxCont+=1
                if auxCont > 1:
                    if dataB.ranges[i] == float('Inf'):
                        scanB_range.append(3.5)
                    elif np.isnan(dataB.ranges[i]):
                        scanF_range.append(0)
                    else:
                        scanB_range.append(dataB.ranges[i])

        obstacle_min_rangeF = round(min(scanF_range), 2)
        obstacle_min_rangeB = round(min(scanB_range), 2)
        obstacle_angle = np.argmin(scanF_range)
        return obstacle_angle, obstacle_min_rangeB, obstacle_min_rangeF, scanF_range



    def step(self, action):
        min_range = 1.5
        dataF, dataB = self.readScanners()
        obstacle_angle, minB_range, minF_range, scanRangeF = self.getObstacles(dataF, dataB)

        vel_temp, ang_vel = self.calculateVelocity(minF_range, action)

        # vel_cmd = Twist()
        self.vel_cmd.linear.x = vel_temp
        self.vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(self.vel_cmd)


        state, done, crash = self.getState(dataF, dataB)

        if crash: ## comment if you want to reset after crash
            if min_range > minF_range > 0.4:
                self.vel_cmd.linear.x = -0.5
                if(obstacle_angle>14):
                    self.vel_cmd.angular.z = 0.7
                    self.pub_cmd_vel.publish(self.vel_cmd)
                    time.sleep(0.5)
                else:
                    self.vel_cmd.angular.z = -0.7
                    self.pub_cmd_vel.publish(self.vel_cmd)
                    time.sleep(0.5)



        reward, goal = self.setReward(state, done, crash, action)



        return np.asarray(state), reward, done, crash, goal

    def reset(self):

        self.vel_cmd.linear.x = 0
        self.vel_cmd.angular.z = 0
        self.pub_cmd_vel.publish(self.vel_cmd)
        # rospy.loginfo("First speed published")
        # rospy.wait_for_service('gazebo/reset_simulation')
        # rospy.wait_for_service('/gazebo/reset_world')
        # try:
        #     reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        #     reset_world()
        # except (rospy.ServiceException) as e:
        #     print("gazebo/reset_simulation service call failed")

        dataF, dataB = self.readScanners()

        # rospy.loginfo("Scanners Done")
        # self.pub_init.publish(self.initPoint)

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        # rospy.loginfo("Goal obtained")
        self.goal_distance = self.getGoalDistace()
        # rospy.loginfo("Calculated Distance")
        state, done, crash = self.getState(dataF, dataB)
        # rospy.loginfo("First state complete")

        return np.asarray(state)
