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
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist #, Point, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from RespawnGoal import Respawn  ### Done maybe check

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = PoseWithCovarianceStamped().pose.pose  ##Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getOdometry)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)#rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        # self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        # self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn() ## substitute
        self.vel_cmd = Twist()


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
        scanF_range = []
        scanB_range = []
        heading = self.heading
        min_range = 0.25 #### change to detect collision
        done = False

        for i in range(len(scanF.ranges)):
            if scanF.ranges[i] == float('Inf'):
                scanF_range.append(3.5)
            elif np.isnan(scanF.ranges[i]):
                scanF_range.append(0)
            else:
                scanF_range.append(scanF.ranges[i])

        for i in range(len(scanB.ranges)):
            if scanB.ranges[i] == float('Inf'):
                scanB_range.append(3.5)
            elif np.isnan(scanB.ranges[i]):
                scanB_range.append(0)
            else:
                scanB_range.append(scanB.ranges[i])

        obstacle_min_range = round(min(scanF_range), 2)
        obstacle_angle = np.argmin(scanF_range)

        vel_temp = self.vel_cmd


        if (min_range+0.2) > min(scanF_range) > 0:
            vel_temp.linear.x = vel_temp.linear.x - 0.3
            self.pub_cmd_vel.publish(vel_temp)
        else:
            vel_temp.linear.x = self.calculateVelocity()
            self.pub_cmd_vel.publish(vel_temp)

        if min_range > min(scanF_range) > 0 or (min_range-0.1) > min(scanB_range) > 0:
            done = True
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        if current_distance < 1:
            self.get_goalbox = True
        # rospy.loginfo('Goal Distance: %d', current_distance)
        return scanF_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-3]
        heading = state[-4]

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if done:
            rospy.loginfo("Collision!!")
            reward = -1000
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 400
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def calculateVelocity(self):
        dist_temp = (round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2))
        if (dist_temp > 5):
            dist_temp = 1.0
        vel_temp = dist_temp * 1.0 / 5
        return vel_temp

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        vel_temp = self.calculateVelocity()


        # vel_cmd = Twist()
        self.vel_cmd.linear.x = vel_temp
        self.vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(self.vel_cmd)

        dataF = None
        dataB = None
        while dataF is None:
            try:
                dataF = rospy.wait_for_message('/sick_front/scan', LaserScan, timeout=5)
            except:
                pass

        while dataB is None:
            try:
                dataB = rospy.wait_for_message('/sick_back/scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(dataF, dataB)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    def reset(self):
        # rospy.wait_for_service('gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        dataF = None
        dataB = None

        while dataF is None:
            try:
                dataF = rospy.wait_for_message('/sick_front/scan', LaserScan, timeout=5)
            except:
                pass

        while dataB is None:
            try:
                dataB = rospy.wait_for_message('/sick_back/scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(dataF, dataB)

        return np.asarray(state)