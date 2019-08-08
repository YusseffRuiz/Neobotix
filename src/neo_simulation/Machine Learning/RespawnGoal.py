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
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('neo_simulation/Machine Learning',
                                                'neobotix_gazebo/models/mpo_700/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_position = PoseStamped()
        self.init_goal_x = 2.0
        self.init_goal_y = 0.0
        self.goal_position.pose.position.x = self.init_goal_x
        self.goal_position.pose.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robots_name_space', self.goal_position.pose, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.pose.position.x, self.goal_position.pose.position.y)

                # rospy.wait_for_service('/gazebo/reset_world')
                # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
                # reset_world()
                # rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.x, self.goal_position.y)

                break
            else:
                pass




    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        if self.stage != 4:
            while position_check:
                goal_x = random.randrange(-9, 9) / 1.0
                goal_y = random.randrange(-9, 9) / 1.0

                if((goal_y == 9 and (goal_x>=4 or goal_x<=-4)) or (goal_y == 8 and (goal_x>=5 or goal_x<=-5)) or (goal_y == 7 and (goal_x>=6 or goal_x<=-6)) or (goal_y == 6 and (goal_x>=7 or goal_x<=-7)) or (goal_y == 5 and (goal_x>=8 or goal_x<=-8))) or ((goal_y == -9 and (goal_x>=4 or goal_x<=-4)) or (goal_y == -8 and (goal_x>=5 or goal_x<=-5)) or (goal_y == -7 and (goal_x>=6 or goal_x<=-6)) or (goal_y == -6 and (goal_x>=7 or goal_x<=-7)) or (goal_y == -5 and (goal_x>=8 or goal_x<=-8))):
                    goal_x = random.randrange(-3, 3) / 1.0
                    goal_y = random.randrange(-3, 3) / 1.0

                if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.pose.position.x = goal_x
                self.goal_position.pose.position.y = goal_y

        else:
            while position_check:
                goal_x_list = [2.0, 5.0, -4.0, 8.0, 8.0, 0.0, -6.0, -6.5, 0.5, -10.0, -11.0, 0.0, 10.0, -2.0]
                goal_y_list = [0.0, 1.0, -2.0, 4.0, 5.5, -2.0, -1.0, 1.1, -1.5, 5.0, 5.8, -1.0, 12.0, -0.8]

                self.index = random.randrange(0, 14)
                print(self.index, self.last_index)
                if self.last_index == self.index:
                    position_check = True
                else:
                    self.last_index = self.index
                    position_check = False

                self.goal_position.pose.position.x = goal_x_list[self.index]
                self.goal_position.pose.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.pose.position.x
        self.last_goal_y = self.goal_position.pose.position.y

        return self.goal_position.pose.position.x, self.goal_position.pose.position.y