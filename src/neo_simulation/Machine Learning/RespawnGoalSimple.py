#!/usr/bin/env python

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
        self.init_goal_x = -2.0
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
                define_goal = random.randrange(1, 3)/1.0

                if define_goal == 1:
                    goal_x = random.randrange(0, 5) / -1.0  ##variate depending on the map
                    goal_y = 0
                elif define_goal == 2:
                    goal_y = random.randrange(0, 9) / 1.0 ##variate depending on the map
                    goal_x = -5
                else:
                    goal_y = random.randrange(0, 9) / 1.0  ##variate depending on the map
                    goal_x = -5



                #
                # if (goal_x <=1 and goal_x >= -4) and (goal_y == -3 or goal_y == -4):
                #     goal_x = random.randrange(-4, 9) / 1.0  ##variate depending on the map
                #     goal_y = random.randrange(4, 7) / -1.0


                if abs(goal_x - self.obstacle_1[0]) <= 0.6 and abs(goal_y - self.obstacle_1[1]) <= 0.6:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.6 and abs(goal_y - self.obstacle_2[1]) <= 0.6:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.6 and abs(goal_y - self.obstacle_3[1]) <= 0.6:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.6 and abs(goal_y - self.obstacle_4[1]) <= 0.6:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.6 and abs(goal_y - 0.0) <= 0.6:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.pose.position.x = goal_x
                self.goal_position.pose.position.y = goal_y


        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.pose.position.x
        self.last_goal_y = self.goal_position.pose.position.y

        return self.goal_position.pose.position.x, self.goal_position.pose.position.y