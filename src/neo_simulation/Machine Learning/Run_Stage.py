#!/usr/bin/env python


import rospy
import rospkg
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float32MultiArray
from collections import deque
from keras.models import Sequential, load_model, model_from_json
from keras.optimizers import RMSprop
from keras.layers.core import Dense, Dropout, Activation
from environment_stage import Env
from scripts.calibratePosition import Calibration





class DQNSolver():
    def __init__(self, state_size, action_size, nameOfFile):
        self.result = Float32MultiArray()
        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=1000000)

        self.model = self.buildModel()
        self.load(nameOfFile)

    def buildModel(self):
        model = Sequential()
        dropout = 0.2

        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))

        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(dropout))

        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:

            q_value = self.model.predict(state.reshape(1, len(state)))
            self.q_value = q_value
            return np.argmax(q_value[0])



    def load(self, nameOfFile):
        # load previously created json and model

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("neo_simulation") ## Modify Path according to needs
        models_dir_path = pkg_path + "/save_model/Final"

        # pkg_path = 'neo_simulation/save_model/' ## Modify Path according to needs
        nameOfJson = os.path.join(models_dir_path, (nameOfFile + '.json'))
        nameOfmodel = os.path.join(models_dir_path, (nameOfFile + '.h5'))

        self.model.set_weights(load_model(nameOfmodel).get_weights())

        with open(nameOfJson) as outfile:
            param = json.load(outfile)
            self.epsilon = param.get('epsilon')
        print("Loaded model from disk")


    def run(self, env):
        rate = rospy.Rate(30)
        state = env.reset()
        # calibrate.calibration()

        while not rospy.is_shutdown():
            action = self.getAction(state)
            next_state, reward, done, crash, goal = env.step(action)
            state = next_state


if __name__ == '__main__':
    rospy.init_node('mpo_700_run_stage_1', anonymous=True)
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    state_size = 32
    action_size = 4
    newDQN = DQNSolver(state_size, action_size, 'stage_1_1')
    env = Env(action_size)
    # calibrate = Calibration()

    newDQN.run(env)
