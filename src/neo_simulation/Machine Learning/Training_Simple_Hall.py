#!/usr/bin/env python


import json
import os
import sys
import time

import numpy as np
import rospy

# from gym import wrappers
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float32MultiArray
from dqn_stage import ReinforceAgent
from scripts.calibratePosition import Calibration
# from scripts.mpo_700_actions import RobotActions
from environment_stage import Env
# from liveplot import LivePlot


EPISODES = 8000

def render(x):
    render_skip = 0
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)


if __name__ == '__main__':
    rospy.init_node('mpo_700_dqn_stage_1', anonymous=True)
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    state_size = 32
    action_size = 4

    calibrate = Calibration()
    # robotActions = RobotActions()
    env = Env(action_size)
    # time.sleep(4)

    # outdir = '/tmp/gazebo_gym_experiments'

    # env = wrappers.Monitor(env, outdir, force=True)
    # print "Monitor Wrapper started"

    # robotActions.resetWorld()

    agent = ReinforceAgent(state_size, action_size, True, 1)
    scores, episodes = [], []
    global_step = 0

    start_time = time.time()
    crash = False
    done = False
    env.reset()
    # calibrate.calibration()
    state = env.reset()

    for e in range(agent.load_episode + 1, EPISODES):

        done = False
        goal = False
        # calibrate.calibration() not required, only used for troubleshooting
        score = 0
        # state = env.reset()

        # env.render(e)
        for t in range(agent.episode_step):
            action = agent.getAction(state)

            next_state, reward, done, crash, goal = env.step(action)

            agent.appendMemory(state, action, reward, next_state, done)

            if len(agent.memory) >= agent.train_start:
                if global_step <= agent.target_update:
                    agent.trainModel()
                else:
                    agent.trainModel(True)

            score = score + reward
            state = next_state
            get_action.data = [action, score, reward]
            pub_get_action.publish(get_action)

            if t >250:
                rospy.loginfo("Time out.")
                done = True
                crash = False
                score -=100


            if e % 10 == 0:
                agent.model.save(agent.dirPath + str(e) + '.h5')
                with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
                    json.dump(param_dictionary, outfile)

            if goal:
                done = True

            if done:
                result.data = [score, np.max(agent.q_value)]
                pub_result.publish(result)
                agent.updateTargetModel()
                scores.append(score)
                episodes.append(e)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              e, score, len(agent.memory), agent.epsilon, h, m, s)
                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))
                if not crash:
                    state = env.reset()
                    rospy.loginfo("Reset")
                    break
                else:
                    # robotActions.moveBack()
                    crash = False
                    state = env.reset()
                    # rospy.loginfo("Reset")
                    rospy.loginfo("Crashed")
                    break


            global_step += 1
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
