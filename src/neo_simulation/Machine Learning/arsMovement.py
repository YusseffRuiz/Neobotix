#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np


##setting the hyper parameters

class Hp():

    def __init__(self):
        self.nb_steps = 1000  ##number of times we are going to update our model
        self.episode_length = 500  ##maximum length of an episode (time the AI will walk on the field)
        self.learning_rate = 0.02  ##control how fast your AI is learning (alpha)
        self.nb_directions = 16  ##number of directions in each weight, chances to find the best solutions, but takes longer
        self.nb_best_directions = 16  ##should be equal or lower than nb_directions
        assert self.nb_best_directions <= self.nb_directions  ##just to make sure that best_directions is lower than nb_directions
        self.noise = 0.03
        self.seed = 1  ##fix the parameters of the environemt
        self.env_name = 'HalfCheetahBulletEnv-v0'  ##is the variable that we are going to connect to our AI, the name will come later with the AI.


### Normalizing the states
### a new class to normalize different states of the AI

##states are vectors describing what is happening. Containts coordinates of the virtual robot.

class Normalizer():

    def __init__(self, nb_inputs):  ##number of values on the vector, which will be the inputs of the perceptron
        self.n = np.zeros(nb_inputs)  ## set the counter of our normalizer variables
        self.mean = np.zeros(nb_inputs)  ## mean of each of the input variables
        ##in order to calculate the variance of the vector, we are going to use 2 variables
        self.mean_diff = np.zeros(nb_inputs)  ##numerator
        self.var = np.zeros(nb_inputs)

    ##each time we see a new state, we update the normalizer to calculate the mean and the variance.
    def observe(self, x):  ##x will be a new state
        ##n should be increased every time a new state is seen
        self.n += 1.
        last_mean = self.mean.copy()  ##mean we have just before this new state
        ##we can compute the mean, however we need to use another computation method. Online computation method: each time we get a new value, the new mean will be:
        self.mean += (x - self.mean) / self.n  ##this is a more better approach to the mean
        ##Variance is a little bit more complex:
        self.mean_diff += (x - last_mean) * (
                    x - self.mean)  ##the sum of the previous numerator + value of the state - the last mean times value of the state - new mean
        self.var = (self.mean_diff / self.n).clip(
            min=1e-2)  ##this never has to be zero and that why we use .clip(min=1e-2)!! Remember this tip.

    def normalize(self, inputs):
        obs_mean = self.mean
        obs_std = np.sqrt(self.var)
        return (inputs - obs_mean) / obs_std


## Building the AI

class PolicyAI():
    def __init__(self, input_size, output_size):  ##return several actions
        self.theta = np.zeros(
            (output_size, input_size))  ##vector of the different weights for the perceptron (rows, columns)

    ### evaluate function refers to step 5 of the proposed algorithm
    def evaluate(self, input, delta=None,
                 direction=None):  ##delta is the perturbation added to define the different directions, It will be defined as None because it could not have perturbations
        if direction is None:
            return self.theta.dot(input)  ## when None, it only has to return the input * weights
        elif direction == "positive":
            return (self.theta + hp.noise * delta).dot(input)
        else:
            return (self.theta - hp.noise * delta).dot(input)

    def sample_deltas(self):  ##returning small values following a gaussian distribution
        first_dimension = self.theta.shape[0]
        second_dimension = self.theta.shape[1]
        return [np.random.randn(first_dimension, second_dimension) for _ in range(hp.nb_directions)]
        # return [np.random.randn(*self.theta.shape) for _ in range(hp.nb_directions)]
        ##*self.theta.shape gives the exact dimensions of the theta vector, only appliable in python3

    #### Update function regers to the method of finite distance differentiations on step 7 of proposed algoritm, only the sum part
    def update(self, rollouts, sigma_r):
        ## rollouts: a list of several trippers [reward in positive, reward in negative, perturbation]
        step = np.zeros(self.theta.shape)
        ### finite distance differentiation
        for r_pos, r_neg, d in rollouts:
            step += (r_pos - r_neg) * d
        self.theta += hp.learning_rate / (hp.nb_best_directions * sigma_r) * step


### We need to create a new explore function to explore the policy on one direction and over one episode
def explore(env, normalizer, policy, direction=None, delta=None):
    state = env.reset()  ## first state when you reset the environment
    done = False  ## boolean True = End of the episode, False = not done
    num_plays = 0.  ## no action played yet!!
    sum_reward = 0  ## is how we are going to measure the reward, a value to select the best values (it can be sum or average)
    while not done and num_plays < hp.episode_length:  ## total number of actions in an episode, and as long as done != True
        normalizer.observe(state)
        state = normalizer.normalize(state)
        action = policy.evaluate(state, delta, direction)
        state, reward, done, _ = env.step(action)
        reward = max(min(reward, 1), -1)  ## good trick to set all big numbers to 1 and all low numbers to -1
        sum_reward += reward
        num_plays += 1
    return sum_reward


#### Training of the AI
def train(env, policy, normalizer, hp):
    ## usually you try different normalizers
    for step in range(hp.nb_steps):
        ### initialize rewards and sample of the delta values
        delta = policy.sample_deltas()  ## a list of (nb_directions) with the random perturbations normalized
        pos_r = [0] * hp.nb_directions
        neg_r = [0] * hp.nb_directions
        #### Lets be careful, negative rewards are only the rewards with the perturbances in the opposite directions, not negative values

        ### Update our positive rewards and explore the policy on one full episode
        ## each of the (nb_directions) explore the positive rewards

        for k in range(hp.nb_directions):
            pos_r[k] = explore(env, normalizer, policy, direction="positive", delta=delta[k])

        for k in range(hp.nb_directions):
            neg_r[k] = explore(env, normalizer, policy, direction="negative", delta=delta[k])

        ## we need to compose a new list with 32 values with all rewards and pcompute standard deviation of these rewards (sigma)
        ### this step is 3.1 of the paper
        all_rewards = np.array(pos_r + neg_r)  ## we have make a concate of the 2 arrays
        sigma_r = all_rewards.std()

        ## Sorting of directions according to the highest reward. Can be seen in step 6 of the proposed algorithm
        scores = {k: max(r_pos, r_neg) for k, (r_pos, r_neg) in enumerate(zip(pos_r, neg_r))}
        ## we create a dictionary with key 1-16 and the value will be the reward
        ## enumerate generate the integers of the zip function (gathering together positive rewars with negative rewards)
        ### sort dictionaries is easier
        order = sorted(scores.keys(), key=lambda x: scores[x], reverse=True)[
                :hp.nb_best_directions]  ##we want the keys sorted, so we put it as 1st value sorted by the maximum values
        ## lambda says that we are specifying a function with the score values of the keys,x will be the keys

        ### Step 7 of the update process, we need to prepare the rollouts to use in update function with the step from paper
        rollouts = [(pos_r[k], neg_r[k], delta[k]) for k in order]

        policy.update(rollouts, sigma_r)

        ## printing the final reward of the policy after the update
        ## keep track of the rewards after the updates

        ## if we use None direction and None perturbation in explore function, we can see how it acts at the end of the update
        reward_evaluation = explore(env, normalizer, policy)
        print('Step: ', step, ' Reward: ', reward_evaluation)


################# Running the main code ###################################

def mkdir(base, name):
    path = os.path.join(base, name)
    if not os.path.exists(path):
        os.makedirs(path)
    return path


work_dir = mkdir('exp', 'brs')
monitor_dir = mkdir(work_dir, 'monitor')

hp = Hp()  ### we are creating an Hp class object and all variables are already initialized
np.random.seed(hp.seed)

## connecting the environment with our AI
### Connect pybullet environment with gym, which makes interaction easier.
## We could use the pybullet library, however due to imcopatibility issues, we will use gym
env = gym.make(hp.env_name)

## check the videos on our AI's on the screen!! :o
env = wrappers.Monitor(env, monitor_dir, force=True)  ## Force training to not stop due to warnings

## Prepare out policy
nb_inputs = env.observation_space.shape[0]
nb_outputs = env.action_space.shape[0]
policy = PolicyAI(nb_inputs, nb_outputs)

## Prepare our normalizer
normalizer = Normalizer(nb_inputs)

### Start training
train(env, policy, normalizer, hp)