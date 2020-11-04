# Render CartPole-v0 environment from OpenAI Gym to check if OpenAI Modules are installed correctly

import gym

env = gym.make('CartPole-v0')
env.reset()
for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
env.close()