import numpy as np
import gymnasium as gym
import math

# Add file to the path
import sys

env = gym.make('CartPole-v1', 
               gravity=9.80665,
               masscart=0.4,
               masspole=0.4,
               half_length=0.6*1.0/2,
               force_mag=10.0,
               tau=0.01,
               theta_threshold_radians = 100*math.pi/3,
               x_threshold=1,
               init_x=0.0,
               init_x_dot=0.0,
               init_theta= math.pi, # 0, # start in the upwards position
               init_theta_dot=0.0,
               perturbations=True,
               render_mode="human",
               screen_width=800,
               screen_height=400)

state, done = env.reset(), False

n = 20
for i in range(n):
        action = np.array([-1], dtype=np.float32)
        next_state, reward, done, _, _ = env.step(action)

        state = next_state

        if done:
            env.reset()
for j in range(5):
    for i in range(2*n):
        action = np.array([1], dtype=np.float32)
        next_state, reward, done, _, _ = env.step(action)

        state = next_state

        if done:
            env.reset()
    
    for i in range(2*n):
        action = np.array([-1], dtype=np.float32)
        next_state, reward, done, _, _ = env.step(action)

        state = next_state

        if done:
            env.reset()