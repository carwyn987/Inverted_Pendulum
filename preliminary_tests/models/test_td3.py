import numpy as np
import torch
import gymnasium as gym
import argparse
import os
import math

# Add file to the path
import sys
sys.path.append('preliminary_tests/models')
import utils
import td3 as TD3


env = gym.make('CartPole-v1', 
               gravity=9.80665,
               masscart=0.4,
               masspole=0.4,
               half_length=0.6,
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

policy = "TD3"
env_name = "CartPole-v1"
seed = 0
file_name = f"{policy}_{env}_{seed}"
policy.load(f"./models/{file_name}")