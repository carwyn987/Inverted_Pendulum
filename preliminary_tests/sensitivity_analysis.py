import gymnasium as gym
import math
import torch
from models.dqn import ReplayBufferInvPend, AgentInvPend, Action_Value_Network
from tqdm import tqdm
import numpy as np
import keyboard

env = gym.make('CartPole-v1', 
               gravity=9.80665,
               masscart=0.4,
               masspole=0.4,
               half_length=0.6,
               force_mag=10.0,
               tau=0.01,
               theta_threshold_radians = math.pi/3,
               x_threshold=1,
               init_x=0.0,
               init_x_dot=0.0,
               init_theta=0, # start in the upwards position
               init_theta_dot=np.random.uniform(-0.001, 0.001),
               render_mode="human",
               screen_width=800,
               screen_height=400)

max_steps = 2000
epsilon = 0.0
buffer_size = 0

actions = np.linspace(-1, 1, 21)
num_actions = len(actions)
observation_shapes = env.observation_space.shape[0]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device: ", device)

replay_buffer = ReplayBufferInvPend(buffer_size)
agent = AgentInvPend(replay_buffer, env)
behavior_model = Action_Value_Network(observation_shapes, num_actions).to(device)
behavior_model.load_state_dict(torch.load('preliminary_tests/model_weights/best_min_perturbations_target_dqn.pth'))
behavior_model.eval()

for step in tqdm(range(max_steps)):

    ep_done, tot_reward = agent.advance_state_and_log(behavior_model,epsilon,device=device)