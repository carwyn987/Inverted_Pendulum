import numpy as np
import gymnasium as gym
import math

# Add file to the path
import sys
sys.path.append('preliminary_tests/models')
import td3 as TD3


env = gym.make('CartPole-v1', 
               gravity=13,
               masscart=0.2,
               masspole=0.4,
               half_length=0.5,
               force_mag=10.0,
               tau=0.01,
               theta_threshold_radians = 100*math.pi/3,
               x_threshold=1.0,
               init_x=0.0,
               init_x_dot=0.0,
               init_theta= math.pi, # 0, # start in the upwards position
               init_theta_dot=0.0,
               perturbations=True,
               render_mode="human",
               screen_width=800,
               screen_height=400)

policy_name = "TD3"
env_name = "CartPole-v1"
seed = 0
file_name = f"{policy_name}_{env_name}_{seed}"
max_action = 0.0075
max_timesteps = 1000
state_dim = env.observation_space.shape[0]
action_dim = 1
expl_noise = 0.0

if policy_name == "TD3":
    args = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": 0.99,
        "tau": 0.005,
        "policy_noise": 0.0,
        "noise_clip": 0.0,
        "policy_freq": 2,
        "policy_noise": 0.0 * max_action,
        "noise_clip": 0.0 * max_action,
        "policy_freq": 2
    }
    policy = TD3.TD3(**args)
else:
    raise ValueError("Policy not recognized")

policy.load(f"./models/{file_name}")

state, done = env.reset(), False

for t in range(int(max_timesteps)):
		
        action = (policy.select_action(np.array(state))).clip(-max_action, max_action).astype(np.float32)
        next_state, reward, done, _, _ = env.step(action)
        print(next_state[2])

        state = next_state

        if done:
            env.reset()