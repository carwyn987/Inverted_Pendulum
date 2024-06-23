"""
python preliminary_tests/sensitivity_analysis.py preliminary_tests/model_weights/latency_2_target_dqn.pth 2

or

python preliminary_tests/sensitivity_analysis.py models/best_td3/ 2
"""

import gymnasium as gym

import math
import torch
from models.dqn import ReplayBufferInvPend, AgentInvPend, Action_Value_Network
from tqdm import tqdm
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from collections import deque

def setup_env_with_params(masscart=0.4, 
                          masspole=0.4, 
                          half_length=0.6, 
                          force_mag=10.0, 
                          tau=0.01, 
                          x_threshold=1, 
                          render=False, 
                          init_x=0.0, 
                          init_x_dot=0.0, 
                          init_theta=math.pi, 
                          theta_threshold_radians=100*math.pi/3,
                          init_theta_dot=0.0, 
                          perturbations=True):
    full_env_dict = {
        'id': 'CartPole-v1',
        'gravity': 9.80665,
        'masscart': masscart,
        'masspole': masspole,
        'half_length': half_length,
        'force_mag': force_mag,
        'tau': tau,
        'theta_threshold_radians': theta_threshold_radians,
        'x_threshold': x_threshold,
        'init_x': init_x,
        'init_x_dot': init_x_dot,
        'init_theta': init_theta,
        'init_theta_dot': init_theta_dot,
        'perturbations': perturbations
    }
    if render:
        full_env_dict['render_mode'] = "human"
        full_env_dict['screen_width'] = 800
        full_env_dict['screen_height'] = 400

    env = gym.make(**full_env_dict)
    return env

def tester(model_weights_path, episodes=5, max_steps=1000, latency=0, env_params=None):
    env = setup_env_with_params(**env_params)
    
    max_steps = max_steps
    epsilon = 0.0
    buffer_size = 0

    actions = np.linspace(-1, 1, 21)
    num_actions = len(actions)
    observation_shapes = env.observation_space.shape[0]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # print("Using device: ", device)

    replay_buffer = ReplayBufferInvPend(buffer_size)
    agent = AgentInvPend(replay_buffer, env, latency=latency)
    if "policy" in model_weights_path or "td3" in model_weights_path.lower():
        from models import td3 as TD3
        policy_name = "TD3"
        env_name = "CartPole-v1"
        seed = 0
        file_name = f"{policy_name}_{env_name}_{seed}"
        max_action = 1.0
        state_dim = env.observation_space.shape[0]
        action_dim = 1

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
            behavior_model = TD3.TD3(**args)
        else:
            raise ValueError("Policy not recognized")

        behavior_model.load(os.path.join(model_weights_path, file_name))
        behavior_model.actor.eval()
        behavior_model.critic.eval()
        return run_td3(env, episodes, max_steps, behavior_model, max_action, latency)
    else:                                                                        
        behavior_model = Action_Value_Network(observation_shapes, num_actions).to(device)
        behavior_model.load_state_dict(torch.load(model_weights_path))
        behavior_model.eval()
        return run_dqn(episodes, max_steps, agent, behavior_model, epsilon, env, device)


def run_td3(env, episodes, max_steps, policy, max_action, latency):
     # Keep track of number of steps passed per episode
    steps_passed = []
    returns = []

    # action deque empty with max size
    action_deque = deque(maxlen=latency)

    state, done = env.reset(), False

    for ep in range(episodes):
        total_reward = 0
        for step in range(max_steps):
            
            action = (policy.select_action(np.array(state))).clip(-max_action, max_action).astype(np.float32)
            
            if latency > 0:
                action_deque.append(action)
                if len(action_deque) < latency:
                    action = np.array([0.0],dtype=np.float32)
                else:
                    action = action_deque.popleft()

            next_state, reward, done, _, _ = env.step(action)
            state = next_state
            total_reward += reward

            if done:
                break
        
        if not done:
            env.reset()

        steps_passed.append(step)
        returns.append(total_reward)
        env.reset()

    return {
        'steps_passed': steps_passed,
        'returns': returns
    }


def run_dqn(episodes, max_steps, agent, behavior_model, epsilon, env, device="cpu"):

    # Keep track of number of steps passed per episode
    steps_passed = []
    returns = []

    for ep in range(episodes):
        for step in range(max_steps):

            ep_done, tot_reward = agent.advance_state_and_log(behavior_model,epsilon,device=device)

            if ep_done:
                break
        
        if not ep_done:
            tot_reward = agent.total_reward
        
        steps_passed.append(step)
        returns.append(tot_reward)
        env.reset()
    
    return {
        'steps_passed': steps_passed,
        'returns': returns
    }

        

def coordinator(model_weights_path, latency_trained_on):

    max_steps = 1000
    num_episodes = 20

    params = {
        'masscart': 0.4, 
        'masspole': 0.4, 
        'half_length': 0.6,
        'force_mag': 10.0, 
        'tau': 0.01, 
        'x_threshold': 1,
        "latency": 2,
    }
    

    """
    mult.   1/4   1/2   1   2   4
    _____________________________
    param1|  0    0     0   0   0
    param2|  0    0     0   0   0
    param3|  0    0     0   0   0
    param4|  0    0     0   0   0
    ...

    All normalized.
    """
    heat_map = np.zeros((len(params), 5))
    param_names = []
    mult_names = ['1/4', '1/2', '1', '2', '4']
    pbar = tqdm(total=len(params)*len(mult_names))
    for i, param in enumerate(params):
        param_names.append(param + f"={params[param]}*mult") if param != 'latency' else param_names.append(param + f"=int({params[param]}*mult)")
        for j, mult in enumerate([1.0/4, 1.0/2, 1, 2, 4]):
            env_params = params.copy()
            env_params[param] *= mult
            tmp_latency = env_params['latency']
            env_params.pop('latency')
            results = tester(model_weights_path, episodes=num_episodes, max_steps=max_steps, latency=int(tmp_latency), env_params=env_params)
            heat_map[i][j] = np.mean(results['returns'])/max_steps
            pbar.update(1)
    
    heat_map[:,2] = np.mean(heat_map[:,2], axis=0)
    print(heat_map)

    # Normalization
    norm = Normalize(vmin=heat_map.min(), vmax=heat_map.max())
    heat_map_normalized = norm(heat_map)

    # Plotting
    plt.figure(figsize=(8, 6))
    plt.imshow(heat_map_normalized, cmap='RdYlGn')  # RdYlGn: red to green colormap
    plt.colorbar(label='Normalized Value')

    # Labels
    plt.xticks(range(len(mult_names)), mult_names)
    plt.yticks(range(len(param_names)), param_names)

    plt.title(f"Sensitivity Analysis \nModel={model_weights_path.split('/')[-1]}\nLatency_Trained_On={latency_trained_on}\n")
    plt.xlabel('Multiplier')
    plt.ylabel('Parameter')

    plt.show()

if __name__ == "__main__":
    model_weights_path = sys.argv[1]
    latency_of_model = sys.argv[2]
    coordinator(model_weights_path, latency_of_model)