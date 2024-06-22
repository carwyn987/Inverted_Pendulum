import gymnasium as gym
import math
import torch
from models.dqn import ReplayBufferInvPend, AgentInvPend, Action_Value_Network
from tqdm import tqdm
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

def setup_env_with_params(masscart=0.4, 
                          masspole=0.4, 
                          half_length=0.6, 
                          force_mag=10.0, 
                          tau=0.01, 
                          x_threshold=1, 
                          render=False, 
                          init_x=0.0, 
                          init_x_dot=0.0, 
                          init_theta=0, 
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
        'theta_threshold_radians': math.pi/3,
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
    behavior_model = Action_Value_Network(observation_shapes, num_actions).to(device)
    behavior_model.load_state_dict(torch.load(model_weights_path))
    behavior_model.eval()

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
    num_episodes = 200

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