import sys
import gymnasium as gym
import math
import torch
from preliminary_tests.models.dqn import ReplayBufferInvPend, AgentInvPend, Action_Value_Network
from tqdm import tqdm
import numpy as np
import keyboard


def setup_and_run(model_weights_path):
    env = gym.make('CartPole-v1', 
                gravity=15,
                masscart=0.2,
                masspole=0.4,
                half_length=0.5,
                force_mag=10.0,
                tau=0.01,
                theta_threshold_radians = 100*math.pi/3,
                x_threshold=1.0,
                init_x=0.0,
                init_x_dot=0.0,
                init_theta= 0.0, # 0, # start in the upwards position
                init_theta_dot=0.0,
                perturbations=True,
                damping=0.994,
                render_mode="human",
                screen_width=800,
                screen_height=400)

    max_steps = 2000
    epsilon = 0.0
    buffer_size = 0

    actions = np.array([-1.0, -1.0/2, -1.0/4, -1.0/8, -1.0/16, -1.0/32, 0, 1.0/32, 1.0/16, 1.0/8, 1.0/4, 1.0/2, 1.0], dtype=np.float32)
    num_actions = len(actions)
    observation_shapes = env.observation_space.shape[0]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device: ", device)

    replay_buffer = ReplayBufferInvPend(buffer_size)
    agent = AgentInvPend(replay_buffer, env, latency=6)
    if "policy" in model_weights_path:
        behavior_model = Policy_Network(observation_shapes, num_actions, hidden_size=64).to(device)
    else:
        behavior_model = Action_Value_Network(observation_shapes, num_actions, hidden_size=64).to(device)
    behavior_model.load_state_dict(torch.load(model_weights_path))
    behavior_model.eval()

    for step in tqdm(range(max_steps)):

        ep_done, tot_reward = agent.advance_state_and_log(behavior_model,epsilon,device=device)

if __name__ == "__main__":
    # 'preliminary_tests/model_weights/latency_2_target_dqn.pth'
    setup_and_run(sys.argv[1])