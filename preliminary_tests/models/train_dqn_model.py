import sys
import gymnasium as gym
import math
import numpy as np
import torch
import torch.optim as optim
from torch import nn
from collections import deque
import random
from preliminary_tests.models.dqn import ReplayBufferInvPend, AgentInvPend, Action_Value_Network
from tqdm import tqdm
import matplotlib.pyplot as plt

# Best guess at true environment parameters
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
			   damping=0.994
            #    render_mode="human",
            #    screen_width=800,
            #    screen_height=400)
)

def train(load_model):

    # Set up replay buffer parameters
    buffer_size = 10000
    batch_size = 32

    # Define learning rate and other learning parameters
    lr = 0.001
    gamma = 0.999
    sync_target_frequency = 2000
    num_episodes = 2000
    max_steps = 1000
    epsilon = 1
    decay = 0.99996
    min_epsilon = 0.02

    stop_if_avg_reward_is = 9000
    num_samples_early_stopping_avg = 10
    stop_if_instant_reward_is = np.inf

    actions = np.array([-1.0, -1.0/2, -1.0/4, -1.0/8, -1.0/16, -1.0/32, 0, 1.0/32, 1.0/16, 1.0/8, 1.0/4, 1.0/2, 1.0], dtype=np.float32)
    num_actions = len(actions)
    observation_shapes = env.observation_space.shape[0]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # 
    print("Using device: ", device)

    # Initialize networks
    behavior_model = Action_Value_Network(observation_shapes, num_actions, hidden_size=64).to(device)
    target_model = Action_Value_Network(observation_shapes, num_actions,hidden_size=64).to(device)

    if load_model:
        print("Loading model weights from file!")
        behavior_model.load_state_dict(torch.load('preliminary_tests/model_weights/latency_2_behavioral_dqn.pth'))
        target_model.load_state_dict(torch.load('preliminary_tests/model_weights/latency_2_target_dqn.pth'))

    # Define optimizer as Adam, with our learning rate
    optimizer = optim.Adam(behavior_model.parameters(), lr=lr)
    lossfn = nn.MSELoss()

    # Initialize the buffer and agent
    replay_buffer = ReplayBufferInvPend(buffer_size)
    agent = AgentInvPend(replay_buffer, env, latency=2)

    # Let's log the returns, and step in episode
    return_save = []
    loss_save = []
    total_steps = 0
    return_achieved = False

    for episode_it in tqdm(range(num_episodes)):

        for step in range(max_steps):
            total_steps += 1

            # Step forwards in the environment (includes choosing action and logging experience)
            ep_done, tot_reward = agent.advance_state_and_log(behavior_model,epsilon,device=device)

            # If we are on the final iteration and the episode didn't conclude, set total reward
            if step == max_steps - 1:
                tot_reward = agent.total_reward
                ep_done = True

            if ep_done:
                return_save.append(tot_reward)
                print(f"Episode {episode_it}, Step {step}, Total Reward: {tot_reward}")

            # Early stopping
            if len(return_save) > num_samples_early_stopping_avg and (np.average(return_save[-num_samples_early_stopping_avg:]) >= stop_if_avg_reward_is and return_save[-1] >= stop_if_avg_reward_is \
                or return_save[-1] >= stop_if_instant_reward_is):
                return_achieved=True
                break

            # Decay epsilon
            epsilon *= decay
            if epsilon < min_epsilon:
                epsilon = min_epsilon

            # Sync target network
            if total_steps % sync_target_frequency == 0:
                target_model.load_state_dict(behavior_model.state_dict())
                print("Updated target model @ " + str(total_steps) + " steps and " + str(epsilon) + " epsilon.")

            # if we have enough data in buffer
            if replay_buffer.size() > 2*batch_size:

                # First, sample from the replay buffer
                cur_state_arr, action_arr, reward_arr, done_arr, next_state_arr = replay_buffer.sampleBuf(batch_size)

                # Now we must tensorize the data
                cur_state_tensor = torch.tensor(cur_state_arr).to(device)
                action_tensor = torch.tensor(action_arr).to(device)
                reward_tensor = torch.tensor(reward_arr).to(device)
                done_tensor_mask = torch.ByteTensor(done_arr).to(device)
                next_state_tensor = torch.tensor(next_state_arr).to(device)

                # First pass the batch into the model
                beh_model_output_cur_state = behavior_model(cur_state_tensor)

                # Extract the Q-values for taken actions. Returns a 1d tensor of action values taken.
                estimated_taken_action_vals = beh_model_output_cur_state.gather(1, action_tensor.unsqueeze(-1)).squeeze(-1)

                # Calculate the maximum action-value for the next state
                max_next_action_value_tensor = target_model(next_state_tensor).max(1)[0]
                
                # Mask the done values such that reward is 0. This is VERY important for quick and stable learning.
                max_next_action_value_tensor[done_tensor_mask != 0] = float(-10.0)

                target_values = max_next_action_value_tensor.detach() * gamma + reward_tensor

                # Calculate loss
                loss = lossfn(estimated_taken_action_vals, target_values)

                if step % 100 == 0:
                    loss_save.append(loss.item())

                # Perform back propogation
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            if ep_done:
                agent.restart_episode()
                break

        if return_achieved:
            break

    print("Total steps: ", total_steps, ", meaning epsilon got down to: ", epsilon)

    # Save the models
    torch.save(behavior_model.state_dict(), 'models/dqn_catcher/behavioral_dqn.pth')
    torch.save(target_model.state_dict(), 'models/dqn_catcher/target_dqn.pth')

    plt.figure()
    plt.title('Episode vs Loss')
    plt.xlabel('Episode')
    plt.ylabel('Loss')
    plt.plot(range(len(loss_save)),loss_save)

    plt.figure()
    plt.title('Episode vs Return')
    plt.xlabel('Episode')
    plt.ylabel('Return')
    plt.plot(range(len(return_save)),return_save)

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        load_model = sys.argv[1] == "--load"
    else:
        load_model = False
    train(load_model)
