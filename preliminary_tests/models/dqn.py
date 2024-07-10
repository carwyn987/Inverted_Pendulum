from collections import namedtuple, deque
import numpy as np
import random
import torch
import torch.nn as nn
import torch.nn.functional as F

action_values = np.array([-1.0, -1.0/2, -1.0/4, -1.0/8, -1.0/16, -1.0/32, 0, 1.0/32, 1.0/16, 1.0/8, 1.0/4, 1.0/2, 1.0], dtype=np.float32)

def arrayIndexToActionValue(index):
    # return float(index-10)/10.0
    return action_values[index]

def actionValueToArrayIndex(val):
    # return int(val[0] * 10) + 10
    return np.argmin(np.abs(action_values - val))

SingleExperience = namedtuple('SingleExperience', 
                              field_names = ['state','action','reward','done','nextstate'])

class ReplayBufferInvPend:

    def __init__(self,size):
        self.buffer = deque(maxlen = size)
        
    def sampleBuf(self,size):
        el_inds = np.random.choice(len(self.buffer), size, replace=False)
        arr_chosen_samples = [self.buffer[i] for i in el_inds]
        state_arr, actions_arr, reward_arr, done_arr, next_state_arr = zip(*arr_chosen_samples)
        return np.array(state_arr,dtype=np.float32),np.array(actions_arr),np.array(reward_arr,dtype=np.float32),np.array(done_arr, dtype=np.uint8),np.array(next_state_arr,dtype=np.float32)

    def append(self, sample):
        self.buffer.append(sample)

    def size(self):
        return len(self.buffer)
    

class AgentInvPend:

    """
    Sets up an inverted pendulum agent to interact with the environment and store experiences in a replay buffer. Includes support for latency.

    Args:
    --------
     - buffer (ReplayBufferInvPend): The replay buffer to store experiences
     - environment (gym.Env): The environment to interact with
     - latency (int): The number of steps to wait before taking an action
    """
    def __init__(self, buffer, environment, latency=0):
        self.env = environment
        self.replay_buffer = buffer

        # Create a deque with max size n for our latency
        self.latency = latency
        if self.latency > 0:
            self.latent_SA_buffer = deque(maxlen=latency)
        elif self.latency < 0:
            raise ValueError("Latency must be greater than or equal to 0")

        self.restart_episode()

    def restart_episode(self):
        self.state = self.env.reset()
        self.total_reward = float(0)

    def choose_epsilon_greedy_action(self,model,epsilon,device="cpu"):
        if random.uniform(0,1) > epsilon:
            state_numpy_array = np.asarray([self.state],dtype="float32")
            state_tensor = torch.tensor(state_numpy_array).to(device)
            model_estimated_action_values = model(state_tensor)
            _, act_v = torch.max(model_estimated_action_values, dim=1) # This is the same as torch.argmax
            action = np.array([arrayIndexToActionValue(int(act_v.item()))])
        else:
            action = self.env.action_space.sample()
        return action
    
    # This function will advance the state of the environment, and log the results
    """
    Args:
    --------
     - model (nn.Module): The model to use to choose the action
     - epsilon (float): The probability of taking a random action
     - device (str): "cpu" or "cuda"
     - latency (int): The number of steps to wait before taking an action
    
    Returns:
    --------
     - done: bool
    """
    def advance_state_and_log(self, model, epsilon, device="cpu"):
        action = self.choose_epsilon_greedy_action(model,epsilon,device)
        if self.latency == 0:
            observation, reward, done, _, info = self.env.step(action.astype(np.float32)) # _ is for truncated
            
            observation = observation.astype("float32")
            action = actionValueToArrayIndex(action)
            sample_tuple = SingleExperience(self.state, action, reward, done, observation)
        else:
            self.latent_SA_buffer.append((self.state, action))
            # If deque is not full, take no action, else get latent state-action pair and execute
            if len(self.latent_SA_buffer) < self.latency:
                observation, reward, done, _, info = self.env.step(np.array([0.0],dtype=np.float32))
            else:
                observation, reward, done, _, info = self.env.step(self.latent_SA_buffer[0][1].astype(np.float32))
            
            sample_tuple = SingleExperience(self.latent_SA_buffer[0][0], actionValueToArrayIndex(self.latent_SA_buffer[0][1]), reward, done, observation.astype("float32"))
        
        self.total_reward += reward
        self.replay_buffer.append(sample_tuple)
        self.state = observation

        if done:
            t_reward = self.total_reward
            self.restart_episode()
            return (True, t_reward)
        return (False, None)
    

class Action_Value_Network(nn.Module):

    def __init__(self,input_size, output_size, hidden_size = 32):
        super(Action_Value_Network, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.fc4 = nn.Linear(hidden_size, output_size)

    def forward(self,x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x