from collections import namedtuple, deque
import numpy as np
import random
import torch
import torch.nn as nn
import torch.nn.functional as F

def arrayIndexToActionValue(index):
    return float(index-10)/10.0

def actionValueToArrayIndex(val):
    return int(val[0] * 10) + 10

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

    def __init__(self, buffer, environment):
        self.env = environment
        self.replay_buffer = buffer
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
    def advance_state_and_log(self, model, epsilon, device="cpu"):
        action = self.choose_epsilon_greedy_action(model,epsilon,device)
        observation, reward, done, _, info = self.env.step(action.astype(np.float32)) # _ is for truncated
        self.total_reward += reward
        observation = observation.astype("float32")
        action = actionValueToArrayIndex(action)

        # Create a tuple of the experience for the replay buffer
        sample_tuple = SingleExperience(self.state, action, reward, done, observation)
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