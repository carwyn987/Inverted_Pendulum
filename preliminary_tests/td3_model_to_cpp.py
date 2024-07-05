import numpy as np
import gymnasium as gym
import math
import sys
sys.path.append('preliminary_tests/models')
import td3 as TD3
import torch
import torchvision

policy_name = "TD3"
env_name = "CartPole-v1"
seed = 0
file_name = f"{policy_name}_{env_name}_{seed}"
max_action = 0.0075
action_dim = 1
state_dim = 4

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

policy.load(f"./models/best_td3/{file_name}")

example = torch.tensor(np.array([0.1, -0.1, 0.14, -0.132], dtype=np.float32))
# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
traced_script_module = torch.jit.trace(policy.actor.to("cpu"), example)

# Save the script module to a file
traced_script_module.save("models/policy.pt")

# script_policy = torch.jit.script(policy)
# script_policy.save("models/policy.pt")