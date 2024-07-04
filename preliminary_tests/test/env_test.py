"""
Custom environment. I have downloaded the source code from git@github.com:Farama-Foundation/Gymnasium.git
and modified Gymnasium/gymnasium/envs/classic_control/cartpole.py to add kwargs for env parameters.

Added to __init__():
```
self.action_space = spaces.Box(low=np.array([-1]),
                                  high=np.array([1]),
                                  dtype=np.float32)
```
and parameters below.

Parameters added:
 - gravity: float = 9.8, 
 - masscart: float = 1.0, 
 - masspole: float = 0.1,
 - half_length: float = 0.5,
 - force_mag: float = 10.0,
 - tau: float = 0.02,
 - theta_threshold_radians: float = 12 * 2 * math.pi / 360, # 12 degrees
 - x_threshold: float = 2.4

I also made the action space continuous by changing the action space to be a Box space with a single float [-1,1] by:

Modifying Gymnasium/gymnasium/envs/classic_control/cartpole.py @ step() line 182 to move the cart by the action value:
force = self.force_mag * action.item()

To update environment:
```
pip uninstall Gymnasium
pip install Gymnasium/
```

"""

import gymnasium as gym
import math
import numpy as np

# Best guess at true environment parameters
env = gym.make('CartPole-v1', 
               gravity=9.80665,
               masscart=0.4,
               masspole=0.4,
               half_length=0.6,
               force_mag=10.0,
               tau=0.01,
               theta_threshold_radians = 100 * 2 * math.pi, # 100 full rotations
               x_threshold=1,
               init_x=0.0,
               init_x_dot=0.0,
               init_theta=math.pi,
               init_theta_dot=0.0,
               render_mode="human",
               screen_width=800,
               screen_height=400)

print(env.reset(seed=1))
print(env.action_space)

n = 10
def run(action, steps):
    for i in range(steps):
        action = np.array([action], dtype=np.float32).flatten()
        observation, reward, terminated, truncated, info = env.step(action)
        if not i%10: print(observation)

        if terminated or truncated:
            env.reset()

run(-1.0,n//2)
while True:
    run(1.0,n)
    run(-1.0,n)
