import numpy as np
import gymnasium as gym
import math

# Add file to the path
import sys

env = gym.make('CartPole-v1', 
               gravity=15,
               masscart=0.2,
               masspole=0.4,
               half_length=0.5,
               force_mag=10.0,
               tau=0.01,
               theta_threshold_radians = 5 * math.pi/2,
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

state, done = env.reset(), False

integral = 0
prev_error = 0

while True:

    # gravity = 9.81
    # length = 1
    # w = (state[5] - state[1]) / 0.04  # angular velocity
    # w_dir = (w/abs(w))
    # epsilon = 0.0
    # theta = state[5] % (2*math.pi)
    # # scale theta to keep centered
    # if state[4] < -0.1:
    #     theta += 0.05
    # elif state[4] > 0.1:
    #     theta -= 0.05
    # theta_dir = 0
    # if state[5] > 0 and state[5] < math.pi * 0.5:
    #     theta_dir = 1
    # elif state[5] > math.pi * 1.5 and state[5] < math.pi * 2:
    #     theta_dir = -1

    # desired_energy = gravity * length / 2.0
    # rotational_energy = 0.5 * (1.0 / 3.0) * length * length * w * w
    # potential_energy = gravity * (length / 2.0) * math.sin((math.pi / 2.0) - theta)
    # rotating_towards_top = 1 if (theta > 0 and theta < math.pi/2 and w < 0) or (theta <= 2*math.pi and theta > 1.5 * math.pi and w > 0) else -1

    # scale_by_energy_off = 1.0 / 2.0

    # if rotating_towards_top == 1:
    #     total_energy = potential_energy + rotational_energy
    #     if desired_energy - total_energy > epsilon: # we need more energy
    #         step = -1.0 * w_dir * (min(desired_energy - total_energy,2) * scale_by_energy_off)
    #     elif total_energy - desired_energy > epsilon: # remove energy
    #         step = 1.0 * w_dir * (min(total_energy - desired_energy,2) * scale_by_energy_off)
    #     else:
    #         step = 0.0
    # else:
    #     # rotate to negate w
    #     step = theta_dir

    # # if isinstance(step, np.ndarray):
    # #     step = step[0]

    # # print(rotating_towards_top, w, state, step)
    # print("Theta: ", theta * 360 / (2 * math.pi), ", Desired: ", desired_energy, ", Total: ", potential_energy + rotational_energy, ", Rotational: ", rotational_energy, ", Decision: ", step)

    p_c = 1.00
    d_c = 10.0
    i_c = 0.1

    error = 0.0 - state[5]
    if error < -math.pi:
        error += 2 * math.pi

    integral += error
    derivative = error - prev_error
    
    prev_error = error

    step = p_c * error + d_c * derivative + i_c * integral

    action = np.array([-1 * step], dtype=np.float32).clip(-1, 1)
    next_state, reward, done, _, _ = env.step(action)

    state = next_state

    if done:
        break
        env.reset()

# n = 30
# step = 0.01
# for i in range(n):
#         action = np.array([-1 * step], dtype=np.float32)
#         next_state, reward, done, _, _ = env.step(action)

#         state = next_state
#         print(next_state)
#         if done:
#             env.reset()
# for j in range(5):
#     for i in range(2*n):
#         action = np.array([1 * step], dtype=np.float32)
#         next_state, reward, done, _, _ = env.step(action)

#         state = next_state

#         if done:
#             env.reset()
    
#     for i in range(2*n):
#         action = np.array([-1 * step], dtype=np.float32)
#         next_state, reward, done, _, _ = env.step(action)

#         state = next_state

#         if done:
#             env.reset()