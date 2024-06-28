"""
Set up environment and sockets to connect ROS control mechanism to Python
"""

import numpy as np
import gymnasium as gym
import math
import sys
import socket
import struct
import time
import copy

def setup_environment():
    """
    Creates an environment and returns env
    """

    env = gym.make('CartPole-v1', 
                gravity=9.80665,
                masscart=0.4,
                masspole=0.4,
                half_length=0.6*1.0/2,
                force_mag=10.0,
                tau=0.01,
                theta_threshold_radians = 100*math.pi/3,
                x_threshold=1,
                init_x=0.0,
                init_x_dot=0.0,
                init_theta= math.pi, # 0, # start in the upwards position
                init_theta_dot=0.0,
                perturbations=True,
                render_mode="human",
                screen_width=800,
                screen_height=400)
    # )

    return env

def run_simulation(env, server_socket, client_socket):
    """
    
    """
    state, done = env.reset(), False
    previous_state = state

    max_timesteps = 1000

    guess_state = math.pi * 20000

    for t in range(int(max_timesteps)):

            # Send state
            encoder_steps = int((state[2] - previous_state[2]) * 20000) # radians -> 360/2000 = 180/1000 = 0.18 degrees (?) # 2 is index of angle
            guess_state += encoder_steps
            send_data(client_socket, encoder_steps)

            print("Guess state: ", guess_state/20000.0, "Actual state: ", state[2])

            # action = np.array([0.8])
            # while action := socket_receive(action_connection, action_client_address) is None:
            #     # 1/2 ms delay
            #     time.sleep(0.0005)

            new_data = receive_data(client_socket)
            while new_data is None or new_data == 0.0:
                new_data = receive_data(client_socket)

            # print("Received data: ", new_data)
            action = np.array([new_data], dtype=np.float32)

            # Take an action
            next_state, reward, done, _, _ = env.step(action)

            # Update states
            previous_state = copy.deepcopy(state)
            state = next_state

            if done:
                state, done = env.reset(), False
                break

### SOCKETS

def setup_server(HOST='127.0.0.1', PORT=12345):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    client_socket, client_address = server_socket.accept()
    return server_socket, client_socket

def send_data(client_socket, data):
    if isinstance(data, float):
        packed_data = struct.pack('!f', data)
    elif isinstance(data, int):
        packed_data = struct.pack('!i', data)
    client_socket.sendall(packed_data)

def receive_data(client_socket):
    packed_data = client_socket.recv(4)
    unpacked_data = struct.unpack('<f', packed_data)[0]
    # unpacked_data = struct.unpack('!f', packed_data)[0]
    return unpacked_data

def close_server(server_socket, client_socket):
    client_socket.close()
    server_socket.close()


if __name__ == "__main__":
    env = setup_environment()
    server_socket, client_socket = setup_server(PORT=12354) # encoder data
    
    run_simulation(env, server_socket, client_socket)
    close_server(server_socket, client_socket)

    """
    server_socket, client_socket = setup_server()
    received_float = receive_data(client_socket)
    send_data(client_socket, received_float)
    received_int = receive_data(client_socket)
    send_data(client_socket, received_int)
    close_server(server_socket, client_socket)
    """