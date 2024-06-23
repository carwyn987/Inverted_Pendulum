# 06/23/24 Status Update

## Accomplishments thus far

TD3 Trained well locally in ~1 hour to swing up and balance pendulum.
Received part numbers of encoder and stepper motor components - need to write interfaces for those components.
 - Let me diagram out a system schematic so that I can code up interplay of components and draft specifics.

Sensitivity analysis complete.
 - Latency up to 4ms is handled by existing model without much loss in performance.
 - Maximum x threshold distance has huge impacts on performance, luckily our estimate should be realatively accurate, and we can simply add a scaling factor to compensate.
 - By far the most impactful parameter on evaluation performance is the rod length. This was expected, as the moment of inertia of a rod about its end is $I=\frac{1}{3}ML^{2}$ - though this doesn't match my intuition that mass would be the most impactful parameter. In fact, length is so important, halving or doubling it in our environment causes the model to fail to swing the rod up at all. Therefore, I will need to parameterize my model on this.

As one of my goals is for the swung-up rod to maintain a centered position in the x-axis, I also need to add reward for staying centered. This should improve robustness, as small perturbations, or failure to correctly estimate x position, will be less likely to push the cart into the stops at each end of the track.

I also have a good idea of the parameter size necessary for a policy to approximate swing-up and balance behavior. In the DQN, the model consisted of:
```
Action_Value_Network(
  (fc1): Linear(in_features=4, out_features=64, bias=True)
  (fc2): Linear(in_features=64, out_features=64, bias=True)
  (fc3): Linear(in_features=64, out_features=64, bias=True)
  (fc4): Linear(in_features=64, out_features=21, bias=True)
)
----------------------------------------------------------------
        Layer (type)               Output Shape         Param #
================================================================
            Linear-1                   [-1, 64]             320
            Linear-2                   [-1, 64]           4,160
            Linear-3                   [-1, 64]           4,160
            Linear-4                   [-1, 21]           1,365
================================================================
Total params: 10,005
Trainable params: 10,005
Non-trainable params: 0
----------------------------------------------------------------
Input size (MB): 0.00
Forward/backward pass size (MB): 0.00
Params size (MB): 0.04
Estimated Total Size (MB): 0.04
```

In TD3, we have:
```
Actor(
  (l1): Linear(in_features=4, out_features=256, bias=True)
  (l2): Linear(in_features=256, out_features=256, bias=True)
  (l3): Linear(in_features=256, out_features=1, bias=True)
)
Critic(
  (l1): Linear(in_features=5, out_features=256, bias=True)
  (l2): Linear(in_features=256, out_features=256, bias=True)
  (l3): Linear(in_features=256, out_features=1, bias=True)
  (l4): Linear(in_features=5, out_features=256, bias=True)
  (l5): Linear(in_features=256, out_features=256, bias=True)
  (l6): Linear(in_features=256, out_features=1, bias=True)
)
----------------------------------------------------------------
        Layer (type)               Output Shape         Param #
================================================================
            Linear-1                  [-1, 256]           1,280
            Linear-2                  [-1, 256]          65,792
            Linear-3                    [-1, 1]             257
================================================================
Total params: 67,329
Trainable params: 67,329
Non-trainable params: 0
----------------------------------------------------------------
Input size (MB): 0.00
Forward/backward pass size (MB): 0.00
Params size (MB): 0.26
Estimated Total Size (MB): 0.26
----------------------------------------------------------------
```

## What now?

 - Train another model on +1 parameter. As training time is almost certainly positively correlated with number of input parameters, and likely has a power law or exponential relationship, this should be setup on a cloud provider to enable scalability and speedups.
   - Currently using Gymnasium classic_control environment (not MuJoCo, Gazebo, or any other simulator / physics engine). Probably easiest to stick with this, but if time enables, I should try replicating environment, training, and simulation with Gazebo on AWS RoboMaker.
 - Review old code, diagram the system design, write adapters for encoder and stepper (alternatively, since I'm training another model, write environment adapters to simulate encoder readings, and to use stepper motor impulses to control to encapsulate the intermediate step in DNN for real-time execution).
 - I will also need to write a state estimation algorithm that maintains current estimates of x, theta, and derivatives for inference.
 - Remember to modularize (i.e. focus on communication medium and packet descriptions first) for ease of algorithm modifications.

## Notes on encoder and stepper motor

It definitely looks like I used a stepper driver in the initial design, but I can't make out which driver it is. This will have to wait until I'm on-site to develop further.

Almost all stepper drivers support **microstepping** between 1/8th and 1/256th.
Stepper motor specs:
 - 8.6V
 - 1A / coil
 - 200 steps / revolution = 1.8 degrees / step
 - Holding torque of 14 kg-cm

Encoder specs:
 - 2000 P/R = 0.18 degrees / "step"
