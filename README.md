*VIDEOS DOWN BELOW! (Consider this statement the hook)*

# Inverted-Pendulum Project

## What is an inverted pendulum?
An [inverted pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum) is a dynamic system with the center of mass above its pivot point, generally constrained to one rotational degree of freedom. While a pendulum is stable, inverting a pendulum requires active balancing applied to it. Generally, this is performed by applying torque to the pendulum directly, or shifting the base longitudinally.

## History of Project

### Most Recent Updates

DQN Model balances pendulum, but cannot converge from hanging (stable) initialization in a reasonable amount of time and resources.

TD3 trained to convergence on episodes of length 1000 steps (0.02s resolution = 20s). Video below. Note: there are perturbations in the test, which explain some unexpected movements. These represent external forces acting on the pole, or noise in the system.

https://github.com/carwyn987/Inverted_Pendulum/assets/28845732/e16f0db4-fafe-4622-8498-058ad09bbe2e

Reward function that worked well for TD3:
```
reward = math.cos(theta % (2*math.pi)) + 1
if theta > -1 * math.pi/24 and theta < math.pi/24:
    reward += 10
if reward < 0: reward = 0
```

### 2024

After a long intermission, I have decided to re-approach the project. However, I have an agenda in mind. As I will be traveling back to where the hardware is for one week only, I wish to simulate a solutions type of job, where the employee is dispatched to an on-site location, where they are responsible for quickly getting a system up and running. Therefore, I plan to prepare for this trip, and see how much I can minimize the on-site time necessary to get the pendulum balanced. Furthermore, I want to make the goals specific and as numerous as possible, so the goal is the following:

1. Enable the pendulum to balance "continuously". For testing purposes, I define this as "achieved" when the following test are completed successfully:
   - 5 individual tests, all 5 must pass first try with a unique codebase.
   - Sample an initial pendulum angle from vertical within the range of [-10,10] degrees.
   - Sample an initial longitudinal position within the middle 30% of the total range.
   - Set up the initial conditions by hand, and release contact with the pole as soon as movement is perceived after starting the program.
   - The pole must balance for 60 seconds for "success" to be achieved.
2. The same algorithm should enable the pole to swing up from a stationary position. I define this as "achieved" when the following test are completed successfully:
   - Begin with the pendulum stationary hanging at the lowest point (180 degrees from vertically upward (azimuth/heading/yaw(?)))
   - Execute the program, and without any contact from the tester, the pendulum should reach a balanced (varying <10 degrees from vertical over at least 10 seconds) within 30 seconds.
3. Just for funsies, a separate algorithm should enable the pendulum to make rotations of the pendulum. I define this as "achieved" when the following test are completed successfully:
   - Starting the test with the rod stationary in the downwards position, the rod should make >=10 full rotations in under 45 seconds.
4. Reach goal: if I achieve this, consider me a god (lowercase g). Modify the system somehow (make it visually based perhaps?) to enable a double pendulum to balance. This will involve hardware modifications to add another joint.

### 2018

In the naive implementation (2018), a deterministic model was created as a decision tree, with a few possible movements based from binning various encoder angles. It was during this year when the hardware component was constructed, and the results were presented in an independent research project class.

Unfortunately, the feedback loop was not responsive enough to maintain a balanced state, however the proof of concept was there.

During this phase, a few iterations of design were explored. Primarily, testing was performed on both a Raspberry PI and an Arguino Mega. There were moments of significant garbage collection, which justified use of the Arduino. Furthermore, the initial design of the hardware used a drawer slide and a servo, which was not an ideal combo due to the control of the servo, and the significant mass of the drawer slide, which contributed to a system with minimal ability to impart movement to the pendulum.


https://github.com/carwyn987/Inverted_Pendulum/assets/28845732/88b4a921-044d-4a91-877d-22acfd15797b

https://github.com/carwyn987/Inverted_Pendulum/assets/28845732/74fb59c1-3fed-446d-82f0-261624505b5c

These can also be viewed on [my youtube channel](https://www.youtube.com/channel/UCNOUx60VSDCHhShPJUi3yFg).
