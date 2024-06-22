# Overview

Here I will plan the technical aspects to ensure readiness and success for on-site integration and testing.

# Review of Goals from README

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

# Brainstorming

## Design Questions
 - Will I run compute on the same machine as the control? 
   - If I train on my laptop, it may be easier to use an ethernet communication interface, simply using the arduino as an intermediary.
 - How will I train? Is there a sample and time efficient enough algorithm to run in real-time, or is pretraining necessary?
 - If I simulate the environment, how will I ensure generalization to real life? 
   - Should I parameterize on intrinsic properties, such as mass of rod, total longitudinal distance, and maximum movement speeds
   - Will training on all of these parameters be prohibited on my local machine (3080ti)?
 - Simulation environment?
   - Python Gym
   - Gazebo (use ROS)?
   - Unreal Engine?
 - How should I format separation of training and inference? Presumably have a real-time inference program, potentially in a faster language such as C++.
   - This implies saving of model weights and loading in C++. Are there C++ libraries for this? (PyTorch C++ API)

## Predicted Issues

 - The encoder readings may not translate to the trained-upon data.
   - Compute a mapping function given some benchmark with an expectation (pendulum oscillations under gravity). Furthermore, it may be an interesting idea to use benchmarks like this as parameters (time for the pendulum to come to a stop given a 30 degree initialized unassisted swing (aka representation of system friction) - may differ if platform can and can't move).
 - The response of a given control signal will vary, in moments as well as duration, magnitude, and oscillatory behavior. 
   - Similar approach to encoder using a mapping. Perhaps also run some tests to see if we can replicate our trained-upon data?
 - Similar to my prior attempts, there will be an issue with the fact that I am currently dead-reckoning longitudinal position.

## Ideas

 - Redesign to use a camera for inputs
   - This will remove the need to use an encoder, and allow for determination of longitudinal displacement. Furthermore, it will allow me to try a double pendulum regardless of not having another encoder.
   - However, it will be challenging to ensure generalization. I may need to simulate the environment with a variety of backgrounds, and a particularly colored rod for this to work well. I could also use a colored sheet as the background, but that feels like cheating.
 - Set up a cloud-based GPU training device that can take an image of the environment. This will allow significant speed-ups due to larger batch sizes, and data and model parallelism capability. Therefore, it will be more realistic to retrain within an hour or two. That being said, in my experience, it's unlikely (if the code is not buggy) a task like this will require more than 24 hours of training, which is technically doable locally in the time frame allotted.
 - Modularize the system. Have a few interchangeable components for each module. An example of this would be having a module reponsible for determining the absolute angle of the pendulum. Then, I can build an image-based solution, an encoder-based solution, a naive and model-based approach for both, etc, and trade out these components to get it working.
   - I like this modular idea, because it allows me to continue progressing on this until the day I leave, without ever leaving much work unfinished.


## Advice

 - List out all unknowns.
 - Don't overparametrize model. If modularization is performed, use best guesses for other dynamics and values.
 - Know what tools I am familiar with, what they can do, and plan out every tool I will use.