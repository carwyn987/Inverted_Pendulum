# Inverted-Pendulum

This repository is an ongoing effort to salvage my old inverted pendulum control system and hardware and reimplement it with more modern and "smoother" system.

An inverted pendulum is a dynamic system with the objective of balancing a rod. Imagine balancing a ruler on your palm as an unconstrained example, and balancing a skateboard by holding one set of its wheels as a constrained example. In practical engineering systems, the rod is allowed to swivel upon one end, and constrained to rotating around one axis. The influence we apply to the system to get this behavior is moving the base of the rod in a perpendicular horizontal dimension. This project attempts to construct the best control system for this dynamics model with the objective of balancing the rod.

In the naive implementation (2018), a deterministic model was created as a decision tree, with a few possible movements based from binning various encoder angles.

However now (November 2022), I wish to return to this project and expand upon it using the skills I acquired in university (Stony Brook University, Undergrad and Graduate Computer Science).

One potential approach is to use a simple neural network (NN) as a classifier where:
 - The network classifies the direction and magnitude of change of the stepper motor
 - One input is the limit switch states (although hardcoding outside of NN is necessary)
 - The main input is the encoder state (tells us angle of pendulum rod)
 - Another potential input is the estimated (by dead reckoning) x position of the rod-carrying cart
