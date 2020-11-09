PX4 Direct Controller
=====================

Here be the ramblings of jelow as he tries to develop a full-stack pipeline for quadcopter trajectory planning. Be warned. There is a lot of text diarrhea. If it works out, this document will be polished into an actual proper tutorial.

Objective
=========
The idea here is to use PX4 to execute a full-stack pipeline for quadcopter trajectory planning. This requires two
repositories:

**ROS Workspace:** (This repository) We handle the high level trajectory planning here.

**Firmware:** (https://github.com/lowjunen/Firmware/tree/direct_motor) We handle the low level trajectory planning here.

To be precise, we actually need 3 considering we have a custom mavlink message. But the customised mavlink repo is already submoduled into the customised Firmware repo). 

Pipeline
========
The pipeline looks as follows:

![Pipeline](https://imgur.com/2Wzwax6.png)

Trajectory objectives would be like flying through a sequence of gates. Trajectory constraints would be like motor limits and obstacle avoidance. These are fed into a high level controller whose purpose is to simulate the entire trajectory and output the corresponding motor wrench commands to achieve it. Let us define the wrench command specific to the quadcopter to be the thrust (Fz) and the roll, pitch, yaw torques (Mx,My,Mz). A good example of such a high level controller would be the differential flatness based controller seen in the Minimum Snap Trajectory paper by Mellinger. Another example would be the iterative Linear Quadratic Regulator (iLQR).

It is important here that we keep things general. Whatever the constraints, objective or high level controller are, what we care about is only the 'traj' message that comes out of it. traj contains the terms we need to compute the wrench command online EXCEPT FOR the quadcopter's current state. In the case of the minimum snap controller that would be the flat output polynomial spline. In the case of iLQR it would be the feedback and feedforward matrices. One might wonder, why not just output the wrench command directly. Well... a problem that I am worried about is that this high level controller is likely to have an expensive compute; i.e it'll take too long to compute to meet online control requirements. Let's assume the actuator controller needs to run at 200Hz. It is simply not robust to believe that the high level controller can solve consistently at under 0.005s. So the workaround is simple... we update our high level solution as and when we can but at the low level end, we use our 'current best' solution. This is essentially what traj encodes. For that 0.005s, we lock our trajectory based on our current best prediction of the entire trajectory. This corresponds to an already computed polynomial spline or FF+FB matrix set. These are often functions of current state, which we can assume is reliably updated at a high rate. And so we compute the corresponding low level solution u = f(traj,x).

In PX4 Architecture
===================
So... where does this fit into the px4 architecture. To keep the rest of the features running (the safety checks, state estimation, mocap integration etc.) we do the following:

![PX4 Modified](https://imgur.com/OR8A2Dp.png)

Note that the high level controller can be separate. So write it in whatever flavour you fancy. What matters is that it has to publish a uORB message that contains the traj data. This is then fed into a low level controller using the existent Message Bus. We then replace the pos->att->rate control module sequence with our own custom module that does the necessary computation where it subscribes to traj and x to output u.

Implementation Thoughts
=======================
A few points on how I plan to implement the pipeline.

**1) Failsafe:** The ONLY failsafe response that I will be implementing for now is a complete kill. I do not plan to do any sophisticated recovery like a 'return to hover' until other aspects are ironed out... namely the fusion of position data to do a sort of geofence for flight tests.

**2) Separate Low-Level Submodule:** A submodule approach allows this type of controller to exist within the larger px4 repo. I feel that development of such low level controls would be beneficial to the research community and so I want to implement it in a way that can be supported in the long term. Thanks to the submodule architecture of the px4 flight software, this can be done pretty easily. We put the low level controller as a module of its own and expose the section that does the actual f(traj,x) compute. We introduce user-definable message types to be able to parse the traj data. And the rest (timing updates on which traj data to use etc) we implement for the user as these are general features that would be required of any low level + high level controller. Hence the generalization that the module should be doing u = f(traj,x). Customization of f() itself is for now through the above Firmware repo. At least until I figure out how to be more tidy about. I intend to push in two flavours of f(); a vanilla diff. flat and a specialized iLQR. 

**3) Dividing the Full Controller Into Two:** Another way of seeing this high-level + low-level pairing is as division of a single controller. In the first part, you simulate the entire trajectory. In the second part you compute the motor commands as a function of this simulated trajectory. The thing is... you don't necessarily have to divide it as so. You could say shorten the horizon of the simulated trajectory and run it on the low-level side. I imagine there might be benefits to that because we have less issues of 'communicating' the data between two computers AND your state estimator is always going to give the best estimate onboard (where the IMU is. yes mocap is offboard but we can more afford a slower position update that an attitude update. Just try flying a quadcopter with a pure mocap and no IMU). Ok anyway I digress. What I'm trying to get at is that whatever low-level controller is supposed to do... it has to do its stuff efficiently. So #1, that piece has to be coded in simple and efficient forms. #2 we need to kill unnecessary apps that are running on the px4. Given that we do away with the original pos->att->rate piece... we really don't need their apps in the drone. And neither do we need stuff like navigator while we're at that. Long story short... we need our own cmake build profile where we are especially careful with what apps we launch at boot. Something to figure out later.

Plan
====
Ok enough with my rambling on what I hope to reach in the long horizon. Here's where I try to flesh out what I have already done and plan to do.

**[DONE] Swap out the pos->att->rate controllers in firmware with the low level controller on mc builds:** For now, I subscribe to an existing uORB topic (rc_channels) just to test.

**[Kinda Done] Custom boot to clean up unnecessary apps:** I currently do this using a separate make file: px4_fmu_v4_direct (i think that's the name?) and then I use a file in the sd card to stop mc_rate_controller. I'll work on a proper config later on.

**[DONE] Define a custom uORB topic to be published from a companion computer:** For now, I am using the diff. flat spline as my default traj message form. I am able to reliably publish a message comprising of a timestamp and a float[20] array (corresponding to a single flat output frame) at 100Hz from a companion computer over to the PX4. Current implementation uses the standard mavlink protocol. Will eventually switch to FastRTPS (TBD).

**[Doing Now] Write out an interchangable piece of mc_dir_control for different flavours of low level control:** For now, it'll be the option of either diff. flat OR an empty LQR. The latter is to test interchangability.

**[Doing Now] Attempt to overload the trajectory_nominal to be able to transmit multiple high-level control formats:** This is because LQR will require different high-level data. Ideally we want it to be even more general (where the message can take on whatever size is needed).

**[TO DO] Test sequence for verifying hover along with the necessary parameter estimation (to generalize across different platforms)**

**[TO DO] Implement iLQR**

Notes
=====
1) Since mc_dir_control is currently 'developmental', I didn't bother with writing up a custom boot sequence for it in the Firmware repo. It instead piggybacks the default multicopter boot. I use a script in the sdcard (as per the tutorial in px4) to stop mc_rate_control and start mc_dir_control in its place.
