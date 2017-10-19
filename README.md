# Online Controller Adaptation:
## Matlab Toolboxes Needed:
- Model Predictive Control Toolbox
- Robotics Toolbox (To do experiments)
## Files:
### Matlab
- AHS_sim: simulates normal behavior of a robot navigating from a starting point to an end point in an occluded environment without incorporating any controller delays.
- AHS_sim_delay: Simulates robot's behavior in case controller delays are incurred. Delays are bounded by maxDelayFactor parameter in the code. Delays are multiples of sampling time and are distributed between sample time and sample time * maxDelayFactor.
- AHS_sim_conservative: Simulates robot's behavior in case we use a conservative controller that considers that maximum delay always happen at each time step.
- AHS_sim_WithAdaptation: Simulates robot's behavior in case we use our online adaptive controller.
- AHS_sim_WithAdaptation_Ros: Runs expirement with adaptive controller.
### ROS (CPP)
- acc.cpp, nh_drive.cpp: Transform acceleration commands from  AHS_sim_WithAdaptation_Ros to velocity commands in ROS.
