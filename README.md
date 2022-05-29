# 2022 Robothon Setup Guide
Team UTS InterpolatURs
Members:
- Rohan Patel
- Tan Huynh
- Jon McLean
- Lam Le


## Clone Repository
Clone the robotics-robothon repository:
`gh repo clone UTS-InterpolatURs/robotics-robothon`
Clone the taskboard detector packages: 
`gh repo clone UTS-InterpolatURs/task_detector`
`gh repo clone UTS-InterpolatURs/edge_detector`
## Install Dependencies
1. Install dependencies packages:
2. Make the package: `catkin_make` or `catkin_build`
## Initialise MATLAB node
Open `MAIN.m` file in `robotics-robothon/MATLAB/Robothon Task Scripts` folder, and RUN the code, this will initialise the MATLAB node
## Initialise ROS nodes
1. Launch RealSense node: `roslaunch realsense_camera rs_camera.launch align_depth:=true`
2. Launch Robot node:
3. Launch Gripper node:
4. Launch Robothon node: 
