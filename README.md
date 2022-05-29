# 2022 Robothon Setup Guide
Team UTS InterpolatURs
Members:
- Rohan Patel
- Tan Huynh
- Jon McLean
- Lam Le


## Clone Repository
#### You can clone this repository everywhere
1. Clone the robotics-robothon repository:
```
gh repo clone UTS-InterpolatURs/robotics-robothon
```
#### You need to clone these repositories in the robothon_ws
2. Go to the workspace: ``cd /robotics-robothon/robothon-ws/`` and clone the taskboard detector packages: 
```
gh repo clone UTS-InterpolatURs/task_detector
```
```
gh repo clone UTS-InterpolatURs/edge_detector
```
## Install Dependencies
1. Install dependencies packages:
```
rosdep install --from-paths src --ignore-src -r -y
```
2. Make the package: `catkin_make` or `catkin_build`
## Initialise ROS nodes
1. Launch RealSense node:
```
roslaunch realsense_camera rs_camera.launch align_depth:=true
```
2. Launch Robot node:
```
roslaunch uts_robothon robot.launch robot_ip:={your robot ip}
```
3. Launch Gripper node:
```
roslaunch uts_robothon gripper.launch
```
4. Launch Robothon node:
```
roslaunch uts_robothon robothon.launch
```
## Initialise MATLAB node
Open `MAIN.m` file in `robotics-robothon/MATLAB/Robothon Task Scripts` folder, and RUN the code, this will initialise the MATLAB node
