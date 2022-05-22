clc;
clear;
clf;
addpath '../UR10e/'
addpath '../'


robot = UTS_UR10(); 
robot.model.tool = robot.realSenseTf;
obj = urVisualServoing(robot);
