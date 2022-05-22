clc;
clear;
clf;
addpath '../UR10e/'
addpath '../'


robot = UTS_UR10(); 
obj = urVisualServoing(robot);
