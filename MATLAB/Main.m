%%
clear all; clf; clc;
addpath 'UR10e'
addpath 'ROSwrapper'
addpath 'Envi'
addpath 'ROSwrapper'

%%
% rosinit('');
hold on;
myUR10e = UTS_UR10(); % <-- Any changes to initial pose pls edit UTS_UR10.m
% Set initial robot base position:
% myUR10e.model.base = transl(0, 0, 0) * rpy2tr(0, 0, 0);
myUR10e.PlotAndColourRobot();
linksUR10e = myUR10e.model.links;
workbench = Workbench();
workbench.PlotWorkbench();
[wbVert,wbFace,wbFaceNorms] = workbench.GetModelVFNorm();

%% Move to ready pose
steps = 50;
qInit = myUR10e.model.getpos();
qHome = deg2rad([0 -70 90 -110 -90 90]);
MoveRobot.MoveOneRobot(myUR10e, qInit, qHome, steps);

%% Check Collision
trUR10 = zeros

