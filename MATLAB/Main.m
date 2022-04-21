%%
clear all; clf; clc;
addpath 'UR10e'
addpath 'Envi'

%%
% rosinit('');
hold on;
% myUR10e = MyUR10(["DabPrintNozzleTool.ply", "DabPrintNozzleToolParameters.mat"]);
myUR10e = UTS_UR10(); % <-- Any changes to initial pose pls edit UTS_UR10.m
myUR10e.PlotAndColourRobot();
workbench = Workbench();
workbench.PlotWorkbench();

%% Move to ready pose
steps = 50;
qInit = myUR10e.model.getpos();
qHome = deg2rad([0 -70 90 -110 -90 90]);
MoveRobot.MoveOneRobot(myUR10e, qInit, qHome, steps);

%% 
