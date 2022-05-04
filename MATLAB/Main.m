%%
clear all; clf; clc;
close all;
addpath 'UR10e'
addpath 'ROSwrapper'
addpath 'Envi'
addpath 'ROSwrapper'

%%
% rosinit('');
hold on;
% Set initial robot base position:
robotBasePose = transl(0, 0, 0) * rpy2tr(0, 0, 0);
myUR10e = UTS_UR10(robotBasePose); % <-- Any changes to initial pose pls edit UTS_UR10.m
PlotFloor(); % <-- Plot the floor

% Created workbench model and plot it
wbPose = transl(0,-1.2,-0.423) * rpy2tr(0,0,0);
workbench = Environment('Envi/Workbench.ply', wbPose); 
workbench.PlotModel();
[wbVert,wbFace,wbFaceNorms] = workbench.GetModelVFNorm();

%% Move to ready pose
steps = 50;
qInit = myUR10e.model.getpos();
qHome = deg2rad([90 -30 90 -150 -90 90]);
MoveRobot.MoveOneRobot(myUR10e, qInit, qHome, steps);

%% Check Intersection
% Get all joint transforms
trJoints = IntColCompute.ComputeJointTransforms(myUR10e,qHome);

for i = 1 : size(trJoints,3)-1
    for faceIdx = 1 : size(wbFace,1)
        vertOnPlane = wbVert(wbFace(faceIdx,1)',:);
        [intersect, check] = IntColCompute.LinePlaneIntersection(wbFaceNorms(faceIdx,:),vertOnPlane,trJoints(1:3,4,i)',trJoints(1:3,4,i+1)');
        if check == 1 && IntColCompute.IsIntersectPointInsideTriangle(intersect, wbVert(wbFace(faceIdx,:)',:))
            plot3(intersect(1),intersect(2),intersect(3),'g*');
            disp('Intersection');
        end
    end
end

 


