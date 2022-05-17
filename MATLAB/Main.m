%%
clear all; clf; clc;
addpath 'UR10e'
addpath 'ROSwrapper'
addpath 'Envi'
addpath 'ROSwrapper'
addpath 'VCam'

%%
% rosinit('');
hold on;
% Set initial robot base position:
robotBasePose = transl(0, 0, 0) * rpy2tr(0, 0, 0);
myUR10e = UTS_UR10(robotBasePose); % <-- Any changes to initial pose pls edit UTS_UR10.m
PlotFloor(); % <-- Plot the floor

P=[1.8,1.8,1.8,1.8;
-0.25,0.25,0.25,-0.25;
 1.25,1.25,0.75,0.75];

robotCam = VCam('robot', myUR10e, 'focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', ' ', ... 
    'fps', 30, 'depth', mean(P(1,:)));

% Created workbench model and plot it
wbPose = transl(0,-1.0,-0.423) * rpy2tr(0,0,0);
workbench = ModelGen('Envi/Workbench.ply', wbPose); 
workbench.PlotModel();
[wbVert,wbFace,wbFaceNorms] = workbench.GetModelVFNorm();

tbPose = transl(0,-1.0,0.026) * rpy2tr(0,0,0);
taskBoard = Taskboard(tbPose);
taskBoard.PlotTaskboard();

%% Move to ready pose
hold off
steps = 50;
qInit = myUR10e.model.getpos();
qHome = deg2rad([90 -60 90 -120 -90 90]);
MoveRobot.MoveOneRobot(myUR10e, robotCam, qInit, qHome, steps);

goalPose = transl(0,-0.8,0.7) * rpy2tr(pi,0,-pi/2);
MoveRobot.MoveRobotTV(myUR10e, robotCam, goalPose, steps);

%% Check Intersection
% % Get all joint transforms
% trJoints = IntColCompute.ComputeJointTransforms(myUR10e,qHome);
% 
% for i = 1 : size(trJoints,3)-1
%     for faceIdx = 1 : size(wbFace,1)
%         vertOnPlane = wbVert(wbFace(faceIdx,1)',:);
%         [intersect, check] = IntColCompute.LinePlaneIntersection(wbFaceNorms(faceIdx,:),vertOnPlane,trJoints(1:3,4,i)',trJoints(1:3,4,i+1)');
%         if check == 1 && IntColCompute.IsIntersectPointInsideTriangle(intersect, wbVert(wbFace(faceIdx,:)',:))
%             plot3(intersect(1),intersect(2),intersect(3),'g*');
%             disp('Intersection');
%         end
%     end
% end

 


