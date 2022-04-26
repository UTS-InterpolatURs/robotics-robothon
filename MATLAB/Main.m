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
qHome = deg2rad([90 -30 90 -150 -90 90]);
MoveRobot.MoveOneRobot(myUR10e, qInit, qHome, steps);

%% Check Intersection
% Get all joint transforms
trUR10 = zeros(4,4,myUR10e.model.n+1);
trUR10(:,:,1) = myUR10e.model.base;
for i = 1 : myUR10e.model.n
    trUR10(:,:,i+1) = trUR10(:,:,i) * trotz(qHome(i)+linksUR10e(i).offset) * transl(0,0,linksUR10e(i).d) * transl(linksUR10e(i).a,0,0) * trotx(linksUR10e(i).alpha);
end

for i = 1 : size(trUR10,3)-1
    for faceIdx = 1 : size(wbFace,1)
        vertOnPlane = wbVert(wbFace(faceIdx,1)',:);
        [intersect, check] = IntColCompute.LinePlaneIntersection(wbFaceNorms(faceIdx,:),vertOnPlane,trUR10(1:3,4,i)',trUR10(1:3,4,i+1)');
        if check == 1 && IntColCompute.IsIntersectPointInsideTriangle(intersect, wbVert(wbFace(faceIdx,:)',:))
            plot3(intersect(1),intersect(2),intersect(3),'g*');
            disp('Intersection');
        end
    end
end


