clc;
clear;
clf;
addpath '../UR10e/'
addpath '../Envi/'
steps = 50;
humanPose = transl(1,0.5,-0.323) * rpy2tr(pi/2,0,pi/2);
human = Environment('Workbench.ply',humanPose);

obj = TesturCollisionDetection(UTS_UR10);
% obj.drawEllipsoid();
% obj.robot.PlotAndColourRobot();
points = plotCube();
% human.PlotModel();
% [verts, face, facenorm] = human.GetModelVFNorm();
q0 = [pi, -pi / 4, pi / 4, -pi / 2, -pi / 2, 0];
q1 = [pi/2, -pi / 4, pi / 4, -pi / 2, -pi / 2, 0];
% q1 = deg2rad([90,-45,90,-135,-90,0]);
obj.robot.model.animate(q0);
% q2 = deg2rad([180,-45,90,-135,-90,0]);
qmatrix = jtraj(q0,q1,steps);
obj.setJointStates(qmatrix);
obj.setObstaclePoints(points);

qcheck = obj.checkCollision(qmatrix);

for i = 1:steps
    if qcheck(i) == 1
        break;
    end
    
    obj.robot.model.animate(qmatrix(i,:));                                    % Moving the robot near the brick
    drawnow();
    
    
end



function [cubePoints] = plotCube()
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];
% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
    ; cubePoints * rotz(pi/2)...
    ; cubePoints * rotz(pi) ...
    ; cubePoints * rotz(3*pi/2) ...
    ; cubePoints * roty(pi/2) ...
    ; cubePoints * roty(-pi/2)];
hold on
% Plot the cube's point cloud
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([-0.4,-1.5,0],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
end
% obj = urCollisionDetection(UTS_UR10);
%
% num = obj.getCollisionStatus();
% disp(num);
% % while true
% %     obj = urCollisionDetection(UTS_UR10);
% %     obj.plotPointCloud();
% %
% % end