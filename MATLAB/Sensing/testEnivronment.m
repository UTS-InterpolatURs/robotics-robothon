clc;
clear;
clf;
addpath '../UR10e/'
addpath '../'
      
robot = UTS_UR10(); 

centrePoints = zeros(robot.model.n+1,3);
radiis = zeros(robot.model.n+1,3);
radiis(1,:) = [0.10,0.10,0.3];
radiis(2,:) = [0.10,0.10,0.3];
radiis(3,:) = [0.5,0.10,0.10];
radiis(4,:) = [0.5,0.10,0.10];
radiis(5,:) = [0.10,0.10,0.10];
radiis(6,:) = [0.10,0.10,0.10];
radiis(7,:) = [0.10,0.10,0.15];
centrePoints(1,:) = [0,0,0];
centrePoints(2,:) = [0,0,0];
centrePoints(3,:) = [radiis(3,1)/2,0,radiis(3,3)];
centrePoints(4,:) = [radiis(3,1)/2,0,0];
centrePoints(5,:) = [0,0,0];
centrePoints(6,:) = [0,0,0];
centrePoints(7,:) = [0,0,0];
X = zeros(21,21,7);
Y = zeros(21,21,7);
Z = zeros(21,21,7);
for i = 1:robot.model.n+1
    [X(:,:,i),Y(:,:,i),Z(:,:,i)] = ellipsoid( centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radiis(i,1), radiis(i,2), radiis(i,3) );
end
for i = 1:robot.model.n+1
    robot.model.points{i} = [reshape(X(:,:,i),[],1),reshape(Y(:,:,i),[],1),reshape(Z(:,:,i),[],1)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});
    warning on;
end
q0 = [pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0];
robot.model.plot3d(q0);

axis equal
camlight