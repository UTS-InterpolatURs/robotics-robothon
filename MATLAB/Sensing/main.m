clc;
clear;
clf;
addpath '../UR10e/'
addpath '../'

obj = urCollisionDetection(UTS_UR10);

num = obj.getCollisionStatus();
disp(num);
% while true
%     obj = urCollisionDetection(UTS_UR10);
%     obj.plotPointCloud();
%     
% end