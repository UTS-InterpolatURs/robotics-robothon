clc;
clear;
addpath '../UR10e/'
addpath '../'

obj = urCollisionDetection(UTS_UR10);
% while true
%     obj = urCollisionDetection(UTS_UR10);
%     obj.plotPointCloud();
%     
% end