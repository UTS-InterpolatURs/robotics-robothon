clc;
cla;
rosshutdown();
rosinit();
pause(2);
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2);
% jointStateStruct = jointStateSubscriber.LatestMessage;
% jointPosition = jointStateStruct.Pose(8).Position;
% point = [jointPosition.X jointPosition.Y jointPosition.Z];
% jointOrientation = jointStateStruct.Pose(8).Orientation;
% quat = [jointOrientation.W jointOrientation.X jointOrientation.Y jointOrientation.Z];
% 
% tr = transl(point)*quat2tform(quat);
% 
% tr(3,4) = tr(3,4) + 0.2;

