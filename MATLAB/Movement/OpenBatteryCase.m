function OpenBatteryCase(controller,startPose)
%PICKANDPLACEKEY Summary of this function goes here
%   Detailed explanation goes here

%move to location above key


p1 = transl(0,0,0.1) * startPose;


t = controller;


q1 = t.GenerateLinearTrajectory(p1,30);

t.ExecuteTrajectory(q1);

t.OpenGripper


q2 = t.GenerateLinearTrajectory(startPose,10);
t.ExecuteTrajectory(q2);


p2 = transl(0.1,0,0) * startPose;
q3 = t.GenerateLinearTrajectory(p2,10);
t.ExecuteTrajectory(q3);







