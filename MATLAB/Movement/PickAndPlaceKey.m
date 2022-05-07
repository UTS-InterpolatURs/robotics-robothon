function PickAndPlaceKey(robot,keyStartPose,keyFinishPose, realBot)
%PICKANDPLACEKEY Summary of this function goes here
%   Detailed explanation goes here

%move to location above key

if ~exist('realBot','var') || isempty(realBot)
t = RobotController(robot);
else
t = RobotController(robot, realBot);
end

p1 = transl(0,0,0.1) * keyStartPose;





q1 = t.GenerateLinearTrajectory(p1,50);

t.ExecuteTrajectory(q1);

t.OpenGripper


q2 = t.GenerateLinearTrajectory(keyStartPose,10);
t.ExecuteTrajectory(q2);

t.CloseGripper

q3 = t.GenerateLinearTrajectory(p1,10);
t.ExecuteTrajectory(q3);

p4 = transl(0,0,0.1) * keyFinishPose;


q4 = t.GenerateLinearTrajectory(p4,50);
t.ExecuteTrajectory(q4);





