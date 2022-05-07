function PickAndPlaceKey(robot,keyStartPose,keyFinishPose, realBot)
%PICKANDPLACEKEY Summary of this function goes here
%   Detailed explanation goes here

%move to location above key

if ~exist('realBot','var') || isempty(realBot)
    useRos=false;
else
useRos = true;
end

p1 = transl(0,0,0.1) * keyStartPose;

t = TrajectoryGenerator(robot);

pause(2);
if(useRos)
    robot.model.animate(realBot.current_joint_states.Position);
end
q1 = t.LinearTrajectory(p1,50);

if(useRos)
    realBot.sendJointTrajectory(q1);
end


for i=1:size(q1,1)

    robot.model.animate(q1(i,:))
    drawnow();
    pause(0.1);

end

robot.SetGripperState("gripperState", 0)

q2 = t.LinearTrajectory(keyStartPose,10);
if(useRos)
    realBot.sendJointTrajectory(q2);
end

for i=1:size(q2,1)

    robot.model.animate(q2(i,:))
    drawnow();
    pause(0.1);

end

robot.SetGripperState("gripperState", 1)

q1 = t.LinearTrajectory(p1,10);
if(useRos)
    realBot.sendJointTrajectory(q1);
end

for i=1:size(q1,1)

    robot.model.animate(q1(i,:))
    drawnow();
    pause(0.1);

end

p4 = transl(0,0,0.1) * keyFinishPose;

t = TrajectoryGenerator(robot);

q4 = t.LinearTrajectory(p4,50);
if(useRos)
    realBot.sendJointTrajectory(q4);
end

for i=1:size(q4,1)

    robot.model.animate(q4(i,:))
    drawnow();
    pause(0.1);

end





