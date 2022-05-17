function PickAndPlaceKey(robot, rc, tb)
%PICKANDPLACEKEY Summary of this function goes here
%   Detailed explanation goes here
%move to location above key

goalPose = robot.GlobalToEndEffPose(tb.key.GetPose() * transl(0,0,0.2));

q = rc.GenerateLinearTrajectory(goalPose, 20, [1,1,1,0,0,0]);
if(rc.ExecuteTrajectory(q) == false)
    
    return;

end

goalPose = robot.GlobalToEndEffPose(tb.key.GetPose());

q = rc.GenerateLinearTrajectory(goalPose, 20, [1,1,1,0,0,0]);
if(rc.ExecuteTrajectory(q) == false)
    
    return;

end

rc.CloseGripper(1000);

q = rc.moveCartesian([0,0,0.1], 20);
if(rc.ExecuteTrajectory(q) == false)
    
    return;

end

goalPose = robot.GlobalToEndEffPose(tb.GetGoalKeySwitch * transl(0,0,0.1) * trotz(pi/2));

q = rc.GenerateJointTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q, tb.key) == false)
    
    return;

end

goalPose = robot.GlobalToEndEffPose(tb.GetGoalKeySwitch * trotz(pi/2));

q = rc.GenerateJointTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q, tb.key) == false)
    
    return;

end

goalPose = goalPose * trotz(pi/3);

q = rc.GenerateJointTrajectory(goalPose, 20, [0,0,0,0,0,0.5]);
if(rc.ExecuteTrajectory(q, tb.key) == false)
    
    return;

end

goalPose = goalPose * trotz(-pi/3);

q = rc.GenerateJointTrajectory(goalPose, 20,[0,0,0,0,0,0.5]);
if(rc.ExecuteTrajectory(q, tb.key) == false)
    
    return;

end

rc.OpenGripper;


q = rc.moveCartesian([0,0,0.2], 20);
if(rc.ExecuteTrajectory(q) == false)
    
    return;

end







