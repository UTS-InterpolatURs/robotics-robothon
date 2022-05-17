function PickAndPlaceEthCable(robot, rc, tb)
%PICKANDPLACEETHCABLE Summary of this function goes here
%   Detailed explanation goes here


goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose() * transl(0,0,0.2));

q = rc.GenerateLinearTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q) == false)

    return;

end


goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);

if(rc.ExecuteTrajectory(q) == false)
    
    return;

end

rc.CloseGripper(1000);

goalPose = robot.GlobalToEndEffPose(transl(0,0,0.1) * tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q, tb.cable) == false)
    
    return;

end

goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2 * transl(0,0,0.1));

q = rc.GenerateJointTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q, tb.cable) == false)
    
    return;

end

goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2);

q = rc.GenerateJointTrajectory(goalPose, 20);
if(rc.ExecuteTrajectory(q, tb.cable) == false)
    
    return;

end

rc.OpenGripper;


q = rc.moveCartesian([0,0,0.2], 20);
if(rc.ExecuteTrajectory(q) == false)
    
    return;

end
end

