function PickAndPlaceEthCable(robot, rc, tb)
%PICKANDPLACEETHCABLE Summary of this function goes here
%   Detailed explanation goes here


goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose() * transl(0,0,0.2));

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q);

goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q);

rc.CloseGripper;

goalPose = robot.GlobalToEndEffPose(transl(0,0,0.1) * tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2 * transl(0,0,0.1));

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2);

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

rc.OpenGripper;


q = rc.moveCartesian([0,0,0.2], 20);
rc.ExecuteTrajectory(q);
end

