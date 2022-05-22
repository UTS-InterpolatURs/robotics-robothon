function PickAndPlaceEthCableTwo(robot, rc, tb)
%PICKANDPLACEETHCABLE Summary of this function goes here
%   Detailed explanation goes here


goalPose = robot.GlobalToEndEffPose(tb.mainboard.GetPose * transl(0.07,0,0.25)* trotz(pi));

traj = rc.GenerateJointTrajectory(goalPose,3);

rc.ExecuteTrajectory(traj);


% #################REPLACE WITH PROBE Z HEIGHT#####################%

goalPose = robot.GlobalToEndEffPose(tb.mainboard.GetPose * transl(0.07,0,0) * trotz(pi));

traj = rc.GenerateJointTrajectory(goalPose,2);

rc.ExecuteTrajectory(traj);


%##########################HARD CODED Z OFFSET#######################
z_offset = 0.01;

goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose * transl(0.02,0,0.1) * trotz(pi));

traj = rc.GenerateJointTrajectory(goalPose,1);

rc.ExecuteTrajectory(traj);


goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose * transl(0.02,0,z_offset) * trotz(pi) * troty(-pi/5));

traj = rc.GenerateJointTrajectory(goalPose,2);

rc.ExecuteTrajectory(traj);

goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose * transl(0.00,0,z_offset) * trotz(pi) * troty(-pi/5));

traj = rc.GenerateJointTrajectory(goalPose,2);

rc.ExecuteTrajectory(traj);

rc.CloseGripper

rc.moveCartesian()



% goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose() * transl(0,0,0.2));
% 
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q) == false)
% 
%     return;
% 
% end
% 
% 
% goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose());
% 
% q = rc.GenerateJointTrajectory(goalPose, 20);
% 
% if(rc.ExecuteTrajectory(q) == false)
%     
%     return;
% 
% end
% 
% rc.CloseGripper(1000);
% 
% goalPose = robot.GlobalToEndEffPose(transl(0,0,0.1) * tb.cable.GetPose());
% 
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q, tb.cable) == false)
%     
%     return;
% 
% end
% 
% goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2 * transl(0,0,0.1));
% 
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q, tb.cable) == false)
%     
%     return;
% 
% end
% 
% goalPose = robot.GlobalToEndEffPose(tb.GetGoalCableSlot2);
% 
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q, tb.cable) == false)
%     
%     return;
% 
% end
% 
% rc.OpenGripper;
% 
% 
% q = rc.moveCartesian([0,0,0.2], 20);
% if(rc.ExecuteTrajectory(q) == false)
%     
%     return;
% 
% end
end

