function PickAndPlaceKey(robot, rc, tb)
%PICKANDPLACEKEY Summary of this function goes here
%   Detailed explanation goes here
%move to location above key

torqueThreshold = 2;

rc.MoveToNeutral;

goalPose = robot.GlobalToEndEffPose(tb.key.GetPose() * transl(0,0,0.2));

q = rc.GenerateLinearTrajectory(goalPose, 2, [1,1,1,0,0,0]);
if(rc.ExecuteTrajectory(q) == false)

    return;

end

goalPose = robot.GlobalToEndEffPose(tb.key.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 200, [1,1,1,1,1,1]);
if(rc.ExecuteTrajectory(q) == false)

    return;

end


rc.CloseGripper(1000);

q = rc.moveCartesian([0,0,0.1], 2);
if(rc.ExecuteTrajectory(q, tb.key) == false)

    return;

end


goalPose = robot.GlobalToEndEffPose(tb.GetGoalKeySwitch * transl(0,0,0.1));

q = rc.GenerateLinearTrajectory(goalPose, 2, [1,1,1,0,0,0]);
if(rc.ExecuteTrajectory(q, tb.key) == false)

    return;

end


goalPose = goalPose * trotz(pi/2);

q = rc.GenerateJointTrajectory(goalPose, 100);
if(rc.ExecuteTrajectory(q, tb.key) == false)

    return;

end


goalPose = robot.GlobalToEndEffPose(tb.GetGoalKeySwitch * trotz(pi/2));

q = rc.GenerateLinearTrajectory(goalPose, 2, [1,1,1,0,0,0]);
if(rc.ExecuteTrajectory(q, tb.key) == false)

    return;

end

% goalPose = goalPose * trotz(pi/5);
%
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q, tb.key) == false)
%
%     return;
%
% end
%
% goalPose = goalPose * trotz(-pi/5);
%
% q = rc.GenerateJointTrajectory(goalPose, 20);
% if(rc.ExecuteTrajectory(q, tb.key) == false)
%
%     return;
%
% end

tic;

pause(0.1);


while (1)
    currentQ = robot.model.getpos();
    goalQ = currentQ;
    goalQ(6) = goalQ(6) + 0.01;
    qMatrix = [currentQ; goalQ];
    rc.ExecuteTrajectory(qMatrix, tb.key);
    pause(rc.controlFrequency);

    if(rc.useRos)
        if(abs(rc.realBot.wrench.Torque.Z) > torqueThreshold)
            break;
        end
    end

    disp(toc);

    if(toc >= 10)
        break;
    end
end

rc.OpenGripper;


q = rc.moveCartesian([0,0,0.2], 3);
if(rc.ExecuteTrajectory(q) == false)

    return;

end







