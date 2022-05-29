
ethPose = MlGripperPose * MlGripperToEthPose * transl([0,0,0.001]);
ethTargetPose = MlGripperPose * MlGripperToEthTargetPose * transl([0,-0.0015,0]);
directionCounter = 0;
xMove = 0.0008;

rc.moveEndEffector([0,0.2,0], 2);
rc.waitForTrajToFinish(2);

traj = rc.GenerateJointTrajectory(ethPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

rc.CloseGripper(1000);

rc.moveCartesian([0,0,0.02], 2);
rc.waitForTrajToFinish(2);

traj = rc.GenerateJointTrajectory(ethTargetPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);

while (1)
    if(abs(rc.realBot.wrench.Force.Z) > 18)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);
        directionCounter = directionCounter + 1;
        disp(abs(currentZ - Z));

        if(abs(currentZ - Z) > 0.0083)
            disp("cable inserted")
            break;
        elseif (directionCounter > 0)

            if(directionCounter >= 5)
                traj = rc.GenerateJointTrajectory(((ethTargetPose) * transl([-xMove,0,0])),1);
                rc.ExecuteTrajectory(traj);
                rc.waitForTrajToFinish(1);
                xMove = -xMove;
                directionCounter = 0;
            end

            rc.moveEndEffector([xMove,0,0.004], 1);
            rc.waitForTrajToFinish(1);
        end

    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);
    pause(0.2);
end

% rc.OpenGripper;
% 
% rc.moveCartesian([0,0,0.01], 1);
% rc.waitForTrajToFinish(1);
% 
% 
% rc.CloseGripper(1000);
% 
% while (1)
%     if(abs(rc.realBot.wrench.Force.Z) > 15)
%         disp("Cable Secured")
%         break;
%     end
% 
%     rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);
%     pause(0.2);
% end


rc.OpenGripper;

traj = rc.GenerateJointTrajectory(MlGripperPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);
