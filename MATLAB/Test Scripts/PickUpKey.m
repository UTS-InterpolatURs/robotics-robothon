load("homeToKeyTf.mat");
load("homeToAboveKeyHoleTf.mat");
rc.OpenGripper;

aboveKeyTf = homePose * homeToKeyTf * transl(0,0,-0.04);

traj = rc.GenerateJointTrajectory(aboveKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);


traj = rc.GenerateJointTrajectory(homePose * homeToKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);

rc.CloseGripper(1100);

traj = rc.GenerateJointTrajectory(aboveKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);

aboveKeyHoleTf = homePose * homeToAboveKeyHoleTf;

traj = rc.GenerateJointTrajectory(aboveKeyHoleTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);

% rc.moveEndEffector([0.001,0,0], 1);
% rc.waitForTrajToFinish(1)

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > 15)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);

        if(currentZ - Z < 0.019)
            disp("key inserted")
            break;
        else

            rc.moveEndEffector([-0.001,0,0.004], 1);
            rc.waitForTrajToFinish(1);
        end

    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

%     if(toc-startTime >= timeout)
%         break;
%     end
    pause(0.2);
end

pause(0.5);

rc.RotateSingleJoint(6,deg2rad(55),5);

pause(5);

rc.OpenGripper;

