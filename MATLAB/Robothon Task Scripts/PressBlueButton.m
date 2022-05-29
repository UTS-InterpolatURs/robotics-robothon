
blueButtonPose = MlGripperPose * MlGripperToBlueButtonPose;

blueButtonPose(3,4) = startPose(3,4) - 0.2;

traj = rc.GenerateJointTrajectory(blueButtonPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);


rc.CloseGripper(600);

timeout = 100;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

rc.moveEndEffector([0,-0.005,-0.13], 2)

rc.waitForTrajToFinish(2);


forceThreshold = 5;

startTime = toc;

% ######################## PROBE TOP OF BOX (fast) #################################
while (1)
    disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("Probed")
        break;
    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end


rc.moveCartesian([0,0,0.002], 0.5);

rc.waitForTrajToFinish(0.5);


% ######################## PROBE TOP OF BOX (slow) #################################
startTime = toc;
while (1)
    disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveCartesian([0,0,-0.0005], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

currentPose = robot.model.fkine(robot.model.getpos());

Z = currentPose(3,4);

pause(0.5);


rc.moveEndEffector([0,-0.004,0.01], 1)
rc.waitForTrajToFinish(1);

rc.OpenGripper();
pause(0.5);

while (1)
    disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("Probed")
        break;
    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end


rc.moveCartesian([0,0,0.1], 2);

rc.waitForTrajToFinish(2);


traj = rc.GenerateJointTrajectory(MlGripperPose * transl([0,0,0.2]), 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);