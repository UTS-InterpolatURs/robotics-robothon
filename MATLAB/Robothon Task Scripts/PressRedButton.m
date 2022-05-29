redButtonPose = MlGripperPose * MlGripperToBlueButtonPose;

redButtonPose(3,4) = startPose(3,4) - 0.2;

traj = rc.GenerateJointTrajectory(redButtonPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);


rc.OpenGripper();

timeout = 100;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

rc.moveEndEffector([0,-0.015,-0.13], 2)

rc.waitForTrajToFinish(2);


forceThreshold = 5;

pause(0.5);

startTime = toc;

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