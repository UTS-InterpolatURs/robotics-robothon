lidPose = MlGripperPose * MlGripperToBatteryLidPose * transl([0,0,-0.01]);

rc.OpenGripper;


traj = rc.GenerateJointTrajectory(lidPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

forceThreshold = 25;

while (1)
    disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("Probed")
        break;
    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

    pause(0.2);
end

pause(0.1);

rc.moveEndEffector([0,-0.08,0], 2);

rc.waitForTrajToFinish(2);

traj = rc.GenerateJointTrajectory(MlGripperPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);
