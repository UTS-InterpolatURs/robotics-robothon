load("homeToEthTf.mat");
load("homeToEthHoleTf.mat");

rc.moveEndEffector([-0.2,0,0], 2);
rc.waitForTrajToFinish(2);

ethPose = homePose * homeToEthTf;
ethPose = ethPose * transl([0,0,-0.003]);

traj = rc.GenerateJointTrajectory(ethPose,2);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(2);

rc.CloseGripper(1100);

rc.moveCartesian([0,0,0.03], 2);
rc.waitForTrajToFinish(2);

aboveEthHolePose = homePose * homeToEthHoleTf;

traj = rc.GenerateJointTrajectory((aboveEthHolePose),2);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(2);


while (1)
    if(abs(rc.realBot.wrench.Force.Z) > 15)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);
        directionCounter = directionCounter + 1;

        if(abs(currentZ - Z) < 0.02)
            disp("cable inserted")
            break;
        elseif (directionCounter > 0)

            if(directionCounter >= 5)
                traj = rc.GenerateJointTrajectory(((aboveEthHolePose) * transl([-xMove,0,0])),2);
                rc.ExecuteTrajectory(traj);
                rc.waitForTrajToFinish(2);
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

rc.OpenGripper;

rc.moveCartesian([0,0,0.01], 1);
rc.waitForTrajToFinish(1);


rc.CloseGripper(1000);

while (1)
    if(abs(rc.realBot.wrench.Force.Z) > 15)
        disp("Cable Secured")
        break;
    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);
    pause(0.2);
end


rc.OpenGripper;

traj = rc.GenerateJointTrajectory(homePose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);



