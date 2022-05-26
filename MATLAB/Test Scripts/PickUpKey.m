load("homeToKeyTf.mat");
load("homeToAboveKeyHoleTf.mat");
% rc.OpenGripper;
% 
% aboveKeyTf = homePose * homeToKeyTf * transl(0,0,-0.04);
% 
% traj = rc.GenerateJointTrajectory(aboveKeyTf,2);
% rc.ExecuteTrajectory(traj);
% rc.waitForTrajToFinish(2);
% 
% 
% traj = rc.GenerateJointTrajectory(homePose * homeToKeyTf,1);
% rc.ExecuteTrajectory(traj);
% rc.waitForTrajToFinish(1);

rc.CloseGripper(1100);

traj = rc.GenerateJointTrajectory(aboveKeyTf,2);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(2);

aboveKeyHoleTf = homePose * homeToAboveKeyHoleTf;

traj = rc.GenerateJointTrajectory(aboveKeyHoleTf,2);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(2);

rc.moveEndEffector([0.001,0,0], 1);
rc.waitForTrajToFinish(1)

startTime = toc;

directionCounter = 1;
xMove = -0.0008;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)
    

    if(abs(rc.realBot.wrench.Force.Z) > 15)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);
        directionCounter = directionCounter + 1;

        if(abs(currentZ - Z) < 0.02)
            disp("key inserted")
            break;
        elseif (directionCounter > 0)

        if(directionCounter >= 5)
            traj = rc.GenerateJointTrajectory((aboveKeyHoleTf * transl([-xMove,0,0])),2);
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

    %     if(toc-startTime >= timeout)
    %         break;
    %     end
    pause(0.2);
end

pause(0.5);

rc.RotateSingleJoint(6,deg2rad(55),3);

pause(3);

rc.OpenGripper;

traj = rc.GenerateJointTrajectory(homePose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

