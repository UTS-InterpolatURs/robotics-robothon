
keyPose = MlGripperPose * MlGripperToKeyPose;

keyPose(3,4) = startPose(3,4) - 0.2;

traj = rc.GenerateJointTrajectory(keyPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);


rc.OpenGripper();

timeout = 60;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

% rc.moveCartesian([0,0,-0.14], 1)
% 
% rc.waitForTrajToFinish(1);

keyPose(3,4) = Z + 0.01;

traj = rc.GenerateJointTrajectory(keyPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);


forceThreshold = 10;

% startTime = toc;

% % ######################## PROBE TOP OF BOX (fast) #################################
% while (1)
%         disp(rc.realBot.wrench.Force.Z)
% 
%     if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
%         disp("canceling")
%         break;
%     end
% 
%     rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);
% 
%     if(toc-startTime >= timeout)
%         break;
%     end
%     pause(0.2);
% end
% 
% pause(0.5);
% 
% rc.moveCartesian([0,0,0.002], 0.5);
% 
% rc.waitForTrajToFinish(0.5);
% 
% 
% 
% % ######################## PROBE TOP OF BOX (slow) #################################
% startTime = toc;
% while (1)
%         disp(rc.realBot.wrench.Force.Z)
% 
%     if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
%         disp("canceling")
%         break;
%     end
% 
%     rc.moveCartesian([0,0,-0.0005], rc.controlFrequency*2);
% 
%     if(toc-startTime >= timeout)
%         break;
%     end
%     pause(0.2);
% end
% 
% pause(0.5);
% 
% rc.moveCartesian([0,0,0.015], 1);

rc.CloseGripper(950);

rc.moveCartesian([0,0,0.04], 1);

rc.waitForTrajToFinish(1);

currentPose = robot.model.fkine(robot.model.getpos());

Z = currentPose(3,4);

keyHolePose =  currentPose * keyStartToAboveHoleTf *transl([-0.003,0,0]);

traj = rc.GenerateJointTrajectory(keyHolePose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);

startTime = toc;

directionCounter = 0;
xMove = -0.00085;

while (1)
%         disp(rc.realBot.wrench.Force.Z)
    
    
    if(abs(rc.realBot.wrench.Force.Z) > 25)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);
        directionCounter = directionCounter + 1;
        disp(abs(currentZ - Z))

        if(abs(currentZ - Z) > 0.026)
            disp("key inserted")
            break;
        elseif (directionCounter > 0)

        if(directionCounter >= 8)
            traj = rc.GenerateJointTrajectory((keyHolePose * transl([-xMove,0,0])),2);
            rc.ExecuteTrajectory(traj);
            rc.waitForTrajToFinish(2);
            xMove = -xMove;
            directionCounter = 0;
        end
            
            rc.moveEndEffector([xMove,0,0.005], 1);
            rc.waitForTrajToFinish(1);
        end

    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

        if(toc-startTime >= timeout)
            break;
        end
    pause(0.2);
end

pause(0.5);

rc.RotateSingleJoint(6,deg2rad(55),2);

pause(2);

rc.OpenGripper;

traj = rc.GenerateJointTrajectory(MlGripperPose * transl([0,0,0.2]), 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);

