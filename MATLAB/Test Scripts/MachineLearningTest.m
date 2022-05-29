
load("MlGripperToKeyPose.mat");
load("keyStartToAboveHoleTf.mat");
load("MlGripperToEthPose.mat");
load("MlGripperToEthTargetPose.mat");
% 
% MachineLearningClient = rossvcclient("/get_features","DataFormat","struct");
% pause(0.1);
% 
% MlRequest = rosmessage(MachineLearningClient);
% MlResponse = call(MachineLearningClient,MlRequest,"Timeout",3);
% 
% 

MlSub =  rossubscriber('/task_detect/detected_points', 'geometry_msgs/Point32', "DataFormat","struct");
pause(2);
x = double(MlSub.LatestMessage.X);
y = double(MlSub.LatestMessage.Y);

startPose = robot.model.fkine(robot.model.getpos());

MlPose = robot.model.fkine(robot.model.getpos()) * transl([x, y, 0]);

traj = rc.GenerateJointTrajectory(MlPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1)

rc.SetToolGripper;
pause(2);

MlGripperPose = robot.model.fkine(robot.model.getpos());


keyPose = MlGripperPose * MlGripperToKeyPose;

keyPose(3,4) = startPose(3,4);

traj = rc.GenerateJointTrajectory(keyPose, 1);
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

rc.moveCartesian([0,0,-0.34], 3)

rc.waitForTrajToFinish(3);


forceThreshold = 10;

startTime = toc;

% ######################## PROBE TOP OF BOX (fast) #################################
while (1)
        disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveCartesian([0,0,-0.001], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5);

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

pause(0.5);

rc.moveCartesian([0,0,0.015], 1);

rc.CloseGripper(1100);

rc.moveCartesian([0,0,0.04], 1);

rc.waitForTrajToFinish(1);

currentPose = robot.model.fkine(robot.model.getpos());

Z = currentPose(3,4);

keyHolePose =  currentPose * keyStartToAboveHoleTf *transl([-0.002,0,0]);

traj = rc.GenerateJointTrajectory(keyHolePose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

startTime = toc;

directionCounter = 0;
xMove = -0.0008;

while (1)
%         disp(rc.realBot.wrench.Force.Z)
    
    
    if(abs(rc.realBot.wrench.Force.Z) > 19)
        disp("force reached")
        currentPose = robot.model.fkine(robot.model.getpos());
        currentZ = currentPose(3,4);
        directionCounter = directionCounter + 1;
        disp(abs(currentZ - Z))

        if(abs(currentZ - Z) > 0.026)
            disp("key inserted")
            break;
        elseif (directionCounter > 0)

        if(directionCounter >= 6)
            traj = rc.GenerateJointTrajectory((keyHolePose * transl([-xMove,0,0])),2);
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

        if(toc-startTime >= timeout)
            break;
        end
    pause(0.2);
end

pause(0.5);

rc.RotateSingleJoint(6,deg2rad(55),3);

pause(3);

rc.OpenGripper;

traj = rc.GenerateJointTrajectory(MlGripperPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);


ethPose = MlGripperPose * MlGripperToEthPose * transl([0,0,0.001]);
ethTargetPose = MlGripperPose * MlGripperToEthTargetPose;
directionCounter = 0;
xMove = 0.0008;

rc.moveEndEffector([0,0.2,0], 2);
rc.waitForTrajToFinish(2);

traj = rc.GenerateJointTrajectory(ethPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

rc.CloseGripper(1150);

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

        if(abs(currentZ - Z) > 0.0285 || abs(rc.realBot.wrench.Force.Z) > 25)
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

traj = rc.GenerateJointTrajectory(MlGripperPose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);











