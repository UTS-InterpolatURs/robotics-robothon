
load("MlGripperToKeyPose.mat");
load("keyStartToAboveHoleTf.mat");
% 
% MachineLearningClient = rossvcclient("/get_feature","DataFormat","struct");
% pause(0.1);
% 
% MlRequest = rosmessage(MachineLearningClient);
% MlResponse = call(MachineLearningClient,MlRequest,"Timeout",3);
% 
% 


startPose = robot.model.fkine(robot.model.getpos());

MlPose = robot.model.fkine(robot.model.getpos()) * transl([0.0592117, -0.0336165 0]);

traj = rc.GenerateJointTrajectory(MlPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);

rc.SetToolGripper;
pause(3);

MlGripperPose = robot.model.fkine(robot.model.getpos());


keyPose = MlGripperPose * MlGripperToKeyPose;

keyPose(3,4) = startPose(3,4);

traj = rc.GenerateJointTrajectory(keyPose, 1);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(1);


rc.OpenGripper();

timeout = 20;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

rc.moveCartesian([0,0,-0.34], 5)

rc.waitForTrajToFinish(5);


forceThreshold = 10;

startTime = toc;

%######################## PROBE TOP OF BOX (fast) #################################
while (1)
    %     disp(rc.realBot.wrench.Force.Z)

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



%######################## PROBE TOP OF BOX (slow) #################################
startTime = toc;
while (1)
    %     disp(rc.realBot.wrench.Force.Z)

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

keyHolePose =  currentPose * keyStartToAboveHoleTf;

traj = rc.GenerateJointTrajectory(keyHolePose, 3);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(3);

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

        if(abs(currentZ - Z) > 0.024)
            disp("key inserted")
            break;
        elseif (directionCounter > 0)

        if(directionCounter >= 5)
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

    %     if(toc-startTime >= timeout)
    %         break;
    %     end
    pause(0.2);
end

pause(0.5);

rc.RotateSingleJoint(6,deg2rad(55),3);

pause(3);

rc.OpenGripper;







