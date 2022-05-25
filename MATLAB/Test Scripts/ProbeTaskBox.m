rc.SetToolGripper;
pause(3);

try rc.CloseGripper(800);
catch 
    disp("No Gripper")
end
timeout = 20;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

startPose = robot.model.fkine(robot.model.getpos());
startRot = tr2rpy(startPose);

rc.moveCartesian([0,0,-0.33], 4)

rc.waitForTrajToFinish(4);


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

currentPose = robot.model.fkine(robot.model.getpos());
% trplot(currentPose);
Z = currentPose(3,4);

rc.moveCartesian([0,0,0.2], 3);

rc.waitForTrajToFinish(3);

rc.moveEndEffector([0.085,-0.08,0], 3);

rc.waitForTrajToFinish(3);

rc.moveCartesian([0,0,-0.22], 3);

rc.waitForTrajToFinish(3);

%######################## PROBE LONG SIDE (fast) #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.001,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5);


rc.moveEndEffector([0.002,0,0], 0.5);

rc.waitForTrajToFinish(0.5);

%######################## PROBE LONG SIDE (slow) #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.0005,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5);

currentPose = robot.model.fkine(robot.model.getpos());

augmentedPose = currentPose * transl([(-0.0079 - (0.152/2)), 0, 0]);

augmentedPose(3,4) = Z;

P1 = augmentedPose(1:2,4);


rc.moveEndEffector([0.01,0.16,0], 2);

rc.waitForTrajToFinish(1);

% trplot(augmentedPose);
% Y = currentPose(1,4) + 0.0079 + (0.152/2);


% rc.moveEndEffector([0.05,0.1,0], 2);
% 
% rc.waitForTrajToFinish(1);

% rc.moveEndEffector([0.10,0.04,0], 3);
% 
% rc.waitForTrajToFinish(3);
% 
% rc.moveCartesian([0,0,-0.22], 3);
% 
% rc.waitForTrajToFinish(3);

%######################## PROBE LONG SIDE (fast) #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.001,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5);


rc.moveEndEffector([0.002,0,0], 0.5);

rc.waitForTrajToFinish(0.5);

%######################## PROBE LONG SIDE (slow) #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.0005,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5);

currentPose = robot.model.fkine(robot.model.getpos());

augmentedPose = currentPose * transl([(-0.0079 - (0.152/2)), 0, 0]);

augmentedPose(3,4) = Z;

P2 = augmentedPose(1:2,4);


X1 = [P1(1), P2(1)];
Y1 = [P1(2), P2(2)];

poly1 = polyfit(X1,Y1,1);
plot(X1,Y1)

rc.moveCartesian([0,0,0.22], 3);

rc.waitForTrajToFinish(3);

rc.moveEndEffector([-0.08,0.07,0], 3);

rc.waitForTrajToFinish(3);

rc.RotateSingleJoint(6,-pi/2,2);

rc.waitForTrajToFinish(2);

rc.moveCartesian([0,0,-0.22], 3);

rc.waitForTrajToFinish(3);

%######################## PROBE SHORT SIDE #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.001,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2)
end

pause(0.5)

rc.moveEndEffector([0.002,0,0], 0.5);

rc.waitForTrajToFinish(0.5);

%######################## PROBE SHORT SIDE (slow) #################################

startTime = toc;

while (1)
    %     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Y) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveEndEffector([-0.0005,0,0], rc.controlFrequency*2);

    if(toc-startTime >= timeout)
        break;
    end
    pause(0.2);
end

pause(0.5)

currentPose = robot.model.fkine(robot.model.getpos());

augmentedPoseTwo = currentPose * transl([(-0.0079 - (0.254/2)), 0, 0]);

augmentedPoseTwo(3,4) = Z;

augmentedPoseTwo = augmentedPoseTwo * transl (0,0.2,0);

P1 = augmentedPoseTwo(1:2,4);

augmentedPoseTwo = augmentedPoseTwo * transl (0,-0.4,0);

P2 = augmentedPoseTwo(1:2,4);

X2 = [P1(1), P2(1)];
Y2 = [P1(2), P2(2)];

poly2 = polyfit(X2,Y2,1);

%calculate intersection
x_intersect = fzero(@(x) polyval(poly1-poly2,x),3);
y_intersect = polyval(poly1,x_intersect);
plot(X2,Y2)
plot(x_intersect,y_intersect,'r*')

tbPose = startPose;
tbPose(1,4) = x_intersect;
tbPose(2,4) = y_intersect;
tbPose(3,4) = Z;

trplot(tbPose);
tbPose = robot.EndEffToGlobalPose(tbPose) * trotz(-pi/2);
newTb = Taskboard(tbPose);
newTb.PlotTaskboard;



rc.moveEndEffector([0.10,0,0], 2);

rc.waitForTrajToFinish(2);

rc.RotateSingleJoint(6,pi/1.5,2);

rc.waitForTrajToFinish(2);

rc.moveEndEffector([0,0,0.2], 2);

rc.waitForTrajToFinish(2);


homePose = startPose;
homePose(1,4) = x_intersect;
homePose(2,4) = y_intersect;
homePose(3,4) = Z + 0.2;
homePose = homePose * trotz(pi/2);


traj = rc.GenerateJointTrajectory(homePose, 3);
rc.ExecuteTrajectory(traj);

rc.waitForTrajToFinish(2);
% 
% deltax = abs((X1(2) - X1(1)));
% deltay = abs((Y1(2) - Y1(1)));
% d = deltay/deltax;
% 
% if(Y1(1) >= Y1(2))
%     theta = atan(d);
% else
%     theta = -atan(d);
% end
% 



