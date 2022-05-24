rc.SetToolGripper;
rc.CloseGripper(800);

timeout = 30;


while(1)
    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.1);
end

rc.moveCartesian([0,0,-0.33], 4)

rc.waitForTrajToFinish(4);


forceThreshold = 10;

startTime = toc;

%######################## PROBE TOP OF BOX #################################
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

rc.moveCartesian([0,0,0.2], 3);

rc.waitForTrajToFinish(3);

rc.moveEndEffector([0.10,0,0], 3);

rc.waitForTrajToFinish(3);

rc.moveCartesian([0,0,-0.22], 3);

rc.waitForTrajToFinish(3);

%######################## PROBE LONG SIDE #################################

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

rc.moveEndEffector([0.05,0,0], 1);

rc.waitForTrajToFinish(1);

rc.moveCartesian([0,0,0.22], 3);

rc.waitForTrajToFinish(3);

rc.moveEndEffector([-0.15,0.16,0], 3);

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

rc.moveEndEffector([0.05,0,0], 1);

rc.waitForTrajToFinish(1);




