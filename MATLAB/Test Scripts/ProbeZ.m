

currentPose = robot.model.fkine(robot.model.getpos());

rc.SetToolGripper;

rc.moveCartesian([0,0,-0.2], 3)

forceThreshold = 10;

tic;

pause(0.1);


while (1)
    rc.moveCartesian([0,0,-0.001], 0.1)
    pause(rc.controlFrequency);

    if(rc.useRos)
        if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
            break;
        end
    end

    disp(toc);

    if(toc >= 30)
        break;
    end
end