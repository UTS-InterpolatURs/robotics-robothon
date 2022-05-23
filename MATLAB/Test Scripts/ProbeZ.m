

currentPose = robot.model.fkine(robot.model.getpos());

rc.SetToolGripper;

rc.moveCartesian([0,0,-0.5], 50)

forceThreshold = 5;

tic;
pause(1)


while (1)
    disp(rc.realBot.wrench.Force.Z)
    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        rc.realBot.CancelTrajectory();
        disp("canceling")
        break;
    end

    if(rc.realBot.robotBusy == false)
        break;
    end
    pause(0.001);
end

disp("exit")