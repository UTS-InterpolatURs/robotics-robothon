

currentPose = robot.model.fkine(robot.model.getpos());

rc.SetToolGripper;

while(1)
     if(rc.realBot.robotBusy == false)
        break;
     end
     pause(0.1);
end

rc.moveCartesian([0,0,-0.34], 4)

while(1)
     if(rc.realBot.robotBusy == false)
        break;
     end
     pause(0.1);
end


forceThreshold = 5;

tic;
pause(1)


while (1)
%     disp(rc.realBot.wrench.Force.Z)

    if(abs(rc.realBot.wrench.Force.Z) > forceThreshold)
        disp("canceling")
        break;
    end

    rc.moveCartesian([0,0,-0.001], 0.1);

    if(toc >= 20)
        break;
    end
     pause(0.1);
end

disp("exit")