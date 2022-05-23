
tic
rc.moveCartesian([0,0,-0.2], 10)

while(1)
    if(toc >=5)
        rc.realBot.CancelTrajectory();
        break;
    end

end

disp(toc);