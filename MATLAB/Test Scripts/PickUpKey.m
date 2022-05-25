load("homeToKeyTf.mat");

rc.OpenGripper;

aboveKeyTf = homePose * homeToKeyTf * transl(0,0,-0.04);

traj = rc.GenerateJointTrajectory(aboveKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);


traj = rc.GenerateJointTrajectory(homePose * homeToKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);

rc.CloseGripper(1000);
pause(2);

traj = rc.GenerateJointTrajectory(aboveKeyTf,5);
rc.ExecuteTrajectory(traj);
rc.waitForTrajToFinish(5);
