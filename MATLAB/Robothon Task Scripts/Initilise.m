load("MlGripperToBlueButtonPose.mat");
load("MlGripperToKeyPose.mat");
load("keyStartToAboveHoleTf.mat");
load("MlGripperToEthPose.mat");
load("MlGripperToEthTargetPose.mat");
load("MlGripperToBatteryLidPose.mat");



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
pause(0.5);