cla
clear all

robot = UTS_UR10;
rc = RobotController(robot);

advanced_teach(robot);

robot.model.animate([0,-pi/2,pi/2,-pi/2,-pi/2,0]);

tb = Taskboard(transl(-0.65,0,0) * trotz(pi/2));
tb.PlotTaskboard;

startPose = tb.cable.GetPose();

goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose() * transl(0,0,0.1));

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q);

goalPose = robot.GlobalToEndEffPose(tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q);

rc.CloseGripper;

goalPose = robot.GlobalToEndEffPose(transl(0,0,0.1) * tb.cable.GetPose());

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

goalPose = robot.GlobalToEndEffPose(transl(0,0.045,0.1) * startPose);

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

goalPose = robot.GlobalToEndEffPose(transl(0,0.045,0) * startPose);

q = rc.GenerateJointTrajectory(goalPose, 20);
rc.ExecuteTrajectory(q, tb.cable);

rc.OpenGripper;

