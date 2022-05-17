cla
clf
clear all

robot = UTS_UR10;
collisionComputer = TesturCollisionDetection(robot);

% realBot = urRosWrapper(robot);
rc = RobotController(robot, collisionComputer);

tb = Taskboard(transl(0,0-0.65,0));
tb.PlotTaskboard;


robot.model.animate([-1.9955   -2.2444   -1.8776   -0.5904    1.5708    0.3607]);
RobothonSimGui(robot, rc, tb);


traj = rc.moveCartesian([1,0,0], 50);
rc.checkCollisionFlag = true;
rc.ExecuteTrajectory(traj);
