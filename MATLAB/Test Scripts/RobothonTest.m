cla
clf
clear all

robot = UTS_UR10;
realBot = urRosWrapper(robot);
rc = RobotController(robot);

tb = Taskboard(transl(0.65,0,0.2) * trotz(pi/2));
tb.PlotTaskboard;




RobothonSimGui(robot, rc, tb);

