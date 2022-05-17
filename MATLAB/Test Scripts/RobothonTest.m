cla
clf
clear all

robot = UTS_UR10;
rc = RobotController(robot);

tb = Taskboard(transl(-0.65,0,0) * trotz(pi/2));
tb.PlotTaskboard;




RobothonSimGui(robot, rc, tb);

