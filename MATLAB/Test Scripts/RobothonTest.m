cla
clf
clear all

robot = UTS_UR10;
rc = RobotController(robot);

tb = Taskboard(transl(-0.65,0,0) * trotz(pi/2));
tb.PlotTaskboard;


ax1 = gca;

robot.model.animate([0,-pi/2,pi/2,-pi/2,-pi/2,0]);



RobothonSimGui(robot, rc, tb);

