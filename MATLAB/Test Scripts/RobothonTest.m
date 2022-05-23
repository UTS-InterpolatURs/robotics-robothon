cla
clf
clear all
hold on;

robot = UTS_UR10;
PlotFloor();
wbPose = transl(0,-0.8,-0.423) * rpy2tr(0,0,0);
% wbPose = transl(-0.06,-0.8,-0.423) * rpy2tr(0,0,0); % <-- Lab Testing
workbench = ModelGen('Envi/Workbench.ply', wbPose);
workbench.PlotModel();

% fence = ModelGen('Envi/Fences.ply', transl(0,0,0.6));
% fence.PlotModel;
% 
% extinguisher = ModelGen('Envi/extinguisher.ply', transl(-1.2,-1,-0.6));
% extinguisher.PlotModel;

% 
% danger_sign = surface_plot('Envi/warning_sign.jpg', [-3,-3;-2,-2],[-3.13,-3.13;-3.13,-3.13],[1,2;1,2]);
realBot = urRosWrapper(robot);
collisionComputer = TesturCollisionDetection(robot);
rc = URController(robot, collisionComputer, realBot);

tb = Taskboard(transl(0,-0.65,0.1));
tb.PlotTaskboard;


RobothonSimGui(robot, rc, tb, realBot);



function surface = surface_plot(item, xx, yy, zz)
%function to plot an image in the environment (ie signs)

surface = surf(xx,yy,zz,'CData',imread(item),'FaceColor','texturemap');
end






