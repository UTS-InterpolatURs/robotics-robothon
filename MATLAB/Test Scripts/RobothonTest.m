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

fence = ModelGen('Envi/Fences.ply', transl(0,0,0));
fence.PlotModel;
collisionComputer = TesturCollisionDetection(robot);

% realBot = urRosWrapper(robot);
rc = RobotController(robot, collisionComputer);

tb = Taskboard(transl(0,-0.65,0.025));
tb.PlotTaskboard;


RobothonSimGui(robot, rc, tb);






