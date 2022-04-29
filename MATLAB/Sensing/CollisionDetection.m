%%
clc;
clf;
cla;
rosshutdown();
rosinit();
%%
clear;
clc;
clf;

addpath '../UR10e/'
addpath '../'
robot = UTS_UR10;
[d_1 d_2 d_3 d_4 d_5 d_6] = robot.model.links.d;
d_ur10 = [d_1 d_2 d_3 d_4 d_5 d_6];
[a_1 a_2 a_3 a_4 a_5 a_6] = robot.model.links.a;
a_ur10 = [a_1 a_2 a_3 a_4 a_5 a_6];
[alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6] = robot.model.links.alpha;
alpha_ur10 = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];
% robot.PlotAndColourRobot;
% q = [0 0 0 -pi -pi/2 0];
% robot.model.animate(q);
% hold on

if true
    num_image = 10;
    msg_array = ImageStorage(480,num_image);
    for i = 1:num_image
        A = rossubscriber('/camera/depth/image_rect_raw');
        pause(0.4);
        depthImage = readImage(A.LatestMessage);
        msg_array.addImage(depthImage,i);
    end
    
    %     JointStates = rossubcriber();
    JointStates = [0 0 0 -pi -pi/2 0];
    endEffector = robot.model.fkine(JointStates);
    point1OnLine = [endEffector(1,4) endEffector(2,4) endEffector(3,4)];
    
    %point2OnLine
    joinNumber = 5;
    jointState = robot.model.base;
    for i = 1:joinNumber
        joinState = jointState * trotz(JointStates(i)) * transl(a_ur10(1,i), 0, d_ur10(1,i)) * trotx(alpha_ur10(1,i));
    end
    point2OnLine = [joinState(1,4) joinState(2,4) joinState(3,4)];
    
end
imshow(depth);


fdx = 420.3378;
fdy = 422.5609;

u0 = 424;
v0 = 240;

h = 480;
w = 848;

u = repmat(1:w,[h,1]);
v = repmat(1:h,[w,1])';


Z = depth;
Z = double(Z(:));
X = (double(Z(:)).*(u(:)-u0))/fdx;
Y = (double(Z(:)).*(v(:)-v0))/fdy;
Pt = [X(:),Y(:),Z(:)];

a=text(424,240, strcat('Z: ', num2str(round(Z))));
set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'magenta');


%         ptCloudOut = pcdownsample(pointCloud(Pt),'gridAverage',0.1);
%         PointCloud_raw = ptCloudOut.Location;
%         plot3(Pt(:,1),Pt(:,2),Pt(:,3));

for i = 1:length(Pt)
    
    tr = eye(4)*transl(Pt(i,1)/1000,Pt(i,2)/1000,Pt(i,3)/1000);
    Pt_tr = tr*endEffector;
    if Pt_tr(1,3) < 0.25
        if (abs(Pt(1,1)-endEffector(1,4)) < 0.010) || (abs(Pt(1,2)-endEffector(2,4)) < 0.010)
            disp('collision');
        end
    end
    %         planeNormal = [Pt_tr(1,4)*endEffector(1,4) Pt_tr(2,4)*endEffector(2,4) Pt_tr(3,4)*endEffector(3,4)]/norm([Pt_tr(1,4)*endEffector(1,4) Pt_tr(2,4)*endEffector(2,4) Pt_tr(3,4)*endEffector(3,4)]);
    %         pointOnPlane = double([Pt_tr(1,4) Pt_tr(2,4) Pt_tr(3,4)]);
    %         [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)
    %         if check == 1
    %             disp('collision');
    %             disp(intersectionPoint);
    %         end
    
end
%planeNormal
%     for x = 1:length(X)
%         for y = 1:length(Y)
%             for z = 1:length(Z(:))
%                 planeNormal = [1 0 0];
%                 %pointOnPlane
%                 pointOnPlane = double([X(x,1) Y(y,1) Z(z,1)]);
%                 [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine);
%                 if check == 1
%                     disp('collision');
%                     disp(intersectionPoint);
%                 end
%             end
%         end
%     end








%