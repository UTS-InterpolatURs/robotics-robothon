%%
clc;
clf;
cla;
% rosshutdown();
% rosinit();

while true
if true
%     A = rossubscriber('/head_camera/rgb/image_raw');
    A = rossubscriber('/camera/color/image_raw');
    
    pause(0.4);
    RGB = readImage(A.LatestMessage);
end

hold on 
imshow(RGB);

Blue = ColourDetection(RGB);
Blue.SetBlue();

% % Define thresholds for channel 1 based on histogram settings
% blueChannel1Min = 0.571;
% blueChannel1Max = 0.779;
% 
% % Define thresholds for channel 2 based on histogram settings
% blueChannel2Min = 0.116;
% blueChannel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% blueChannel3Min = 0.000;
% blueChannel3Max = 1.000;

% Blue.SetChannels(blueChannel1Min,blueChannel1Max,blueChannel2Min,blueChannel2Max,blueChannel3Min,blueChannel3Max);


[X,Y] = Blue.DrawBoundaryAndRectangle();


%%

% fdx = 554.2547; fdy = 554.2547;
% u0 = 320.5; v0 = 240.5; h=480; w=640;
% if true
%     B = rossubscriber('/head_camera/depth_registered/image_raw');
%     pause(0.4);
%     depthImage = readImage(B.LatestMessage);
% end
% x = (depthImage(Y,X)*(X-u0))/fdx;
% y = -(depthImage(Y,X)*(Y-v0))/fdy;
% z = depthImage(Y,X);
% 
% 
% try node = ros.Node('/cameraTalker'); end
% 
% pub = ros.Publisher(node,'/xyz3D','geometry_msgs/PointStamped');
% 
% msg = rosmessage(pub);
% 
% msg.Point.X = x;
% msg.Point.Y = y;
% msg.Point.Z = z;
% msg.Header.Seq = 1;
% msg.Header.FrameId = 'head_camera_depth_frame';
% msg.Header.Stamp = rostime('Now','system');
% send(pub,msg);
% pause(2);


%%
% BRed   = NoiseFilter(BRed);
% BGreen = NoiseFilter(BGreen);


% xRed1   = BRed{1,1}(:,2);
% yRed1   = BRed{1,1}(:,1);
% plot(xRed1, yRed1,'r');
% hold on;
% imshow(maskedRGBImage);


% axis([0 1920 0 1080]);

% xRed2   = BRed{2,1}(:,2);
% yRed2   = BRed{2,1}(:,1);
% plot(xRed2, yRed2,'r');
% 
% xRed3   = BRed{3,1}(:,2);
% yRed3   = BRed{3,1}(:,1);
% plot(xRed3, yRed3,'r');

% xGreen1 = BGreen{1,1}(:,2);
% yGreen1 = BGreen{1,1}(:,1);
% plot(xGreen1, yGreen1,'g');

end
% [xRed1Center, yRed1Center] = Circlefit(xRed1, yRed1); plot(xRed1Center, yRed1Center,'*');
% [xRed2Center, yRed2Center] = Circlefit(xRed2, yRed2); plot(xRed2Center, yRed2Center,'*');
% [xRed3Center, yRed3Center] = Circlefit(xRed3, yRed3); plot(xRed3Center, yRed3Center,'*');
% [xGreen1Center, yGreen1Center] = Circlefit(xGreen1, yGreen1); plot(xGreen1Center, yGreen1Center,'*');
