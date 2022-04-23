clc;
clf;
cla;
rosshutdown();
rosinit();

while true
if true
    A = rossubscriber('/head_camera/rgb/image_raw');
    pause(0.4);
    RGB = readImage(A.LatestMessage);
end

% RGB = imread('yellowline.png');


I = rgb2hsv(RGB);


% % Green
% % Define thresholds for channel 1 based on histogram settings
% greenChannel1Min = 0.323;
% greenChannel1Max = 0.546;
% 
% % Define thresholds for channel 2 based on histogram settings
% greenChannel2Min = 0.186;
% greenChannel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% greenChannel3Min = 0.000;
% greenChannel3Max = 1.000;
% 
% greenSliderBW = (I(:,:,1) >= greenChannel1Min ) & (I(:,:,1) <= greenChannel1Max) & ...
%     (I(:,:,2) >= greenChannel2Min ) & (I(:,:,2) <= greenChannel2Max) & ...
%     (I(:,:,3) >= greenChannel3Min ) & (I(:,:,3) <= greenChannel3Max);
% BWGreen = greenSliderBW;
% 
% BGreen = bwboundaries(BWGreen,'noholes');

% Red
% Define thresholds for channel 1 based on histogram settings
blueChannel1Min = 0.571;
blueChannel1Max = 0.779;

% Define thresholds for channel 2 based on histogram settings
blueChannel2Min = 0.116;
blueChannel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
blueChannel3Min = 0.000;
blueChannel3Max = 1.000;

% Create mask based on chosen histogram thresholds
blueSliderBW = ( (I(:,:,1) >= blueChannel1Min) & (I(:,:,1) <= blueChannel1Max) ) & ...
    (I(:,:,2) >= blueChannel2Min ) & (I(:,:,2) <= blueChannel2Max) & ...
    (I(:,:,3) >= blueChannel3Min ) & (I(:,:,3) <= blueChannel3Max);
BWBlue = blueSliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWBlue,[1 1 3])) = 0;


hold on;
imshow(RGB);
% hold on;
% imshow(maskedRGBImage);
% [B,L] = bwboundaries(BWRed,'noholes');
% imshow(label2rgb(L, @jet, [.5 .5 .5]))
% hold on
% for k = 1:length(B)
%    boundary = B{k};
%    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
% end
[BBlue, L] = bwboundaries(BWBlue,'noholes');
% BredFiltered = Filter(BRed);
BblueFiltered = BBlue;
hold on
for k = 1:length(BblueFiltered)
   boundary = BblueFiltered{k};
   plot(boundary(:,2), boundary(:,1), 'b', 'LineWidth', 2)
end
hold on
[X,Y] = DrawRect(L);
%%
fdx = 554.2547; fdy = 554.2547;
u0 = 320.5; v0 = 240.5; h=480; w=640;
if true
    B = rossubscriber('/head_camera/depth_registered/image_raw');
    pause(0.4);
    depthImage = readImage(B.LatestMessage);
end
x = (depthImage(Y,X)*(X-u0))/fdx;
y = -(depthImage(Y,X)*(Y-v0))/fdy;
z = depthImage(Y,X);


try node = ros.Node('/cameraTalker'); end


pub = ros.Publisher(node,'/xyz3D','geometry_msgs/PointStamped');

msg = rosmessage(pub);

msg.Point.X = x;
msg.Point.Y = y;
msg.Point.Z = z;
msg.Header.Seq = 1;
msg.Header.FrameId = 'head_camera_depth_frame';
msg.Header.Stamp = rostime('Now','system');
send(pub,msg);
pause(2);
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
