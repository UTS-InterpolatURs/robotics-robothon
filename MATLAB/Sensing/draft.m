clc;
clf;
rosshutdown();
rosinit();

while true
if true
    A = rossubscriber('/camera/color/image_raw');
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
redChannel1Min = 0.934;
redChannel1Max = 0.046;

% Define thresholds for channel 2 based on histogram settings
redChannel2Min = 0.588;
redChannel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
redChannel3Min = 0.000;
redChannel3Max = 1.000;

% Create mask based on chosen histogram thresholds
redSliderBW = ( (I(:,:,1) >= redChannel1Min) | (I(:,:,1) <= redChannel1Max) ) & ...
    (I(:,:,2) >= redChannel2Min ) & (I(:,:,2) <= redChannel2Max) & ...
    (I(:,:,3) >= redChannel3Min ) & (I(:,:,3) <= redChannel3Max);
BWRed = redSliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWRed,[1 1 3])) = 0;


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
[BRed, L] = bwboundaries(BWRed,'noholes');
BredFiltered = Filter(BRed);
% BredFiltered = BRed;
hold on
for k = 1:length(BredFiltered)
   boundary = BredFiltered{k};
   plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
end
hold on
DrawRect(L);
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
