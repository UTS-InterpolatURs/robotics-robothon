clc;
clf;
try
%     rosshutdown();
    rosinit();
    addpath '../UR10e/'
    addpath '../'
    robot = UTS_UR10;
end
num_image = 10;
msg_array = ImageStorage(480,num_image);
for i = 1:num_image
    A = rossubscriber('/camera/depth/image_rect_raw');
    pause(0.4);
    depthImage = readImage(A.LatestMessage);
    msg_array.addImage(depthImage,i);
end

test = PointCloud();

test.setExtrinsic(422.3378,422.5609,424,240,480,848);

test_pt = test.getPointCloud(msg_array(:,:,5));



