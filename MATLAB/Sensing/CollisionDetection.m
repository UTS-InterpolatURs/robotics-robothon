%%
clc;
clf;
cla;
rosshutdown();
rosinit();
%%
while true
    if true
    depthMessage = rossubscriber('/camera/depth/image_rect_raw');
    pause(0.3);
    depth = readImage(depthMessage.LatestMessage);
    
    end
    
    
    fdx = 420.3378;
    fdy = 422.5609;
    
    u0 = 424;
    v0 = 240;
    
    h = 480;
    w = 848;
    
    u = repmat(1:w,[h,1]); 
    v = repmat(1:h,[w,1])';
    
    
    Z = depth;
    X = (double(Z(:)).*(u(:)-u0))/fdx;
    Y = (double(Z(:)).*(v(:)-v0))/fdy;
    Pt = [X(:),Y(:),double(Z(:))];
    ptCloudOut = pcdownsample(pointCloud(Pt),'gridAverage',0.1);
    PointCloud_raw = ptCloudOut.Location;
    plot3(Pt(:,1),Pt(:,2),Pt(:,3));
    
    
    
    
    
    
end
%