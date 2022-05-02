clc;
clf;
clear;

%     rosshutdown();
%     rosinit();
addpath '../UR10e/'
addpath '../'
robot = UTS_UR10;
[d_1 d_2 d_3 d_4 d_5 d_6] = robot.model.links.d;
d_ur10 = [d_1 d_2 d_3 d_4 d_5 d_6];
[a_1 a_2 a_3 a_4 a_5 a_6] = robot.model.links.a;
a_ur10 = [a_1 a_2 a_3 a_4 a_5 a_6];
[alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6] = robot.model.links.alpha;
alpha_ur10 = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];

num_image = 3;
rows = 480;
cols = 848;
msg_array = ImageStorage(rows,cols,num_image);
for i = 1:num_image
    A = rossubscriber('/camera/depth/image_rect_raw');
    pause(0.4);
    depthImage = readImage(A.LatestMessage);
    msg_array.addImage(double(depthImage),i);
end

test = PointCloud();

test.setExtrinsic(422.3378,422.5609,424,240,480,848);
testpc = test.getPointCloud(msg_array.getImage(1));
pClouds = zeros(length(testpc),3,num_image);

for i = 1:num_image
    pClouds(:,:,i) = test.getPointCloud(msg_array.getImage(i));
end

pClouds_mask = pClouds;


q1 = deg2rad([0 -45 45 -90 -90 0]);

% q2 = [0.0000 -0.5946 -5.1732 4.1970 4.7124 6.2832];
qmatrix = [q1];
count = 0;
for q = 1 : length(qmatrix(:,1))
    
    endEffector = robot.model.fkine(qmatrix(q,:));
    point1OnLine = [endEffector(1,4) endEffector(2,4) endEffector(3,4)];
    joinNumber = 5;
    jointState = robot.model.base;
    for j = 1:joinNumber
        joinState = jointState * trotz(qmatrix(q,j)) * transl(a_ur10(1,j), 0, d_ur10(1,j)) * trotx(alpha_ur10(1,j));
    end
    point2OnLine = [joinState(1,4) joinState(2,4) joinState(3,4)];
    
    %create tr_pClouds
    for num_image = 1 : length(pClouds(1,1,:))
        for c = 1: length(pClouds(:,1,num_image))
            temp_cor = transl(pClouds(c,1,num_image),pClouds(c,2,num_image),pClouds(c,3,num_image));
            tr_cor = endEffector*temp_cor;
            pClouds_mask(c,1,num_image) = tr_cor(1,4);
            pClouds_mask(c,2,num_image) = tr_cor(2,4);
            pClouds_mask(c,3,num_image) = tr_cor(3,4);
        end
    end
    
    for num_image = 1 : length(pClouds_mask(1,1,:))
        for c = 1: length(pClouds_mask(:,1,num_image))
            try
                p1 = [pClouds_mask(c,1,num_image) pClouds_mask(c,2,num_image) pClouds_mask(c,3,num_image)];
                p2 = [pClouds_mask(c+1,1,num_image) pClouds_mask(c+1,2,num_image) pClouds_mask(c+1,3,num_image)];
                p3 = [pClouds_mask(c+2,1,num_image) pClouds_mask(c+2,2,num_image) pClouds_mask(c+2,3,num_image)];
                % Three points on the triangle are in verts
                triangleNormal = cross((p1-p2),(p2-p3));
                triangleNormal = triangleNormal / norm(triangleNormal);
                trianglePoint = (p1+p2+p3)/3;
                [intersectionPoint,check] = LinePlaneIntersection(triangleNormal,trianglePoint,point1OnLine,point2OnLine);
                if check ~= 0
                    count = count +1;
                end
            end
        end
    end
    
    %         planeNormal = [1 0 0];
    %
    %                 %pointOnPlane
    %                 pointOnPlane = [3.2 0 0];
    %
    %                 [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)
    
end
