clc;
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


num_image = 1;
rows = 480;
cols = 848;
msg_array = ImageStorage(rows,cols,num_image);
for i = 1:num_image
    A = rossubscriber('/camera/depth/image_rect_raw');
    pause(0.4);
    depthImage = readImage(A.LatestMessage);
    msg_array.addDepthImage(double(depthImage),i);
end

test = PointCloud();

test.setExtrinsic(422.3378,422.5609,424,240,480,848);
testpc = rmmissing(test.getUR10PointCloud(msg_array.getDepthImage(1)));
pClouds = zeros(length(testpc),3,num_image);

for i = 1:num_image
    pClouds(:,:,i) = rmmissing(test.getUR10PointCloud(msg_array.getDepthImage(i)));
end

for i = 1:num_image
    for j = 1:length(pClouds(:,1,1))
        if pClouds(j,3,i) > 1000
            pClouds(j,3,i) = nan;
        elseif pClouds(j,3,i) < 200
            pClouds(j,3,i) = nan;
        end
    end
end
shortest_dist = 1000;
for i = 1:num_image
    for j = 1:length(pClouds(:,1,1))
        if pClouds(j,3,i) < shortest_dist
            shortest_dist = pClouds(j,3,i);
        end
    end
end

for i = 1:num_image
    for j = 1:length(pClouds(:,1,1))
        if pClouds(j,3,i) > shortest_dist + 500
            pClouds(j,3,i) = nan;
        end
    end
end

% for i = 1:length(pClouds(:,1,1))
%     X_cor(i,1) = pClouds(i,1,1) + pClouds(i,1,2) + pClouds(i,1,3)/3;
%     Y_cor(i,1) = pClouds(i,2,1) + pClouds(i,2,2) + pClouds(i,2,3)/3;
%     Z_w(i,1) = pClouds(i,3,1) + pClouds(i,3,2) + pClouds(i,3,3)/3;
% end
X_cor(:,1) = pClouds(:,1,1);
Y_cor(:,1) = pClouds(:,2,1);
Z_w(:,1) = pClouds(:,3,1);
pClouds_mask = [X_cor(:), Y_cor(:), Z_w(:)]/1000;
q0 = [pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0];

% q1 = deg2rad([0 -45 45 -90 -90 0]);
Tr = [0.9985 -0.0094 -0.0531 0.5938;
    -0.0096 -0.9999 -0.0034 -0.0014
    -0.0530 0.0039 -0.9986 0.3474
    0 0 0 1.0000];

q2 = robot.model.ikcon(Tr,q0);

% q2 = [0.0000 -0.5946 -5.1732 4.1970 4.7124 6.2832];
qmatrix = [q2];
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
    %     for num_image = 1 : length(pClouds(1,1,:))
    %         for c = 1: length(pClouds(:,1,num_image))
    %             temp_cor = transl(pClouds(c,1,num_image),pClouds(c,2,num_image),pClouds(c,3,num_image));
    %             tr_cor = endEffector*temp_cor;
    %             pClouds_mask(c,1,num_image) = tr_cor(1,4);
    %             pClouds_mask(c,2,num_image) = tr_cor(2,4);
    %             pClouds_mask(c,3,num_image) = tr_cor(3,4);
    %         end
    %     end
    % for num_image = 1 : length(pClouds_mask(1,1,:))
    %     for c = 1: length(pClouds_mask(:,1,num_image))
    %         try
    %             p1 = [pClouds_mask(c,1,num_image) pClouds_mask(c,2,num_image) pClouds_mask(c,3,num_image)];
    %             p2 = [pClouds_mask(c+1,1,num_image) pClouds_mask(c+1,2,num_image) pClouds_mask(c+1,3,num_image)];
    %             p3 = [pClouds_mask(c+2,1,num_image) pClouds_mask(c+2,2,num_image) pClouds_mask(c+2,3,num_image)];
    %             % Three points on the triangle are in verts
    %             triangleNormal = cross((p1-p2),(p2-p3));
    %             triangleNormal = triangleNormal / norm(triangleNormal);
    %             trianglePoint = (p1+p2+p3)/3;
    %             [intersectionPoint,check] = LinePlaneIntersection(triangleNormal,trianglePoint,point1OnLine,point2OnLine);
    %             if check ~= 0
    %                 count = count +1;
    %             end
    %         end
    %     end
    % end
    
    for c = 1: length(pClouds_mask(:,1))
        temp_cor = transl(pClouds_mask(c,1),pClouds_mask(c,2),pClouds_mask(c,3));
        tr_cor = endEffector*temp_cor;
        pClouds_mask(c,1) = tr_cor(1,4);
        pClouds_mask(c,2) = tr_cor(2,4);
        pClouds_mask(c,3) = tr_cor(3,4);
    end
    
    
    for c = 1: length(pClouds_mask(:,1))
        try
            p1 = [pClouds_mask(c,1) pClouds_mask(c,2) pClouds_mask(c,3)];
            p2 = [pClouds_mask(c+1,1) pClouds_mask(c+1,2) pClouds_mask(c+1,3)];
            p3 = [pClouds_mask(c+2,1) pClouds_mask(c+2,2) pClouds_mask(c+2,3)];
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
hold off;
pcshow(pClouds_mask(:,:));
hold off;

