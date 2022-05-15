classdef TesturCollisionDetection < handle
    properties
        imageSubcriber
        jointStatesSubscriber
        rows = 720
        cols = 1280
        d_ur
        a_ur
        alpha_ur
        num_image = 1;
        pClouds_mask
        qmatrix
        robot
        A
        B
        centrePoints
        radiis
        obstaclePoints
        
    end
    methods
        function self = TesturCollisionDetection(robot)
            self.robot = robot;
            [d_1 d_2 d_3 d_4 d_5 d_6] = self.robot.model.links.d;
            self.d_ur = [d_1 d_2 d_3 d_4 d_5 d_6];
            [a_1 a_2 a_3 a_4 a_5 a_6] = self.robot.model.links.a;
            self.a_ur = [a_1 a_2 a_3 a_4 a_5 a_6];
            [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6] = robot.model.links.alpha;
            self.alpha_ur = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];
            self.centrePoints = zeros(self.robot.model.n+1,3);
            self.radiis = zeros(self.robot.model.n+1,3);
            self.radiis(1,:) = [0.10,0.10,0.3];
            self.radiis(2,:) = [0.10,0.10,0.3];
            self.radiis(3,:) = [0.5,0.10,0.10];
            self.radiis(4,:) = [0.5,0.10,0.10];
            self.radiis(5,:) = [0.10,0.10,0.10];
            self.radiis(6,:) = [0.10,0.10,0.10];
            self.radiis(7,:) = [0.10,0.10,0.10];
            self.centrePoints(1,:) = [0,0,0];
            self.centrePoints(2,:) = [0,0,0];
            self.centrePoints(3,:) = [self.radiis(3,1)/2,0,self.radiis(3,3)];
            self.centrePoints(4,:) = [self.radiis(3,1)/2,0,0];
            self.centrePoints(5,:) = [0,0,0];
            self.centrePoints(6,:) = [0,0,0];
            self.centrePoints(7,:) = [0,0,0];
            
%             self.jointStatesCallback();
%             self.imageSubcriber = rossubscriber('/camera/aligned_depth_to_color/image_raw_throttle',@self.imageCallback);
%                         self.imageSubcriber = rossubscriber('/camera/aligned_depth_to_color/image_raw');
%                         pause(0.4);
%                         self.imageCallback();
        end
        
        function setEllipsoid(self)
            X = zeros(21,21,self.robot.model.n+1);
            Y = zeros(21,21,self.robot.model.n+1);
            Z = zeros(21,21,self.robot.model.n+1);
            for i = 1:self.robot.model.n+1
                [X(:,:,i),Y(:,:,i),Z(:,:,i)] = ellipsoid( self.centrePoints(i,1), self.centrePoints(i,2), self.centrePoints(i,3), self.radiis(i,1), self.radiis(i,2), self.radiis(i,3) );
            end
            for i = 1:self.robot.model.n+1
                self.robot.model.points{i} = [reshape(X(:,:,i),[],1),reshape(Y(:,:,i),[],1),reshape(Z(:,:,i),[],1)];
                warning off
                self.robot.model.faces{i} = delaunay(self.robot.model.points{i});
                warning on;
            end
            
        end
        
        
        function getPointClouds(self)
            msg_array = ImageStorage(self.rows,self.cols,self.num_image);
            depthImage = readImage(self.imageSubcriber.LatestMessage);
            msg_array.addDepthImage(double(depthImage),self.num_image);
            pc = PointCloud();
            pc.setExtrinsic(918.7401,918.3084,647.22,345.83,720,1280);
            testpc = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(1)));
            pClouds = zeros(length(testpc),3,self.num_image);
            for i = 1:self.num_image
                pClouds(:,:,i) = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(i)));
            end
            X_cor(:,1) = pClouds(:,1,1);
            Y_cor(:,1) = pClouds(:,2,1);
            Z_w(:,1) = pClouds(:,3,1);
            self.pClouds_mask = [X_cor(:), Y_cor(:), Z_w(:)]/1000; 
        end
        function plotPointCloud(self)
            hold off
            pcshow(self.pClouds_mask(:,:));
        end
        function combineObstaclePoints(self)
            tr = self.robot.model.fkine(self.qmatrix(1,:));
            pointsAndOnes = [tr * [self.pClouds_mask,ones(size(self.pClouds_mask,1),1)]']';
            cubePoints = pointsAndOnes(:,1:3);
            self.setObstaclePoints(cubePoints);
        end
        function [col_array] = checkCollision(self,qmatrix)
            self.setJointStates(qmatrix);
            col_array = zeros(length(self.qmatrix(:,1)),1);
%             self.combineObstaclePoints();
            points2Check = self.getObstaclePoints();
            for q = 1 : length(self.qmatrix(:,1))
                col_flag = false;
                tr = zeros(4,4,self.robot.model.n+1);
                tr(:,:,1) = self.robot.model.base;
                for i = 1:self.robot.model.n
                    tr(:,:,i+1) = tr(:,:,i) * trotz(self.qmatrix(q,i)) * transl(self.a_ur(i), 0, self.d_ur(i)) * trotx(self.alpha_ur(i));                   
                end
                for i = 1: size(tr,3)     
                    pointsAndOnes = [inv(tr(:,:,i)) * [points2Check,ones(size(points2Check,1),1)]']';
                    updatedPoints = pointsAndOnes(:,1:3);
                    algebraicDist = self.GetAlgebraicDist(updatedPoints, self.centrePoints(i,:), self.radiis(i,:));
                    pointsInside = find(algebraicDist < 1);
%                     display(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                    if pointsInside > 10
                        col_flag = true;
                    end
                end
                if col_flag == true
                    col_array(q) = 1;
                end
            end
        end
 
        function [updatedPoints] = getObstaclePoints(self)
            updatedPoints = self.obstaclePoints;
        end
        
        function setObstaclePoints(self,points)
            self.obstaclePoints = [self.obstaclePoints;points];
        end
          
        function setJointStates(self,qmatrix_)
%             q0 = [pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0];
%            
%             q1 = deg2rad([180 -45 90 -135 -90 0]);
%             Tr = [0.9985 -0.0094 -0.0531 0.5938;
%                 -0.0096 -0.9999 -0.0034 -0.0014
%                 -0.0530 0.0039 -0.9986 0.3474
%                 0 0 0 1.0000];
%             
%             q2 = self.robot.model.ikcon(Tr,q0);
            
            % q2 = [0.0000 -0.5946 -5.1732 4.1970 4.7124 6.2832];
            self.qmatrix = qmatrix_;
        end
        

        function algebraicDist = GetAlgebraicDist(self,points, centerPoint, radii)
            
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end