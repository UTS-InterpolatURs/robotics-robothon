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
            % Set link parameters for the robots
            [d_1 d_2 d_3 d_4 d_5 d_6] = self.robot.model.links.d;
            self.d_ur = [d_1 d_2 d_3 d_4 d_5 d_6];
            [a_1 a_2 a_3 a_4 a_5 a_6] = self.robot.model.links.a;
            self.a_ur = [a_1 a_2 a_3 a_4 a_5 a_6];
            [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6] = robot.model.links.alpha;
            self.alpha_ur = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];
            self.centrePoints = zeros(self.robot.model.n+1,3);
            self.radiis = zeros(self.robot.model.n+1,3);
            % Set volume and centre points for the elipsoids 
            self.radiis(1,:) = [0.10,0.10,0.3];
            self.radiis(2,:) = [0.10,0.10,0.3];
            self.radiis(3,:) = [0.5,0.10,0.10];
            self.radiis(4,:) = [0.5,0.10,0.10];
            self.radiis(5,:) = [0.10,0.10,0.10];
            self.radiis(6,:) = [0.10,0.10,0.10];
            self.radiis(7,:) = [0.10,0.10,0.30];
            self.centrePoints(1,:) = [0,0,0];
            self.centrePoints(2,:) = [0,0,0];
            self.centrePoints(3,:) = [self.radiis(3,1)/2,0,self.radiis(3,3)*2];
            self.centrePoints(4,:) = [self.radiis(3,1)/2,0,0];
            self.centrePoints(5,:) = [0,0,0];
            self.centrePoints(6,:) = [0,0,0];
            self.centrePoints(7,:) = [0,0,0];
        end

        function drawEllipsoid(self)
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
            self.robot.model.plot3d(self.robot.model.getpos());

        end
        function [points] = getLightcurtain(self)
            [Y,Z] = meshgrid(-1:0.1:1,-1:0.1:1);
            sizeMat = size(Y);
            X = repmat(1,sizeMat(1),sizeMat(2));
            points = [X,Y,Z]
        end

        function [check] = lightCurtainDetected(self)
            check = 0;
            points2Check = self.getObstaclePoints();
            col_flag = false;
            centrePoints_ = [0,-1.2,0];
            radi = [1,0.01,1];
            algebraicDist = self.GetAlgebraicDist(points2Check, centrePoints_, radi);
            pointsInside = find(algebraicDist < 1);
            if pointsInside > 0
                col_flag = true;
            end

            if col_flag == true
                check = 1;
            end

        end

        function generatePointClouds(self)
            % subcribe depth information
            self.imageSubcriber = rossubscriber('/camera/aligned_depth_to_color/image_raw');
            pause(0.4);
            msg_array = ImageStorage(self.rows,self.cols,self.num_image);
            depthImage = readImage(self.imageSubcriber.LatestMessage);
            % Add depth image to a storage
            msg_array.addDepthImage(double(depthImage),self.num_image);
            pc = PointCloud();
            pc.setExtrinsic(918.7401,918.3084,647.22,345.83,720,1280);
            % Get point clouds
            testpc = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(1)));
            pClouds = zeros(length(testpc),3,self.num_image);
            % Filter point clouds
            for i = 1:self.num_image
                pClouds(:,:,i) = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(i)));
            end
            X_cor(:,1) = pClouds(:,1,1);
            Y_cor(:,1) = pClouds(:,2,1);
            Z_w(:,1) = pClouds(:,3,1);
            self.pClouds_mask = [X_cor(:), Y_cor(:), Z_w(:)]/1000;
            tr = self.robot.model.fkine(self.robot.model.getpos()); %% change to camera pose
            % Transform to robot base 
            pointsAndOnes = [tr * [self.pClouds_mask,ones(size(self.pClouds_mask,1),1)]']';
            self.pClouds_mask = pointsAndOnes(:,1:3);
            % Add point cloud as obstacle 
            self.setObstaclePoints(self.pClouds_mask);
        end
        function [pcs] = getPointClouds(self)
            pcs = self.pClouds_mask;
        end
        function plotPointCloud(self)
            hold off
            pcshow(self.pClouds_mask(:,:));
        end
        function plotPointCloudwithRobot(self)
            hold on
            pointCloudwithRobot_h = plot3(self.pClouds_mask(:,1),self.pClouds_mask(:,2),self.pClouds_mask(:,3),'r.');

        end

        function [col_array] = checkCollision(self,qmatrix)
            self.setJointStates(qmatrix);
            col_array = zeros(length(self.qmatrix(:,1)),1);
            % Get points to check for collision
            points2Check = self.getObstaclePoints();
            for q = 1 : length(self.qmatrix(:,1))
                col_flag = false;
                tr = zeros(4,4,self.robot.model.n+1);
                tr(:,:,1) = self.robot.model.base;
                for i = 1:self.robot.model.n
                    tr(:,:,i+1) = tr(:,:,i) * trotz(self.qmatrix(q,i)) * transl(self.a_ur(i), 0, self.d_ur(i)) * trotx(self.alpha_ur(i));
                end
                for i = 1: size(tr,3)
                    % Transform object to the robot links/ not moving the
                    % robot to the object
                    pointsAndOnes = [inv(tr(:,:,i)) * [points2Check,ones(size(points2Check,1),1)]']';
                    updatedPoints = pointsAndOnes(:,1:3);
                    algebraicDist = self.GetAlgebraicDist(updatedPoints, self.centrePoints(i,:), self.radiis(i,:));
                    pointsInside = find(algebraicDist < 1);
                    if pointsInside > 0
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
            self.qmatrix = qmatrix_;
        end


        function algebraicDist = GetAlgebraicDist(self,points, centerPoint, radii)

            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end


        function ClearObstaclePoints(self)
            self.obstaclePoints = [50,50,50];
        end
    end
end