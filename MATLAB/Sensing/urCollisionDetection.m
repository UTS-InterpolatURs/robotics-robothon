classdef urCollisionDetection < handle
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
    end
    methods
        function self = urCollisionDetection(robot)
            self.robot = robot;
            [d_1 d_2 d_3 d_4 d_5 d_6] = self.robot.model.links.d;
            self.d_ur = [d_1 d_2 d_3 d_4 d_5 d_6];
            [a_1 a_2 a_3 a_4 a_5 a_6] = self.robot.model.links.a;
            self.a_ur = [a_1 a_2 a_3 a_4 a_5 a_6];
            [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6] = robot.model.links.alpha;
            self.alpha_ur = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];
            self.jointStatesCallback();
%             self.imageSubcriber = rossubscriber('/camera/aligned_depth_to_color/image_raw_throttle',@self.imageCallback);
                        self.imageSubcriber = rossubscriber('/camera/aligned_depth_to_color/image_raw');
                        pause(0.4);
                        self.imageCallback();
        end
        function imageCallback(self,~,msg)
            msg_array = ImageStorage(self.rows,self.cols,self.num_image);
                        depthImage = readImage(self.imageSubcriber.LatestMessage);
%             depthImage = readImage(msg);
            msg_array.addDepthImage(double(depthImage),self.num_image);
            pc = PointCloud();
            pc.setExtrinsic(918.7401,918.3084,647.22,345.83,720,1280);
            testpc = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(1)));
            pClouds = zeros(length(testpc),3,self.num_image);
            for i = 1:self.num_image
                pClouds(:,:,i) = rmmissing(pc.getUR10PointCloud(msg_array.getDepthImage(i)));
            end
            %             shortest_dist = 1000;
            %             for i = 1:self.num_image
            %                 for j = 1:length(pClouds(:,1,1))
            %                     if pClouds(j,3,i) > 1000
            %                         pClouds(j,3,i) = nan;
            %                     elseif pClouds(j,3,i) < 300
            %                         pClouds(j,3,i) = nan;
            %                     elseif pClouds(j,3,i) < shortest_dist
            %                         shortest_dist = pClouds(j,3,i);
            %
            %                     end
            %                 end
            %             end
            %             for i = 1:self.num_image
            %                 for j = 1:length(pClouds(:,1,1))
            %                     if pClouds(j,3,i) > shortest_dist + 300
            %                         pClouds(j,3,i) = nan;
            %                     end
            %                 end
            %             end
            
            %             for i = 1:length(pClouds(:,1,1))
            %                 %                 X_cor(i,1) = pClouds(i,1,1) + pClouds(i,1,1) + pClouds(i,1,1)/3;
            %                 %                 Y_cor(i,1) = pClouds(i,2,1) + pClouds(i,2,1) + pClouds(i,2,1)/3;
            %                 %                 Z_w(i,1) = pClouds(i,3,1) + pClouds(i,3,1) + pClouds(i,3,1)/3;
            %                 X_cor(i,1) = pClouds(i,1,1);
            %                 Y_cor(i,1) = pClouds(i,2,1);
            %                 Z_w(i,1) = pClouds(i,3,1);
            %             end
            X_cor(:,1) = pClouds(:,1,1);
            Y_cor(:,1) = pClouds(:,2,1);
            Z_w(:,1) = pClouds(:,3,1);
            self.pClouds_mask = [X_cor(:), Y_cor(:), Z_w(:)]/1000;
            endEffector = self.robot.model.fkine(self.qmatrix(1,:));
            for c = 1: length(self.pClouds_mask(:,1))
                if ~isnan(self.pClouds_mask(c,3))
                    %                     temp_cor = transl(self.pClouds_mask(c,1),self.pClouds_mask(c,2),self.pClouds_mask(c,3)); %takes lots of time to execute
                    temp_cor = [1 0 0 self.pClouds_mask(c,1) ;
                        0 1 0 self.pClouds_mask(c,2);
                        0 0 1 self.pClouds_mask(c,3);
                        0 0 0 1];
                    tr_cor = endEffector*temp_cor;
                    self.pClouds_mask(c,1) = tr_cor(1,4);
                    self.pClouds_mask(c,2) = tr_cor(2,4);
                    self.pClouds_mask(c,3) = tr_cor(3,4);
                end
            end
%             self.plotPointCloud();
        end
        
        function plotPointCloud(self)
            hold off
            pcshow(self.pClouds_mask(:,:));
        end
        
        function jointStatesCallback(self,~,msg)
            q0 = [pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0];
            
            % q1 = deg2rad([0 -45 45 -90 -90 0]);
            Tr = [0.9985 -0.0094 -0.0531 0.5938;
                -0.0096 -0.9999 -0.0034 -0.0014
                -0.0530 0.0039 -0.9986 0.3474
                0 0 0 1.0000];
            
            q2 = self.robot.model.ikcon(Tr,q0);
            
            % q2 = [0.0000 -0.5946 -5.1732 4.1970 4.7124 6.2832];
            self.qmatrix = [q0];
        end
        
        function [count] = getCollisionStatus(self)
            count = 0;
            
            for q = 1 : length(self.qmatrix(:,1))
                endEffector = self.robot.model.fkine(self.qmatrix(q,:));
                disp(endEffector);
                centerPoint = [endEffector(1,4),endEffector(2,4),endEffector(3,4)-0.3];
                
                radii = [0.1,0.1,0.1];
                [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
                hold on
                ellipsoidAtOrigin_h = surf(X,Y,Z);
                
                cubePointsAndOnes = [inv(endEffector) * [self.pClouds_mask,ones(size(self.pClouds_mask,1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                self.robot.PlotAndColourRobot();
                self.robot.model.animate(self.qmatrix(q,:));
                hold on;
%                 cubeAtOigin_h = plot3(updatedCubePoints(:,1),updatedCubePoints(:,2),updatedCubePoints(:,3),'r.');
                cubeAtOigin_h = plot3(self.pClouds_mask(:,1),self.pClouds_mask(:,2),self.pClouds_mask(:,3),'r.');
%                 algebraicDist = self.GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
                algebraicDist = self.GetAlgebraicDist(self.pClouds_mask, centerPoint, radii);
                disp(algebraicDist(1));
%                 pointsInside = find(algebraicDist < 0);
                for i = 1: length(algebraicDist)
                    if algebraicDist(i) < 1
                        count = count + 1;
                    end
                end
                %                 point1OnLine = [endEffector(1,4) endEffector(2,4) endEffector(3,4)];
                %                 joinNumber = 5;
                %                 jointState = self.robot.model.base;
                %                 for j = 1:joinNumber
                %                     joinState = jointState * trotz(self.qmatrix(q,j)) * transl(self.a_ur(1,j), 0, self.d_ur(1,j)) * trotx(self.alpha_ur(1,j));
                %                 end
                %                 point2OnLine = [joinState(1,4) joinState(2,4) joinState(3,4)];
                %
                %
                %                 for c = 1: length(self.pClouds_mask(:,1))
                %                     if ~isnan(self.pClouds_mask(c,3))
                %                         try
                %                             p1 = [self.pClouds_mask(c,1) self.pClouds_mask(c,2) self.pClouds_mask(c,3)];
                %                             p2 = [self.pClouds_mask(c+1,1) self.pClouds_mask(c+1,2) self.pClouds_mask(c+1,3)];
                %                             p3 = [self.pClouds_mask(c+2,1) self.pClouds_mask(c+2,2) self.pClouds_mask(c+2,3)];
                %                             % Three points on the triangle are in verts
                %                             triangleNormal = cross((p1-p2),(p2-p3));
                %                             triangleNormal = triangleNormal / norm(triangleNormal);
                %                             trianglePoint = (p1+p2+p3)/3;
                %                             [intersectionPoint,check] = LinePlaneIntersection(triangleNormal,trianglePoint,point1OnLine,point2OnLine);
                %                             if check == 1
                %                                 count = count +1;
                %                             end
                %                         end
                %                     end
                %                 end
                
                
            end
        end
        function algebraicDist = GetAlgebraicDist(self,points, centerPoint, radii)
            
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end