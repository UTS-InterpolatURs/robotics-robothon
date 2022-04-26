classdef IntColCompute < handle
    properties (Constant)
        
    end
    
    methods (Static)
        function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end
            
            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;
            
            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        function result = IsIntersectPointInsideTriangle(intersectP,triangleVerts)
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);
            
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            
            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);
            
            D = uv * uv - uu * vv;
            
            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            result = 1;                      % intersectP is in Triangle
        end
        
        function [trRobot] = ComputeJointTransforms(robot,qCurrent)
            robotLinks = robot.model.links;
            trRobot = zeros(4,4,robot.model.n+1);
            trRobot(:,:,1) = robot.model.base;
            for i = 1 : robot.model.n
                trRobot(:,:,i+1) = trRobot(:,:,i) * trotz(qCurrent(i)+robotLinks(i).offset) * transl(0,0,robotLinks(i).d) * transl(robotLinks(i).a,0,0) * trotx(robotLinks(i).alpha);
            end
        end
        
%         function [checkCollision] = CheckCollision(robot, )
%             
%         end
    end
end