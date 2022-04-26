classdef Workbench < handle
    % Private variables for Environment class
    properties (Access = private)
        pose;
        poseUpdate;
        vertexCount;
        wbMesh;
        wbVertices;
        
        vertices;
        face;
        faceNormals;
    end
    % Public functions for usage
    methods
        % Class constructor
        function self = Workbench()
            wbPose = transl(0,-1.2,-0.423) * rpy2tr(0,0,0);
            self.SetWbPose(wbPose);
        end
        % Setter for environment pose
        function SetWbPose(self,wbPose)
            self.pose = wbPose;
        end
        % Getter for environment pose
        function wbPose = GetWbPose(self)
            wbPose = self.pose;
        end
        % Function for plotting the environment
        function PlotWorkbench(self)
%             surf([-3,-3;3,3],[-2.5,2;-2.5,2],[-0.316,-0.306;-0.316,-0.306],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            hold on;
            [self.face,self.vertices,data] = plyread('Workbench.ply','tri');
            self.vertexCount = size(self.vertices,1);
            midPoint = sum(self.vertices)/self.vertexCount;
            self.wbVertices = self.vertices - repmat(midPoint,self.vertexCount,1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.wbMesh = trisurf(self.face, self.wbVertices(:,1), self.wbVertices(:,2), self.wbVertices(:,3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
            self.poseUpdate = [self.pose * [self.wbVertices,ones(self.vertexCount,1)]']';
            self.wbMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
        % Function for getting model vertices, face, face normals
        function [vertices,face,faceNormals] = GetModelVFNorm(self)
            vertices = self.vertices;
            face = self.face;
            
            faceNormals = zeros(size(face,1),3);
            for faceIndex = 1:size(face,1)
                v1 = vertices(face(faceIndex,1)',:);
                v2 = vertices(face(faceIndex,2)',:);
                v3 = vertices(face(faceIndex,3)',:);
                faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
        end
    end
end