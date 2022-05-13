classdef Environment < handle
    % Private variables for Environment class
    properties (Access = private)
        pose;
        poseUpdate;
        vertexCount;
        enMesh;
        enVertices;
        modelName;
        
        vertices;
        face;
        faceNormals;
    end
    % Public functions for usage
    methods
        % Class constructor
        function self = Environment(modelName,pose)
            self.SetPose(pose);
            self.modelName = modelName;
        end
        % Setter for environment pose
        function SetPose(self,pose)
            self.pose = pose;
        end
        % Getter for environment pose
        function pose = GetPose(self)
            pose = self.pose;
        end
        % Function for plotting the environment
        function PlotModel(self)
            hold on;
            [self.face,self.vertices,data] = plyread(self.modelName,'tri');
            self.vertexCount = size(self.vertices,1);
            midPoint = sum(self.vertices)/self.vertexCount;
            self.enVertices = self.vertices - repmat(midPoint,self.vertexCount,1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.enMesh = trisurf(self.face, self.enVertices(:,1), self.enVertices(:,2), self.enVertices(:,3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
            self.poseUpdate = [self.pose * [self.enVertices,ones(self.vertexCount,1)]']';
            self.enMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
        
        function MoveModel(self,goalPose)
            self.SetPose(goalPose);
            self.poseUpdate = [self.pose * [self.enVertices,ones(self.vertexCount,1)]']';
            self.enMesh.Vertices = self.poseUpdate(:,1:3);
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