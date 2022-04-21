classdef Workbench < handle
    % Private variables for Environment class
    properties (Access = private)
        pose;
        poseUpdate;
        vertexCount;
        wbMesh;
        wbVertices;
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
            [face,vertices,data] = plyread('Workbench.ply','tri');
            self.vertexCount = size(vertices,1);
            midPoint = sum(vertices)/self.vertexCount;
            self.wbVertices = vertices - repmat(midPoint,self.vertexCount,1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.wbMesh = trisurf(face, self.wbVertices(:,1), self.wbVertices(:,2), self.wbVertices(:,3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
            self.poseUpdate = [self.pose * [self.wbVertices,ones(self.vertexCount,1)]']';
            self.wbMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
    end
end