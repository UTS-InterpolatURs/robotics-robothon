classdef VCam < handle
    properties
        camera;
        camPose;
        robot;
    end
    
    properties (Access = private)
        focal;
        pixel;
        resolution;
        centre;
        name;
        fps;
        depth;
    end
    
    methods
        function self = VCam(args)
            arguments
                args.robot;
                args.focal;
                args.pixel;
                args.resolution;
                args.centre;
                args.name;
                args.fps;
                args.depth;
            end
            self.robot = args.robot;
            self.focal = args.focal;
            self.pixel = args.pixel;
            self.resolution = args.resolution;
            self.centre = args.centre;
            self.name = args.name;
            self.fps = args.fps;
            self.depth = args.depth;
            
            self.GetVirtualCam();
            self.PlotVirtualCam();
        end
        
        function GetVirtualCam(self)
            % Get end effector pose
            endEffPose = self.robot.model.fkine(self.robot.model.getpos());
            % Set camera pose
            self.camPose = endEffPose * transl(-0.10831,0,0.04361) * rpy2tr(0,deg2rad(19),0);
            % Intrinsic
            self.camera = CentralCamera('focal',self.focal,'pixel',self.pixel,'resolution',self.resolution,'centre',self.centre,'name',self.name);
            % Extrinsic
            self.camera.T = self.camPose;
        end
        
        function PlotVirtualCam(self)           
            self.camera.plot_camera('Tcam',self.camPose,'label','scale',0.04);
        end
        
        function UpdateCamPose(self)
            % Get end effector pose
            endEffPose = self.robot.model.fkine(self.robot.model.getpos());
            % Set camera pose
            self.camPose = endEffPose * transl(-0.10831,0,0.04361) * rpy2tr(0,deg2rad(19),0);
            % Update 
            self.camera.T = self.camPose;
%             drawnow();
        end
        
        function ProjectPointsImg(self, points)
            
        end
    end
end