classdef UTS_UR10 < handle
    properties
        %> Robot model
        model;
        eStopStatus;

        %> workspace
        workspace = [-3 3 -3 3 -0.91 4];
        toolOffset = 0.275354;
        neutralQ;
        neutralPose;
        realSenseTf;
        gripperTf
    end

    properties (Access = private)
        openFaceData;
        openVertexData;
        openPlyData;
        closeFaceData;
        closeVertexData;
        closePlyData;
    end

    methods%% Class for UR10 robot simulation
        function self = UTS_UR10(basePose)
            if nargin < 1
                basePose = transl(0,0,0) * rpy2tr(0,0,0); %> Default base pose when no arguments is inputted
            end
            self.GetUR10Robot();
            self.model.base = basePose;
            campos([-4.5 -2.5 3.0]);

            self.realSenseTf = transl(0.10831,0,0.04361) * rpy2tr(0,deg2rad(-20),pi/2);
            self.gripperTf = transl(0,0,0.12256);
            self.model.tool = self.gripperTf;
            self.PlotAndColourRobot();
            self.eStopStatus = 0;
            self.neutralQ = [-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi/4];
            self.neutralPose = transl(-0.1639, -0.6873, 0.5278) * trotx(pi);
        end

        %% GetUR10Robot
        % Given a name (optional), create and return a UR10 robot model
        function GetUR10Robot(self)
            pause(0.001);
            name = ['UR_10_',datestr(now,'yyyymmddTHHMMSSFFF')];
            handles = findobj('Tag', name);
            if ~isempty(handles)
                error("Existed a robot with the same name");
            end

            L1 = Link('d',0.128,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset', 0);
            L2 = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',0); % was 'offset',pi/2
            L3 = Link('d',0,'a',-0.5716,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);
            L4 = Link('d',0.16389,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0); % was 'offset',pi/2
            L5 = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            L6 = Link('d',0.09037,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset', (pi+pi/4)); %(pi+pi/4)

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);

            [self.openFaceData, self.openVertexData, self.openPlyData] = plyread('UTS_UR10Link6_GripperOpen.ply', 'tri');
            [self.closeFaceData, self.closeVertexData, self.closePlyData] = plyread('UTS_UR10Link6_GripperClose.ply', 'tri');
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n - 1
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UTS_UR10Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            self.model.faces{self.model.n + 1} = self.openFaceData;
            self.model.points{self.model.n + 1} = self.openVertexData;

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            self.model.animate([-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi/4]);
        end

        function SetGripperState(self, args)
            arguments
                self;
                args.gripperState;
            end
%             if (args.gripperState == 0)
%                 gripperVertexData = self.openVertexData;
%                 gripperFaceData = self.openFaceData;
%             elseif (args.gripperState == 1)
%                 gripperVertexData = self.closeVertexData;
%                 gripperFaceData = self.closeFaceData;
%             end
                        
            gripperVertexData = LinearInterp(self.openVertexData, self.closeVertexData, args.gripperState);
            gripperFaceData = LinearInterp(self.openFaceData, self.closeFaceData(1:end-4,:), args.gripperState);

            self.model.points{self.model.n + 1} = gripperVertexData;
            self.model.faces{self.model.n + 1} = gripperFaceData;
            handles = findobj('Tag', self.model.name);
            h = get(handles, 'UserData');
            h.link(self.model.n+1).Children.Vertices = self.model.points{self.model.n + 1};

            hold on;
            self.model.animate(self.model.getpos());
            drawnow();
        end

        function endEffPose = GetEndEffPose(self)
            endEffPose = self.model.fkine(self.model.getpos());
            endEffPose = endEffPose * transl(0,0,self.toolOffset);
        end

        function goalPoseAdjusted = GetGoalPose(self, goalPose)
            goalPoseAdjusted = goalPose * transl(0,0,-self.toolOffset);
        end

        function pose = GlobalToEndEffPose(self, globalPose)
            pose = globalPose * trotx(pi);
        end
        
        function pose = GetCamPose(self)
            
        end
    end
end