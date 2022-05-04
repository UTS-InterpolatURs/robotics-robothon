classdef UTS_UR10 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-3 3 -3 3 -0.91 4];
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
            self.PlotAndColourRobot();
            campos([-4.5 -2.5 3.0]);
        end
        
        %% GetUR10Robot
        % Given a name (optional), create and return a UR10 robot model
        function GetUR10Robot(self)
            pause(0.001);
            name = ['UR_10_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            L1 = Link('d',0.128,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            L2 = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0); % was 'offset',pi/2
            L3 = Link('d',0,'a',-0.5716,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.16389,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0); % was 'offset',pi/2
            L5 = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.09037,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
            
            [self.openFaceData, self.openVertexData, self.openPlyData] = plyread('UTS_UR10Link6_RG6open.ply', 'tri');
            [self.closeFaceData, self.closeVertexData, self.closePlyData] = plyread('UTS_UR10Link6_RG6close.ply', 'tri');
            length(self.openVertexData)
            length(self.closeVertexData)
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
            self.model.faces{self.model.n + 1} = self.closeFaceData;
            self.model.points{self.model.n + 1} = self.closeVertexData;
            
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
        end
        
        function SetGripperState(self, args)
            arguments
                self;
                args.gripperState;
                args.model;
            end
            
            gripperVertexData = LinearInterp(self.closeVertexData, self.openVertexData, args.gripperState);
            
%             correctTr = eye(4);
%             gripperVertexData = [gripperVertexData, ones(size(gripperVertexData, 1), 1)];
%             gripperVertexData = gripperVertexData * (correctTr^-1)';
%             gripperVertexData = gripperVertexData(:, 1:3);
            
            args.model.points{args.model.n + 1} = gripperVertexData;
            handles = findobj('Tag', args.model.name);
            h = get(handles, 'UserData');
            h.link(args.model.n+1).Children.Vertices = args.model.points{args.model.n + 1};
            
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            drawnow();
        end
    end
end