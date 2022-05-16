classdef urVisualServoing <handle
    properties
    vsSubscriber
    robot
    msg
    newTr
    end
    methods
        function self = urVisualServoing(robot)
            self.robot = robot;
            self.vsSubscriber =  rossubscriber('/ColourDetectionChatter_throttle');  
        end
        function subscribeVS(self)
            pause(0.5)
            vc = self.vsSubscriber.LatestMessage.Data;          
            vcTr = transl(vc(1,1),vc(2,1),vc(3,1))*rpy2tr(vc(4,1),vc(5,1),vc(6,1));
            tr = self.robot.model.fkine(self.robot.model.getpos()); %% change to camera pose
            self.newTr = tr*vcTr;
        end
        function [tr] = getVsTr(self)
            tr = self.newTr;
        end
    
    end
end