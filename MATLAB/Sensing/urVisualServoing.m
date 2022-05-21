classdef urVisualServoing <handle
    properties
    vsSubscriber
    robot
    msg
    newTr
    q
    fps = 1
    end
    methods
        function self = urVisualServoing(robot)
            self.robot = robot;
            self.vsSubscriber =  rossubscriber('/ColourDetectionChatter_throttle',@self.subscriberCallBackVS);  
        end
        function subscriberCallBackVS(self,~,msg)
            vc = msg.Data;
%             print(vc)
            vcTemp = vc(1:6,1);
            J = self.robot.model.jacobn(self.robot.model.getpos());
            Jinv = pinv(J);
            % get joint velocities
            qp = Jinv*vcTemp;
            ind=find(qp>pi);
            if ~isempty(ind)
                qp(ind)=pi;
            end
            ind=find(qp<-pi);
            if ~isempty(ind)
                qp(ind)=-pi;
            end
            self.q = self.robot.model.getpos() + (1/self.fps)*qp';
            
            
%             vcTr = transl(vc(1,1),vc(2,1),vc(3,1))*rpy2tr(vc(4,1),vc(5,1),vc(6,1));
%             tr = self.robot.model.fkine(self.robot.model.getpos()); %% change to camera pose
%             
%             self.newTr = tr*vcTr;
        end
        function [tr] = getVsTr(self)
            tr = self.robot.model.fkine(self.q);
        end
        function [q] = getJointVelocities(self)
            q = self.q;
        end
    
    end
end