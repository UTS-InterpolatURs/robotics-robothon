classdef urVisualServoing <handle
    properties
        vsSubscriber
        robot
        newTr
        fps = 3
        jointStatesSubscriber
        current_q
        jointPublisher
        new_q
        hist_q
    end
    methods
        function self = urVisualServoing(robot)
            self.robot = robot;
            self.vsSubscriber =  rossubscriber('/ColourDetectionChatter',@self.subscriberCallBackVS);
            self.jointStatesSubscriber = rossubscriber('/joint_states_throttle',@self.jointStatesCallback,"DataFormat","struct");
            self.jointPublisher = rospublisher('/desired_joint_state', 'sensor_msgs/JointState');
            self.hist_q = zeros(1,6);
        end
        function subscriberCallBackVS(self,~,msg)
            if self.hist_q ~= self.current_q           
                vc = msg.Data;
                vcTemp = vc(1:6,1);
                
                try
                    J = self.robot.model.jacobn(self.current_q);
                catch
                    J = ones(6,6);
                end
                Jinv = pinv(J);
                % Get error q
                qp = Jinv*vcTemp;
                %Maximum angular velocity cannot exceed 180 degrees/s
                ind=find(qp>pi);
                if ~isempty(ind)
                    qp(ind)=pi;
                end
                ind=find(qp<-pi);
                if ~isempty(ind)
                    qp(ind)=-pi;
                end
                %Update joints 
                self.new_q = self.current_q + (1/self.fps)*qp';
                % Send joints
                self.sendJointTrajectory();
                self.hist_q = self.current_q;
            end
            
        end
        function jointStatesCallback(self,~,msg)
                       
            currentJointState_321456 = (msg.position); % Note the default order of the joints is 3,2,1,4,5,6
            
            self.current_q = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
            %             disp(self.current_q)
            self.robot.model.animate(self.current_q);
            
        end
        
        function sendJointTrajectory(self)
            pause(0.2);
            msg = rosmessage(self.jointPublisher);
            
            msg.Position =self.getJointVelocities;
            send(self.jointPublisher, msg);
        end
        
        function [tr] = getVsTr(self)
            tr = self.robot.model.fkine(self.new_q);
        end
        function [q] = getJointVelocities(self)
            q = self.new_q;
        end
        
        
        
    end
end