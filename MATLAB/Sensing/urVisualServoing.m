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
            self.vsSubscriber =  rossubscriber('/ColourDetectionChatter_throttle',@self.subscriberCallBackVS);
            self.jointStatesSubscriber = rossubscriber('/joint_states_throttle',@self.jointStatesCallback,"DataFormat","struct");
            %             self.jointStatesSubscriber = rossubscriber('/joint_states',"DataFormat","struct");
            self.jointPublisher = rospublisher('/desired_joint_state', 'sensor_msgs/JointState');
            self.hist_q = zeros(1,6);
        end
        function subscriberCallBackVS(self,~,msg)
            if self.hist_q ~= self.current_q
                %             pause(3)
                %             self.current_q = self.jointStatesSubscriber.LatestMessage.position;
                %             self.robot.model.animate(self.current_q);
                
                vc = msg.Data;
                %             print(vc)
                vcTemp = vc(1:6,1);
                try
                    J = self.robot.model.jacobn(self.current_q);
                catch
                    J = ones(6,6);
                end
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
                self.new_q = self.current_q + (1/self.fps)*qp';
                self.sendJointTrajectory();
                self.hist_q = self.current_q;
            end
            
            
            %             vcTr = transl(vc(1,1),vc(2,1),vc(3,1))*rpy2tr(vc(4,1),vc(5,1),vc(6,1));
            %             tr = self.robot.model.fkine(self.robot.model.getpos()); %% change to camera pose
            %
            %             self.newTr = tr*vcTr;
        end
        function jointStatesCallback(self,~,msg)
            
            % display (msg.Data)
            
            currentJointState_321456 = (msg.position); % Note the default order of the joints is 3,2,1,4,5,6
            %             disp(msg.position)
            
            self.current_q = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
            %             disp(self.current_q)
            self.robot.model.animate(self.current_q);
            
            %             test = isalmost(self.robot.model.getpos(),self.current_joint_states.Position, 0.001);
            %             if all(test) == 0
            %                 %self.robot.model.animate(self.current_joint_states.Position);
            %                 drawnow();
            %                 %self.callback_counter = 0;
            %             end
            %             self.callback_counter = self.callback_counter + 1;
            %            disp(self.callback_counter);
            %
            %
            %             display(test);
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