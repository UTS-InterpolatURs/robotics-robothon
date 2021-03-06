classdef Gripper < handle
    %GRIPPER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        gripperSubscriber
        gripperPublisher
        jointStates
    end

    methods
        function self = Gripper()
            self.gripperPublisher = rospublisher('/gripper_desired_joint_states', 'sensor_msgs/JointState');
            self.gripperSubscriber = rossubscriber('/gripper_joint_states_throttle',@self.jointStatesCallback,"DataFormat","struct");
        end

        function openGripper(self)
            msg = rosmessage(self.gripperPublisher);

            msg.Name = {'left_jaw', 'right_jaw'};
            msg.Position = [-0.65,-0.65];


            send(self.gripperPublisher,msg);

        end

        function openGripperLeft(self)
            msg = rosmessage(self.gripperPublisher);

            msg.Name = {'left_jaw'};
            msg.Position = [-0.65];


            send(self.gripperPublisher,msg);

        end


        function closeGripper(self, closingEffort)
            if ~exist('closingEffort','var') || isempty(closingEffort)
                closingEffort = 100;
            end
            msg = rosmessage(self.gripperPublisher);
            msg.Name = {'right_jaw', 'left_jaw'};
            msg.Position = self.jointStates.Position';
            while 1
                currentJointPos = self.jointStates.Position;
                currentJointEffort = abs(self.jointStates.Effort);

                if currentJointPos(1) >= 0.15 && currentJointPos(2) >= 0.15
                    break;
                end
                if abs(currentJointEffort(1)) >= closingEffort && abs(currentJointEffort(2)) >= closingEffort
                    break;
                end
                if abs(currentJointEffort(1)) < closingEffort 
                    msg.Position(1) = msg.Position(1) + 0.01;
                end
                if abs(currentJointEffort(2)) < closingEffort 
                    msg.Position(2) = msg.Position(2) + 0.01;                
                end


                send(self.gripperPublisher,msg);
                pause(0.05)
            end


        end

        function setGripper(self, percentageOpen)
            msg = rosmessage(self.gripperPublisher);
            angle = 0 - (0.37 * (percentageOpen/100));
            disp(angle);
            msg.Name = {'left_jaw', 'right_jaw'};
            msg.Position = [angle,angle];

            send(self.gripperPublisher,msg);

        end

        function jointStatesCallback(self,~,msg)
            self.jointStates = msg;
        end
    end
end

