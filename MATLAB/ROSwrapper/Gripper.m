classdef Gripper
    %GRIPPER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        gripperSubscriber
        gripperPublisher
    end

    methods
        function self = Gripper()
            self.gripperPublisher = rospublisher('/gripper_desired_joint_states', 'sensor_msgs/JointState');
            %             self.gripperSubscriber = rossubscriber('/gripper_joint_states',@self.jointStatesCallback,"DataFormat","struct");
        end

        function openGripper(self)
            msg = rosmessage(self.gripperPublisher);

            msg.Name = {'left_jaw', 'right_jaw'};
            msg.Position = [-0.5,0.5];

            send(self.gripperPublisher,msg);

        end


        function closeGripper(self)
            msg = rosmessage(self.gripperPublisher);

            msg.Name = {'left_jaw', 'right_jaw'};
            msg.Position = [0.17,-0.17];

            send(self.gripperPublisher,msg);

        end

        function setGripper(self, percentageOpen)
            msg = rosmessage(self.gripperPublisher);
            angle = 0.3 - (0.67 * (percentageOpen/100));
            disp(angle);
            msg.Name = {'left_jaw', 'right_jaw'};
            msg.Position = [angle,-angle];

            send(self.gripperPublisher,msg);

        end
    end
end

