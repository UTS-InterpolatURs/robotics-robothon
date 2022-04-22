classdef urRosWrapper < handle
    %URROSWRAPPER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        jointStatesSubscriber
        wrenchSubscriber
        publisher
        current_joint_states
        wrench
        robot
        jointPublisher
        callback_counter
    end

    methods
        function self = urRosWrapper(robot)
            %URROSWRAPPER Construct an instance of this class
            self.robot = robot;
            self.jointStatesSubscriber = rossubscriber('/joint_states',@self.jointStatesCallback,"DataFormat","struct");
            self.wrenchSubscriber = rossubscriber('/wrench',@self.wrenchCallback,"DataFormat","struct");
            self.jointPublisher = rospublisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', 'control_msgs/FollowJointTrajectoryActionGoal');
            self.callback_counter = 0;
            self.current_joint_states = zeros(1,6);
        end

        function jointStatesCallback(self,~,msg)

            % display (msg.Data)
            self.current_joint_states = msg;
            currentJointState_321456 = (msg.Position)'; % Note the default order of the joints is 3,2,1,4,5,6


            self.current_joint_states.Position = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
            self.robot.model.animate(self.current_joint_states.Position);
            test = isalmost(self.robot.model.getpos(),self.current_joint_states.Position, 0.001);
            if all(test) == 0
                self.robot.model.animate(self.current_joint_states.Position);
                self.callback_counter = 0;
            end
            self.callback_counter = self.callback_counter + 1;


            %display(test);
        end
        function wrenchCallback(self,~,msg)
             
            self.wrench = msg.Wrench;
            %display(self.wrench.Force)
        end

        function sendJointTrajectory(self,traj)
            test = isalmost(traj(1,:),self.current_joint_states.Position, 0.001);
            if all(test) == 0
                disp('Start of Traj does not match current robot position');
                return
            end
            msg = rosmessage(self.jointPublisher);
            for i = 1:size(traj,1)
                nextJointState = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                nextJointState.Positions = traj(i,:);
                %nextJointState.Velocities = v(i,:);
                %nextJointState.Accelerations = a(i,:);
                %nextJointState.Effort = ones(1,6);
                nextJointState.TimeFromStart = rosduration(i*0.1);
                msg.Goal.Trajectory.Points = [msg.Goal.Trajectory.Points; nextJointState];
            end

            msg.Goal.Trajectory.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            send(self.jointPublisher, msg);
        end

        function generateAndSendJointTrajectory(self,goalq)
        traj = jtraj(self.current_joint_states.Position, deg2rad(goalq), 100);
        self.sendJointTrajectory(traj);
        end

    end
end
