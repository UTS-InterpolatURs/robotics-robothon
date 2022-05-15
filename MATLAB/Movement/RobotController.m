classdef RobotController< handle
    %TRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot
        useRos
        realBot
    end

    methods
        function self = RobotController(robot, realBot)
            %TRAJECTORYGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            self.robot = robot;
            if ~exist('realBot','var') || isempty(realBot)
                self.useRos=false;
            else
                self.useRos = true;
                self.realBot = realBot;
            end
        end
        function qMatrix = GenerateJointTrajectory(self,goalPose,steps)
            goalPoseAdjusted = self.robot.GetGoalPose(goalPose);
            robotQ = self.robot.model.getpos();
            goalQ = self.robot.model.ikcon(goalPoseAdjusted, robotQ);
            %             baseToOriginTR = inv(self.robot.model.base);
            %             robotToObject = baseToOriginTR * goalPoseAdjusted;
            %             viaQ = robotq;
            %
            %             viaQ(1) = atan2(robotToObject(2,4),robotToObject(1,4)) - pi;
            %
            %             qMatrix = jtraj(robotq,viaQ,steps);

            %             targetQ = self.robot.model.ikcon(goalPoseAdjusted, self.robot.model.getpos());
            %             qMatrix = [qMatrix; jtraj(qMatrix(end,:), targetQ, steps)];
            qMatrix = jtraj(robotQ, goalQ, steps);
        end

        function qMatrix = GenerateLinearTrajectory(self,goalPose,steps, velocityMask)
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
            end
            goalPoseAdjusted = self.robot.GetGoalPose(goalPose);

            model = self.robot.model;
            currentPose = model.fkine(model.getpos());
            X = zeros(3,steps);
            qMatrix = zeros(steps,6);
            t = 10;             % Total time (s)
            deltaT = steps*t;      % Control frequency
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            if exist('velocityMask','var')
                W = diag([velocityMask]);
            end
            epsilon = 0.015;      % Threshold value for manipulability/Damped Least Squares
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles


            x1 = currentPose(1:3,4);
            x2 = goalPoseAdjusted(1:3,4);

            a1 = tr2rpy(currentPose);
            a2 = tr2rpy(goalPoseAdjusted);

            s = lspb(0,1,steps);

            for i = 1:steps
                X(1,i) = x1(1)*(1-s(i)) + s(i)*x2(1);
                X(2,i) = x1(2)*(1-s(i)) + s(i)*x2(2);
                X(3,i) = x1(3)*(1-s(i)) + s(i)*x2(3);
                theta(1,i) = a1(1)*(1-s(i)) + s(i)*a2(1);                 % Roll angle
                theta(2,i) = a1(2)*(1-s(i)) + s(i)*a2(2);            % Pitch angle
                theta(3,i) = a1(3)*(1-s(i)) + s(i)*a2(3);                % Yaw angle
            end

            qMatrix(1,:) = model.getpos();

            for i = 1:steps-1
                T = model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = X(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                    disp("Approaching singularity - activating DLS");
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities

            end

        end

        function qMatrix = moveCartesian(self, x ,steps)
            currentPose = self.robot.GetEndEffPose();
            goalPose = transl(x) * currentPose;
            velocityMask = [1,1,1,0,0,0];

            qMatrix = self.GenerateLinearTrajectory(goalPose,steps, velocityMask);
        end

        function ExecuteTrajectory(self, qMatrix, object)
            restartFlag = false;
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
                self.realBot.sendJointTrajectory(qMatrix);
            end


            for i=1:size(qMatrix,1)
                if(self.robot.eStopStatus == 1)
                    if(self.useRos)
                        self.realBot.pause();
                        restartFlag = true;
                    end

                end
                while(self.robot.eStopStatus == 1)
                    pause(0.1);
                end

                if restartFlag == true
                    self.realBot.play();
                end

                self.robot.model.animate(qMatrix(i,:))
                if exist('object','var')
                    object.MoveModel(self.robot.GetEndEffPose() * trotx(pi));
                end
                drawnow();
                pause(0.1);
            end

        end



        function OpenGripper(self)
            if(self.useRos)
                self.realBot.gripper.openGripper;
            end
            self.robot.SetGripperState("gripperState", 0)
        end

        function CloseGripper(self, effort)
            if(self.useRos)
                self.realBot.gripper.closeGripper(effort);
            end
            self.robot.SetGripperState("gripperState", 1)
        end
    end
end

