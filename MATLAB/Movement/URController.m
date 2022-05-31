classdef URController< handle
    %TRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot
        useRos
        realBot
        collisionComputer
        checkCollisionFlag
        controlFrequency
        %         desiredJointStateSubscriber
        vsSubscriber
        fps
        vsCounter
        vsFlagSubscriber
        vsFlag
    end

    methods
        function self = URController(robot, collisionComputer, realBot)
            %TRAJECTORYGENERATOR Construct an instance of this class
            % Initialises robot controller with collision computer,
            % simulation robot and real robot class (if present)
            self.robot = robot;
            self.collisionComputer = collisionComputer;
            self.checkCollisionFlag = false;
            self.controlFrequency = 0.05;
            self.fps = 3;
            self.vsCounter = 0;
            self.vsFlag = true;

            if ~exist('realBot','var') || isempty(realBot)
                self.useRos=false;
            else
                self.useRos = true;
                self.realBot = realBot;
                %                 self.desiredJointStateSubscriber = rossubscriber('/desired_joint_state', @self.desiredJointStatesCallback, 'sensor_msgs/JointState',"DataFormat","struct");
                self.vsSubscriber =  rossubscriber('/ColourDetectionChatter',@self.subscriberCallBackVS, 'std_msgs/Float32MultiArray');
                self.vsFlagSubscriber =  rossubscriber('/localisation_flag',@self.subscriberCallBackVSFlag, 'std_msgs/Bool');

            end
        end
        %Generates a joint trajectory using jtraj quintic polynomial
        % IF mask is passed, ikine is used, otherwise ikcon is used
        function qMatrix = GenerateJointTrajectory(self,goalPose,duration,mask)
            timescale = 0:self.controlFrequency:duration;
            robotQ = self.robot.model.getpos();
            if exist("jointMask", "var")
                goalQ = self.robot.model.ikine(goalPose, robotQ, mask);
            else
                goalQ = self.robot.model.ikcon(goalPose, robotQ);

            end

            qMatrix = jtraj(robotQ, goalQ, timescale);
        end
        
        %Generates a linear trajectory using RMRC (trapazoidal velocity
        %profile). Optional weighting matrix (velocity mask) can be passed
        %to limit motion 
        function qMatrix = GenerateLinearTrajectory(self,goalPose,duration, velocityMask)
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
            end
            model = self.robot.model;
            currentPose = model.fkine(model.getpos());
            deltaT = self.controlFrequency;       %Control frequency
            steps = duration/deltaT;             %Steps needed to achieve a given duration 

            qMatrix = zeros(steps,6);

            X = zeros(3,steps);

            W = diag([1 1 1 0.15 0.15 0.15]);    % Weighting matrix for the velocity vector
            if exist('velocityMask','var')
                W = diag(velocityMask); %OPTIONAL mask passed in by user
            end
            epsilon = 0.01;      % Threshold value for manipulability/Damped Least Squares
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles

            %establish starting point and angle
            x1 = currentPose(1:3,4);
            x2 = goalPose(1:3,4);

            a1 = tr2rpy(currentPose);
            a2 = tr2rpy(goalPose);
            
            %trajectory scalar
            s = lspb(0,1,steps);

            for i = 1:steps
                %apply trajectory scalar to interpolate from start to
                %finish
                X(1,i) = x1(1)*(1-s(i)) + s(i)*x2(1);
                X(2,i) = x1(2)*(1-s(i)) + s(i)*x2(2);
                X(3,i) = x1(3)*(1-s(i)) + s(i)*x2(3);
                theta(1,i) = a1(1)*(1-s(i)) + s(i)*a2(1);                 % Roll angle
                theta(2,i) = a1(2)*(1-s(i)) + s(i)*a2(2);            % Pitch angle
                theta(3,i) = a1(3)*(1-s(i)) + s(i)*a2(3);                % Yaw angle
            end
            %establish first q as current position to avoid initial jerk
            qMatrix(1,:) = model.getpos();

            for i = 1:steps-1
                T = model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = X(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
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

        %calls RMRC function using a mask to ensure motion in XYZ only
        function moveCartesian(self, X ,duration)
            currentPose = self.robot.EndEffToGlobalPose(self.robot.model.fkine(self.robot.model.getpos()));
            goalPose = transl(X) * currentPose;
            %             trplot(goalPose);
            velocityMask = [1,1,1,0,0,0];

            qMatrix = self.GenerateLinearTrajectory(goalPose,duration, velocityMask);
            %             trplot(self.robot.model.fkine(qMatrix(end,:)))
            self.ExecuteTrajectory(qMatrix);
        end

        %uses RMRC to control end effector in end effector frame
        function moveEndEffector(self, X ,duration)
            currentPose = self.robot.EndEffToGlobalPose(self.robot.model.fkine(self.robot.model.getpos()));
            goalPose = currentPose * transl(X);
            velocityMask = [1,1,1,0,0,0];

            qMatrix = self.GenerateLinearTrajectory(goalPose,duration, velocityMask);
            self.ExecuteTrajectory(qMatrix);
        end
        
        %function that takes trajectory and sends it to the real robot
        %and/or animates the simulation robot

        function success = ExecuteTrajectory(self, qMatrix, object)
            self.robot.robotBusy = true;
            trajPatchIndex = 0; %counter for collision avoidance
            avoidanceFlag = 0; %flag for collision avoidance
            restartFlag = false; %flag for e-stop resume
            if(self.useRos) %check to see if real robot is currently executing a trajecotry
                if(self.realBot.robotBusy == true)
                    disp("robot is busy");
                    return;
                end
                %send trajectory to real robot
                self.realBot.robotBusy = true;
                self.realBot.sendJointTrajectory(qMatrix, self.controlFrequency);
                return;
            end

            %begin iteration for sim robot
            for i=1:size(qMatrix,1)
                if(self.robot.eStopStatus == 1) %check if e-stop has been triggered
                    if(self.useRos)
                        self.realBot.pause();
                        restartFlag = true;
                    end

                end
                %#######LOOP TO BLOCK WHILE E_STOP IS PRESSED#############
                %                 tic
                %                 while(toc < 30)
                %                     if(self.robot.eStopStatus == 0)
                %                         break;
                %                     end
                %                     pause(0.1);
                %                 end
                %                 if (toc >= 30)
                %                     disp("estop timeout triggered, please restart program");
                %                     success = false;
                %                     return;
                %                 end


                if restartFlag == true %resume real robot trajectory if program has been resumed
                    self.realBot.play();
                end
                
                %The following loop executes if user has selected the
                %option to check collisions 
                if(self.checkCollisionFlag == true)
                    result = false; 
                    try result = self.collisionComputer.checkCollision(qMatrix(i,:));
                    end
                    
                    %if a collsion is detected....
                    if(result == true)
                        disp("COLLISION IMMINENT - COLLISION AVOIDANCE ACTIVATED!")
                        traj = self.robot.model.getpos;
                        %first move robot backwards along trajectory 5
                        %steps...
                        for j = 1:5
                            traj = [traj; qMatrix(i-j,:)];
                        end
                        self.ExecuteTrajectory(traj);
                        %then move robot up 30cm (to clear obstacle like a
                        %hand)
                        traj = self.moveCartesian([0,0,0.3], 20);
                        self.ExecuteTrajectory(traj);
                        
                        %check all remaining points in trajectory for
                        %collision and translate all collision points up by
                        %30cm
                        checkMatrix = qMatrix(i:end,:);
                        resultMatrix = self.collisionComputer.checkCollision(checkMatrix);

                        for k = 1:(size(resultMatrix, 1))
                            index = (i-1) + k;
                            if(resultMatrix(k) == true)
                                newGoal = self.robot.model.fkine(qMatrix(index,:)) * transl(0,0,-0.3);
                                qMatrix(index,:) = self.robot.model.ikcon(newGoal,qMatrix(index, :));
                                avoidanceFlag = 1;
                                trajPatchIndex = index;
                            end
                        end

                    end

                end
                %after ammended trajectory is executed, patch the old and
                %new trajectories together to achieve a smooth transisition
                if(avoidanceFlag == true && i == trajPatchIndex + 1)
                    traj = jtraj(qMatrix(i-1,:),qMatrix(i,:), 20);
                    self.ExecuteTrajectory(traj);
                end
                
                %Animate robot model 
                %if object is passes, move object with end effector
                self.robot.model.animate(qMatrix(i,:))
                if exist('object','var')
                    object.MoveModel(self.robot.GetEndEffPose() * trotx(pi));
                end
                %pause for controlFrequency to have real robot and sim
                %robot move simutanioulsy 
                drawnow();
                pause(self.controlFrequency);


            end

            success = true;

            self.robot.robotBusy = false;
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

        function MoveToNeutral(self)
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
            end
            currentQ = self.robot.model.getpos();

            if(self.robot.model.tool == self.robot.realSenseTf)
                traj = jtraj(currentQ, [-1.5710, -1.3736, -2.0368, -0.9529, 1.5709, 0.7852], 100);
            else
                traj = jtraj(currentQ, self.robot.neutralQ, 100);
            end
            self.ExecuteTrajectory(traj);
            %             traj = self.GenerateJointTrajectory(self.robot.neutralPose, 100);
            %             self.ExecuteTrajectory(traj);


        end

        function RotateSingleJoint(self, joint, angle, duration)
            currentQ = self.robot.model.getpos();
            newQ = currentQ;
            newQ(joint) = newQ(joint) + angle;

            timescale = 0:self.controlFrequency:duration;
            traj = jtraj(currentQ, newQ, timescale);
            self.ExecuteTrajectory(traj);
        end

        function SetToolCamera(self)
            if(self.useRos)
                if(self.realBot.robotBusy == true)
                    disp("robot is busy");
                    return;
                end
            end
            if(self.robot.model.tool == self.robot.realSenseTf)
                disp("Tool is already set to camera");
                return
            end
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
            end
            currentQ = self.robot.model.getpos();
            currentPose = self.robot.model.fkine(currentQ);
            newPose = currentPose * trotz(pi/2);

            self.robot.model.tool = self.robot.realSenseTf;

            newQ = self.robot.model.ikcon(newPose, currentQ);
            time = 0:self.controlFrequency:2;

            traj = jtraj(currentQ, newQ, time);
            self.ExecuteTrajectory(traj);


        end

        function SetToolGripper(self)
            if(self.useRos)
                if(self.realBot.robotBusy == true)
                    disp("robot is busy");
                    return;
                end
            end
            if(self.robot.model.tool == self.robot.gripperTf)
                disp("Tool is already set to gripper");
                return
            end
            if(self.useRos)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
            end
            currentQ = self.robot.model.getpos();
            currentPose = self.robot.model.fkine(currentQ);
            newPose = currentPose * trotz(-pi/2);
            self.robot.model.tool = self.robot.gripperTf;

            newQ = self.robot.model.ikcon(newPose, currentQ);
            time = 0:self.controlFrequency:2;
            traj = jtraj(currentQ, newQ, time);
            self.ExecuteTrajectory(traj);

        end

        function desiredJointStatesCallback(self, ~, msg)
            if(self.robot.robotBusy || self.robot.acceptCommand == false)
                disp("robot is busy");
                return;
            end
            traj = jtraj(self.realBot.current_joint_states.Position, msg.Position, 25);
            self.ExecuteTrajectory(traj);
        end

        function waitForTrajToFinish(self, duration)
            timeOut = duration + 2;
            startTime = toc;
            while(1)
                if(self.realBot.robotBusy == false)
                    break;
                end
                pause(0.1);
                if(toc-startTime >= timeOut)
                    disp("Trajectory Timed Out");
                    break;
                end
            end

        end

        function subscriberCallBackVS(self,~,msg)
            if(self.vsFlag == false)
                self.robot.model.animate(self.realBot.current_joint_states.Position);
                current_q = self.robot.model.getpos();
                %             if self.hist_q ~= self.current_q
                %             pause(3)
                %             self.current_q = self.jointStatesSubscriber.LatestMessage.position;
                %             self.robot.model.animate(self.current_q);

                vc = msg.Data;
                %             print(vc)
                vcTemp = vc(1:6,1);
                try
                    J = self.robot.model.jacobn(current_q);
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
                new_q = current_q + (1/self.fps)*qp';
                newQ = zeros(1,6);
                for i = 1:6
                    newQ(i) = new_q(i);
                end

                timescale = 0:self.controlFrequency:0.6;
                traj = jtraj(current_q, newQ, timescale);
                self.ExecuteTrajectory(traj);

                %             end
            end
        end

        function subscriberCallBackVSFlag(self,~,msg)
            self.vsFlag = msg.Data;
            disp("vs flag raised")
        end
    end
end

