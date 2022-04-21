% Custom class for move robots and bricks
classdef MoveRobot < handle
    properties (Constant)

    end
    methods (Static)
        % Move one robot
        function MoveOneRobot(robot,qCurr,qGoal,steps)
            qMatrix = jtraj(qCurr,qGoal,steps);
            for i=1:steps
                robot.model.animate(qMatrix(i,:));
                drawnow();
            end
        end
        
        
    end
end