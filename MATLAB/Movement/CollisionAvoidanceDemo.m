function CollisionAvoidanceDemo(robot, rc)
%COLLISIONAVOIDANCEDEMO Summary of this function goes here
%   Detailed explanation goes here


robot.model.animate([-1.9955   -2.2444   -1.8776   -0.5904    1.5708    0.3607]);


traj = rc.moveCartesian([1,0,0], 50);
rc.ExecuteTrajectory(traj);

end

