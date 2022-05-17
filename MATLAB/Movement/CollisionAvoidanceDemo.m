function CollisionAvoidanceDemo(robot, rc)
%COLLISIONAVOIDANCEDEMO Summary of this function goes here
%   Detailed explanation goes here


robot.model.animate([-2.2043   -2.0461   -1.0667   -1.5996    1.5708    0.1519]);


traj = rc.moveCartesian([1,0,0], 50);
rc.ExecuteTrajectory(traj);

end

