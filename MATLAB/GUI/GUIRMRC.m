function newq  = GUIRMRC(xdot, robot)
%GUIRMRC Summary of this function goes here
%   Detailed explanation goes here
    q = robot.model.getpos();
    J = robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.05
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        newq = q;
    end
end

