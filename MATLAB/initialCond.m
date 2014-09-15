function [q, qd] = initialCond(motionName)
% Initial conditions for the compass-gait

q = [-pi/6; -2*pi/6];
qd = [2*pi, pi];

end