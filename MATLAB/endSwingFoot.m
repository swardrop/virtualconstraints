function [x, y] = endSwingFoot(q, org)
% End of the swing foot in terms of the coordinates q and the origin.

% Compass-gait
x = org(1) + l*(sin(q(2)) + sin(q(1)-q(2)));
y = org(2) + l*(cos(q(2)) - cos(q(1)-q(2)));
end