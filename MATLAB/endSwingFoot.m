function [p2, dp2] = endSwingFoot(q, org)
% p2 - End of the swing foot in terms of the coordinates q and the origin.
% dp2 - Derivative of p2 in terms of q.
[~,l] = dynParams;
% Compass-gait
p2 = zeros(2, size(q,2));
p2(1,:) = org(1) + l*(sin(q(2,:)) + sin(q(1,:)-q(2,:)));
p2(2,:) = org(2) + l*(cos(q(2,:)) - cos(q(1,:)-q(2,:)));

dp2 = [ l*cos(q(1,:)-q(2,:)), l*(cos(q(2,:)) - cos(q(1,:)))
        l*sin(q(1,:)-q(2,:)), -l*(sin(q(2,:)) + sin(q(1,:)-q(2,:))) ];
end