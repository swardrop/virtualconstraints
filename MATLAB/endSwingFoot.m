function p2 = endSwingFoot(q, org)
% End of the swing foot in terms of the coordinates q and the origin.
[~,l] = dynParams;
% Compass-gait
p2 = zeros(2, size(q,2));
p2(1,:) = org(1) + l*(sin(q(2,:)) + sin(q(1,:)-q(2,:)));
p2(2,:) = org(2) + l*(cos(q(2,:)) - cos(q(1,:)-q(2,:)));
end