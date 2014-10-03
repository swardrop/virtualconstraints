function s = scuff(q)
% Returns true for the compass-gait if the swing leg is behind the
% stance leg, and for any other model it should return false.

s = q(1) <= 0 || q(2) <= 0;

end