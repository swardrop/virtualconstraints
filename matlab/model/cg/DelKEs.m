function DelKE = DelKEs(nk)
% Produce an array of nk DeltaKEs based upon the robot (More massive robots
% will need larger max and min DelKEs for same effect).

% Determine the range in DelKE by choosing a nominal final configuration
% (flat ground) and adding a set velocity.
nom_qf = pi/12*[2,1];
max_Delqd = pi*[0.5;0.25];
M = dynMatrices(nom_qf);

max_DelKE = max_Delqd' * M * max_Delqd;
min_DelKE = -max_DelKE;

DelKE = linspace(min_DelKE, max_DelKE, nk);

end