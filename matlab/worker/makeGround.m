function groundHeight = makeGround(q_p, q_m)
% Define a groud height map on the basis of the initial and final
% configurations. Note that the stance foot is considered to be (0,0).

% Current method: split full step (from initial to final) into three
% partitions. The first is at the height of q_p, second is at
% zero and third is at q_m. The height changes half way between zero and
% the endpoints such that the middle height is double as long as the
% others.

end1 = endSwingFoot(q_p, [0 0]);
end2 = endSwingFoot(q_m, [0 0]);

groundHeight(1,:) = end1';
groundHeight(2,:) = [end1(1)/2, 0];
groundHeight(3,:) = [end2(1)/2, end2(2)];

end