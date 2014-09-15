function frames = visualise(theta1, theta2, isImpact, ground)

[~, ls, ~, ~] = getDynParams();
l1 = ls(1);
l2 = ls(2);
X1 = zeros(2, length(theta1));
X2 = zeros(size(X1));
C = [0;0]; % Pivot point of stance leg - centre of frame.
frames(1:length(theta1)) = struct('cdata', [], 'colormap', []);
fsize_x = 6*l1;        % Frame window size (x)
fsize_y = 3*l1;        % (y)

% Draw the ground
j = 2;
steps = [ground(j-1,1), ground(j-1,2)];
while j <= size(ground,1)
    steps(end+1,:) = [ground(j,1), ground(j-1,2)];
    steps(end+1,:) = ground(j,:);
    j = j + 1;
end
steps(end+1,:) = [realmax, ground(j-1,2)];
area(steps(:,1), steps(:,2),  ...
    'FaceColor', [0.6 0.15 0.15], ...
    'BaseValue', -realmax);
hold on;
% trace = plot(0,0);
pts = plot(0,0);
links = plot(0,0);

% Generate a frame for each data point in time
for i = 1 : length(theta1)
    % Check for an impact - if so, shift centre to endpoint of swing leg
    if isImpact(i) && i > 1
        C = X2(:, i-1); % Maybe fix?
    end
    t1 = theta1(i);
    t2 = theta2(i);
    % Express coordinates of joint between links
    x1 = C(1) + l1 * cos(t1);
    y1 = C(2) + l1 * sin(t1);
    X1(1, i) = x1;
    X1(2, i) = y1;
    % Express coordinates of endpoint of second link
    x2 = x1 + l2 * cos(t1 + t2);
    y2 = y1 + l2 * sin(t1 + t2);
    X2(1, i) = x2;
    X2(2, i) = y2;
    
%     delete(trace);
    delete(pts);
    delete(links);
    
    % Plot the current position of the arm, as well as a trace of its path
%     trace = plot(X1(1,1:i), X1(2,1:i), 'm-', ...
%                  X2(1,1:i), X2(2,1:i), 'b-');
    pts = plot([x2 x1 C(1)], [y2 y1 C(2)], 'ko', 'MarkerFaceColor', 'k');
    links = plot([C(1) x1 x2], [C(2) y1 y2], 'k-', 'LineWidth', 3);
    
    axis([X1(1,i)-fsize_x/2, X1(1,i)+fsize_x/2, ...
        X1(2,i)-3/4*fsize_y, X1(2,i)+1/4*fsize_y]);
    daspect([1 1 1]);
    frames(i) = getframe();
end
end