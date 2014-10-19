function visualisePrint(q, isImpact, ground)
figure('Position', [300, 100, 1000, 600])
% Visualise compass-gait robot

[~, l] = dynParams();
X1 = zeros(2, size(q,2));
X2 = zeros(size(X1));
C = [0;0]; % Pivot point of stance leg - centre of frame.
frames(1:length(isImpact)) = struct('cdata', [], 'colormap', []);
fsize_x = 6*l;        % Frame window size (x)
fsize_y = 3*l;        % (y)

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
    'FaceColor', [0.35 0.3 0.3], ...
    'BaseValue', -realmax);
hold on;
% trace = plot(0,0);
pts = plot(0,0);
links = plot(0,0);

licol = 'r';
ricol = 'k';
rcol = [0.8 0.3 0.3];
lcol = [0.5 0.5 0.5];

% Generate a frame for each data point in time
for i = 1 : length(isImpact)
    % Check for an impact - if so, shift centre to endpoint of swing leg
    if isImpact(i) && i > 1
        C = X2(:, i-1);
    end
    q1 = q(1,i);
    q2 = q(2,i);
    % Express coordinates of joint between links
    X1(1, i) = C(1) + l * sin(q2);
    X1(2, i) = C(2) + l * cos(q2);
    x1 = X1(1,i);
    y1 = X1(2,i);
    % Express coordinates of endpoint of second link
    X2(1, i) = X1(1, i) + l * sin(q1-q2);
    X2(2, i) = X1(2, i) - l * cos(q1-q2);
    x2 = X2(1,i);
    y2 = X2(2,i);
%     delete(trace);
    %delete(pts);
    %delete(links);
    
    % Plot the current position of the arm, as well as a trace of its path
%     trace = plot(X1(1,1:i), X1(2,1:i), 'm-', ...
%                  X2(1,1:i), X2(2,1:i), 'b-');
    if isImpact(i) || i == length(isImpact)
        %plot([C(1) x1], [C(2) y1], '-', 'LineWidth', 2, 'Color', licol);
        %plot([x1 x2], [y1 y2], '-', 'LineWidth', 2, 'Color', ricol);
        tmp = licol;
        licol = ricol;
        ricol = tmp;
        tmp = lcol;
        lcol = rcol;
        rcol = tmp;
    elseif i>2 && X2(2,i) < X2(2,i-1) && X2(2,i-1) > X2(2,i-2) ...
            && X2(1,i) > X1(1,i)
        plot([x2 x1 C(1)], [y2 y1 C(2)], 'k.', 'MarkerFaceColor', 'k', ...
            'MarkerSize', 10);
        plot([C(1) x1], [C(2) y1], 'k-', 'LineWidth', 1,'Color', licol);
        plot([x1 x2], [y1 y2], 'k-', 'LineWidth', 1,'Color', ricol);
    end
    
%     axis([X1(1,i)-fsize_x/2, X1(1,i)+fsize_x/2, ...
%         X1(2,i)-3/4*fsize_y, X1(2,i)+1/4*fsize_y]);
    axis([-1 5 -0.5 1.5]);
    daspect([1 1 1]);
    %frames(i) = getframe();
end
end