function visualiseStatic(q)

[~,l] = dynParams;

X1 = zeros(2,size(q,2));
X2 = zeros(size(X1));

q1 = q(1,:);
q2 = q(2,:);
% Express coordinates of joint between links
X1(1,:) = l * sin(q2);
X1(2,:) = l * cos(q2);
% Express coordinates of endpoint of second link
X2(1,:) = X1(1,:) + l * sin(q1-q2);
X2(2,:) = X1(2,:) - l * cos(q1-q2);

hold off
% Plot path of end of swing leg
plot(X2(1,:), X2(2,:), 'Color', [0.5 0.5 1], 'LineStyle', ':');
axis([-l l -0.1*l 1.1*l]);
axis equal
hold on

% Plot initial robot position
plot([0 X1(1,1) X2(1,1)], [0 X1(2,1) X2(2,1)], 'k-');

% Plot intermediate positions
len = size(q,2);
num = 9;
idx = round(len/(num+1)) : round(len/(num+1)) : len - floor(len/(num+1));
x = [zeros(length(idx), 1), X1(1,idx)', X2(1,idx)'];
y = [zeros(length(idx), 1), X1(2,idx)', X2(2,idx)'];
set(gca, 'ColorOrder', ...
    [0.8:0.2/(num-1):1; ...
    0.8:-0.2/(num-1):0.6; ...
    0.8:-0.2/(num-1):0.6]');
plot(x',y');

% Plot final robot position
plot([0 X1(1,end) X2(1,end)], [0 X1(2,end) X2(2,end)], 'r-');

end