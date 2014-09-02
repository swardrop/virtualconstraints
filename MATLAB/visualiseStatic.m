function visualiseStatic(theta1, theta2)

l1 = 1;
l2 = 1;
X1 = zeros(length(theta1),2);
X2 = zeros(size(X1));

X1(:,1) = l1 * cos(theta1);
X1(:,2) = l1 * sin(theta1);

X2(:,1) = X1(:,1) + l2 * cos(theta1 + theta2);
X2(:,2) = X1(:,2) + l2 * sin(theta1 + theta2);

hold off
plot(X2(:,1), X2(:,2), 'Color', [0.5 0.5 1], 'LineStyle', ':');
axis([-1 1 -0.1 1.1]);
axis equal
hold on

% Plot initial robot position
plot([0 X1(1,1) X2(1,1)], [0 X1(1,2) X2(1,2)], 'k-');

% Plot intermediate positions
len = length(theta1);
num = 10;
idx = round(len/(num+1)) : round(len/(num+1)) : len - floor(len/(num+1));
x = [zeros(length(idx), 1), X1(idx,1), X2(idx,1)]';
y = [zeros(length(idx), 1), X1(idx,2), X2(idx,2)]';
set(gca, 'ColorOrder', ...
    [0.8:0.2/(num-1):1; ...
    0.8:-0.2/(num-1):0.6; ...
    0.8:-0.2/(num-1):0.6]');
plot(x,y);

% Plot final robot position
plot([0 X1(end,1) X2(end,1)], [0 X1(end,2) X2(end,2)], 'r-');

end