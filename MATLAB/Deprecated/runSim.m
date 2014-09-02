close all
global t_bez
duration = 5;
% Define dynamical constants
I1 = 1;         % Moment of inertia about end of stance leg
l1 = 0.5;
m1 = 1.2;

I2 = 0.6;       % Moment of inertia about centre of swing leg
l2 = 0.5;
m2 = 1.2;

g = 9.81;

points = BezGuiAttempt';
% % Set Bezier points (four for cubic Bezier curve)
% % P0 and P3 must obey th_2 = - 2*th_1 (impact condition)
% P0 = [2*pi/3; -4*pi/3];
% P3 = [pi/3; -2*pi/3];
% % P2 and P3 define the curvature of the line
% P1 = [0.4*pi; -1*pi];
% P2 = [0.4*pi; -0.5*pi];
% points = [P0 P1 P2 P3];
% [th1_lookup, th2_lookup] = cubicBezier(P0, P1, P2, P3);
t_bez = solveBezFor_t(points);

% Initial conditions
[t1d_0, t2d_0, t1_0, t2_0] = initialCond('slowWalk');

stoptime = sprintf('%.1f', duration);
disp('Running simulation, please wait...');
simOut = sim('compassGaitSim_impacts1', ...
    'StartTime', '0.0', 'StopTime', stoptime);

% Extract timeseries data
t1 = simOut.find('t1');
t2 = simOut.find('t2');
impacts = simOut.find('impact');
t1d = simOut.find('t1d');
t2d = simOut.find('t2d');

% Diagnostic plots
figure
hold on
plot(100*impacts-50,'g')
plot(t1d,'b')
plot(t2d,'m')
plot(t1,'k')
plot(t2,'r')
legend('Impacts', 't1d', 't2d', 't1', 't2');

% Animation of motion
figure
mov = visualise(t1.data, t2.data, impacts.data);