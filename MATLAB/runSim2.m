close all
duration = 5;
dur_tol = 1e-8;
[Is, ls, ms, g] = getDynParams();
I1 = Is(1);
I2 = Is(2);
l1 = ls(1);
l2 = ls(2);
m1 = ms(1);
m2 = ms(2);
org = [0, 0];

% Set up simulation outputs
timeleft = duration;
t = [];
t1 = [];
t2 = [];
t1d = [];
t2d = [];
T2 = [];
t2err = [];
t2ed = [];
impact = [];

% Set up terrain (stored as set of heights with zero-order hold)
%ground = [-realmax,0];                    % Flat ground
%ground = [-realmax, 0; 1 -0.05];            % Step up 0.05m at 1m
ground = [-realmax, 0;
            1 -0.05
            2 0
            3 0.05
            3.5 0.1];
ground_x = ground(:,1);
ground_y = ground(:,2);

% Define holonomic constraint (Bezier curve)
constrData = ConstrGui;
points = constrData(1).points;

% Set initial conditions
[t1d_0, t2d_0, t1_0, t2_0] = initialCond('slowWalk');
theta_dot_sq_0 = t1_0^2;
last_t = 0;
while (timeleft > dur_tol)
    
    [u, theta] = nomTorqueCG(points, theta_dot_sq_0);
    nom_torque_table = [theta', u'];
    
    stoptime = sprintf('%.2f', timeleft);
    disp('Simulating footstep...');
    simOut = sim('compassGaitSim', ...
        'StartTime', '0.0', 'StopTime', stoptime);
    
    %%% Extract simulation data
    % Build time base
    new_t = simOut.find('tout');
    t = [t; new_t + last_t];
    last_t = t(end);
    % Extract state data
    t1 = [t1; simOut.find('t1')];
    t2 = [t2; simOut.find('t2')];
    t1d = [t1d; simOut.find('t1d')];
    t2d = [t2d; simOut.find('t2d')];
    % Extract controller data
    T2 = [T2; simOut.find('T2')];
    t2err = [t2err; simOut.find('t2err')];
    t2ed = [t2ed; simOut.find('t2ed')];
    % Construct array of impact times
    impact(end+1) = true;
    impact = [impact; false(length(new_t)-1, 1)];
    
    % Deduct time of footstep from remaining time
    timeleft = timeleft - new_t(end);
    
    theta_dot_sq_0 = t1d(end)^2;
    
    if simOut.find('fall')
        fprintf('Fallback detected. Ceasing simulation.\n\n');
        timeleft = 0;
    elseif timeleft > dur_tol
        fprintf('Time of impact: %.2f s\n', last_t);
        % Enact change of coordinates and velocities at impact
        [t1_0, t2_0, t1d_0, t2d_0] = ...
            impactDynamics(t1(end), t2(end), t1d(end), t2d(end));
        % Set origin in (x,y) for new swing phase
        org(1) = org(1) + l1*cos(t1(end)) + l2*cos(t1(end)+t2(end));
        org(2) = org(2) + l1*sin(t1(end)) + l2*sin(t1(end)+t2(end));
        fprintf('Origin: (%.2f, %.2f)\n\n', org(1), org(2));
        
    end
    
end

% Diagnostic plots
figure
plot(t, [impact, t1d, t2d, t1, t2])
legend('Impacts', 't1d', 't2d', 't1', 't2');
figure
subplot(2,1,1)
plot(t, [t2err impact*0.3]);
legend('t2err', 'impacts');
grid on
subplot(2,1,2)
plot(t, T2);
grid on
legend('u');

% Animation of motion
figure
mov = visualise(t1, t2, impact, ground);