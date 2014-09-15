close all
duration = 5;
dur_tol = 1e-8;
org = [0, 0];

% Set up simulation outputs
timeleft = duration;
t = [];
q = [];
qd = [];
u = [];
err = [];
impact = [];

% Set up terrain (stored as set of heights with zero-order hold)
%ground = [-realmax,0];                    % Flat ground
%ground = [-realmax, 0; 1 -0.05];          % Step up 0.05m at 1m
ground = [-realmax, 0;
            1 -0.05
            2 0
            3 0.05
            3.5 0.1];

% Define holonomic constraint (Bezier curve)
constrData = ConstrGui;
theta_p = constrData(1).theta_p;
alpha_p = constrData(1).alpha_p;

% Set initial conditions
[q_0, qd_0] = initialCond('slowWalk');
theta_dot_sq_0 = phasevar(qd_0)^2;
last_t = 0;
while (timeleft > dur_tol)
    
    [u, theta] = nomTorque(points, theta_dot_sq_0);
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
        [q_, qd_, error] = ...
            impactDynamics([t1(end); t2(end)], [t1d(end); t2d(end)]);
        t1_0 = q_(1); t2_0 = q_(2); t1d_0= qd_(1); t2d_0 = qd_(2);
        % Set origin in (x,y) for new swing phase
        org(1) = org(1) + l1*cos(t1(end)) + l2*cos(t1(end)+t2(end));
        org(2) = org(2) + l1*sin(t1(end)) + l2*sin(t1(end)+t2(end));
        fprintf('Origin: (%.2f, %.2f)\n\n', org(1), org(2));
        if (error)
            break;
        end
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