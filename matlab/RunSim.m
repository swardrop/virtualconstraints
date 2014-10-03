clear
close all
duration = 5;
dur_tol = 1e-8;
org = [0; 0];

% Set up simulation outputs
timeleft = duration;
t = [];
q = [];
qd = [];
qdd = [];
u = [];
err = [];
impact = [];

% Set up terrain (stored as set of heights with zero-order hold)
%ground = [-realmax,0];                    % Flat ground
%ground = [-realmax, 0; 1 -0.05];          % Step up 0.05m at 1m
% ground = [-realmax, 0;
%             1       -0.05
%             2       0
%             3       0.05
%             3.5     0.1];
ground = [-realmax,0];

% Define holonomic constraint (Bezier curve)
constrData = ConstrGui;
theta_p = constrData(1).theta_p;
alpha_p = constrData(1).alpha_p;

cd worker
addpath ../model/active
% Set initial conditions
q_0 = constrEndPts(theta_p, alpha_p);
theta_dot_sq_0 = thdsq_nom(constrData, 0);
qd_0 = constrData.d_Phi(:,1)*sqrt(theta_dot_sq_0);
last_t = 0;
while (timeleft > dur_tol)
    
    u_nom = nomTorque(constrData, theta_dot_sq_0);
    nom_torque = [constrData.th_base; u_nom];
    
    stoptime = sprintf('%.2f', timeleft);
    disp('Simulating footstep...');
    simOut = sim('RobotSim', ...
        'StartTime', '0.0', 'StopTime', stoptime);
    
    %%% Extract simulation data
    % Build time base
    new_t = simOut.find('tout')';
    t = [t, new_t + last_t]; %#ok<*AGROW>
    last_t = t(end);
    % Extract state data
    q = [q, simOut.find('q')'];
    qd = [qd, simOut.find('qd')'];
    qdd = [qdd, simOut.find('qdd')'];
    % Extract controller data
    u = [u, simOut.find('u')'];
    err = [err, simOut.find('err')'];
    % Construct array of impact times
    impact(end+1) = true; %#ok<*SAGROW>
    impact = [impact, false(1, length(new_t)-1)];
    
    % Deduct time of footstep from remaining time
    timeleft = timeleft - new_t(end);
    
    if simOut.find('fall')
        fprintf('Fallback detected. Ceasing simulation.\n\n');
        timeleft = 0;
    elseif timeleft > dur_tol
        fprintf('Time of impact: %.2f s\n', last_t);
        % Enact change of coordinates and velocities at impact
        [q_0, qd_0, error] = ...
            impactDynamics(q(:,end), qd(:,end));
        theta_dot_sq_0 = phasevar(qd_0)^2;
        % Set origin in (x,y) for new swing phase
        org = endSwingFoot(q(:,end),org);
        fprintf('Origin: (%.2f, %.2f)\n\n', org(1), org(2));
        if (error)
            break;
        end
    end
end

% Diagnostic plots
figure('Position', [10, 100, 600, 900])
subplot(3,1,1)
plot(t, q)
ylabel('$q$ (rad)', 'interpreter', 'latex')
grid on
subplot(3,1,2)
plot(t, qd)
ylabel('$\dot{q}$ (rad/s)', 'interpreter', 'latex')
grid on
subplot(3,1,3)
plot(t, qdd)
ylabel('$\ddot{q}$ (rad/s/s)', 'interpreter', 'latex')
grid on
suptitle('State evolution')

figure('Position', [620, 400, 600, 600])
subplot(2,1,1)
plot(t, u);
grid on
ylabel('$u$ (Nm)', 'interpreter', 'latex')
subplot(2,1,2)
plot(t, err);
ylabel('Error in q (rad)');
grid on
suptitle('Torque expended');

% Animation of motion
figure('Position', [300, 100, 1000, 600])
mov = visualise(q, impact, ground);
cd ..