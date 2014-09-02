function [Gamma, Psi, th_base, th_c] = PartialSolZeroDyn(constrPts)

[Is, ls, ms, g] = getDynParams();
I1 = Is(1);
I2 = Is(2);
l1 = ls(1);
l2 = ls(2);
m1 = ms(1);
m2 = ms(2);

theta_f = constrPts(end,1);
theta_0 = constrPts(1,1);
step_size = 0.001;
step_size = step_size * sign(theta_f-theta_0);

integ1 = 0;
integ2 = 0;
Gamma = [];
Psi = [];
gamma_prev = 0;
th_base = theta_0:step_size:theta_f;
fx = zeros(size(th_base));
gx = zeros(size(th_base));
j = 1;
th_c = inf;

for theta = theta_0:step_size:theta_f
    % Produce angle phi from theta
    phi = 0;
    n = size(constrPts,1) - 1;
    for i = 0 : n
        phi = phi + nchoosek(n,i)*(theta_f-theta)^(n-i)*...
            (theta-theta_0)^i*constrPts(i+1,2);
    end
    phi = phi/(theta_f - theta_0)^n;

    % Calculate derivative of phi
    d_phi = n*((theta-theta_0)^(n-1)*constrPts(n+1,2) ...
        - (theta_f-theta)^(n-1)*constrPts(1,2));
    for i = 1 : n-1
        d_phi = d_phi + nchoosek(n,i)*constrPts(i+1,2) * (...
            i*(theta_f-theta)^(n-i)*(theta-theta_0)^(i-1) - ...
            (n-i)*(theta_f-theta)^(n-i-1)*(theta-theta_0)^i);
    end
    d_phi = d_phi/(theta_f - theta_0)^n;
    % Calculate second derivative of phi
    dd_phi = n*(n-1)*((theta_f-theta)^(n-2)*constrPts(1,2) + ...
        (theta-theta_0)^(n-2)*constrPts(n+1,2) + (n-2)*(...
        (theta-theta_0)*(theta_f-theta)^(n-3)*constrPts(2,2) + ...
        (theta_f-theta)*(theta-theta_0)^(n-3)*constrPts(n,2)) - 2*(...
        (theta_f-theta)^(n-2)*constrPts(2,2) + ...
        (theta-theta_0)^(n-2)*constrPts(n,2)));
    for i = 2 : n-2
        dd_phi = dd_phi + nchoosek(n,i)*constrPts(i+1,2) * (...
            i*(i-1)*(theta_f-theta)^(n-i)*(theta-theta_0)^(i-2) - ...
            2*i*(n-i)*(theta_f-theta)^(n-i-1)*(theta-theta_0)^(i-1) + ...
            (n-i-1)*(n-i)*(theta_f-theta)^(n-i-2)*(theta-theta_0)^i );
    end
    dd_phi = dd_phi/(theta_f - theta_0)^n;

    %%% Prepare matrices
    % Variables
    d_Phi = [1; d_phi];
    dd_Phi = [0; dd_phi];
    % Compute mass/inertia matrix
    M = [I1+I2+m2*(I1^2+0.25*l2+l1*l2*cos(phi)), ...
        I2+0.5*m2*(0.5*l2^2+l1*l2*cos(phi)); ...
        I2+0.5*m2*(0.5*l2^2+l1*l2*cos(phi)), ...
        I2+0.25*m2*l2^2];
    % Compute coriolis/centrifugal matrix
    C = sin(phi)*[-m2*l1*l2*d_phi, -0.5*m2*l1*l2; 0.5*m2*l1*l2, 0];
    % Compute gradient of gravitational potential energy
    G = [0.5*l1*m1*g*cos(theta) + m2*g*(l1*cos(theta)+0.5*l2*cos(theta+phi))
        0.5*m2*g*l2*cos(theta+phi)];
    % Use trivial B_perp
    B_perp = [1 0];

    alpha = B_perp*M*d_Phi;
    beta = B_perp*(M*dd_Phi + C*d_Phi);
    gamma = B_perp*G;
    
    % Find th_c
    if i == 1
        gamma_prev = gamma;
    end
    if (gamma < 0 && gamma_prev > 0) || (gamma > 0 && gamma_prev < 0)
        % Use linear interpolation between two points
        th_c = gamma_prev/(gamma_prev-gamma)*step_size + theta-step_size;
    end
    gamma_prev = gamma;
    
    fx(j) = 2*beta/alpha;
    gx(j) = -2*gamma/alpha;
    j = j + 1;
end

int_fx = cumtrapz(fx)*step_size;
int_gxblah = cumtrapz(gx.*exp(int_fx))*step_size;
Gamma = exp(-int_fx);
Psi = exp(-int_fx).*int_gxblah;

end