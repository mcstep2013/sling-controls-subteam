clear;

%% INITIAL CONDITIONS

rho = 1;      % linear density of cable (mass/length)
L0 = 500;     % total length of cable

m_c = 50000;  % mass of countermass
m_p = 1000;   % mass of payload

R_c = 1;      % radius of countermass
R_p = 1;      % radius of payload
R_t = 0.01;   % radius of tether

M1 = 10;  % moment about x-axis
M2 = 0;  % moment about y-axis
M3 = 0;  % moment about z-axis

H = [1e4, 1e10, 1e10];  % angular momentum vector


%% SOLVES DYNAMICS

% Initializations
t_span = linspace(0, 100, 5000);   % time span
I_COM = get_I_COM(rho, L0, m_c, m_p, R_c, R_p, R_t);  % MOI vector
initial_state = [H(1) / I_COM(1), H(2) / I_COM(2), H(3) / I_COM(3), L0];

% Set Tolerances
rel_tol = 1e-3;
abs_tol = 1e-6;
ode_opts = odeset('RelTol', rel_tol, 'AbsTol', abs_tol);

% ODE
dynamics = @(t,x) euler_dynamics(x, M1, M2, M3, m_p, m_c, rho, R_c, R_p, R_t);

% Solve ODE
[t, state] = ode45(dynamics, t_span, initial_state, ode_opts);

% Extract states
omega_x = state(:, 1);
omega_y = state(:, 2);
omega_z = state(:, 3);
L_vals = state(:, 4);


%% FUNCTIONS

% Determines angular accelerations
function dx = euler_dynamics(x, M1, M2, M3, m_p, m_c, rho, R_c, R_p, R_t)

    L = x(4);  % Length

    % Recalculates MOI's
    I_COM = get_I_COM(rho, L, m_c, m_p, R_c, R_p, R_t);

    % MOI's
    I1 = I_COM(1);
    I2 = I_COM(2);
    I3 = I_COM(3);
        
    d = getCtoCOM_L(L, rho, m_p, m_c); % Distance from counter mass to COM
    m_t = rho * L; % Tether mass

    % MOI derivatives with respect to L
    I1dot = 0;
    I2dot = 2*m_p*(L-d) + (1/6) * m_t * L + m_t * (L/2 - d);
    I3dot = 2*m_p*(L-d) + (1/6) * m_t * L + m_t * (L/2 - d);

    dx = zeros(3, 1);

    dx(1) = (M1 - I1dot*x(1) - x(2)*x(3)*(I3 - I2))/I1;
    dx(2) = (M2 - I2dot*x(2) - x(1)*x(3)*(I1 - I3))/I2;
    dx(3) = (M3 - I3dot*x(3) - x(1)*x(2)*(I2 - I1))/I3;
    dx(4) = 10;   % Rate L increases with respect to time
end

% Finds length from counter mass to COM
function [CtoCOM_L] = getCtoCOM_L(L, rho, m_p, m_c)
    CtoCOM_L = L * (1/2) * (rho*L + m_p) / (m_c + m_p + rho*L);
end

% Calculates tensor matrix about COM
function [I_COM] = get_I_COM(rho, L, m_c, m_p, R_c, R_p, R_t)

    % Gets length from counter mass to COM
    d = getCtoCOM_L(L, rho, m_p, m_c);

    % Mass of tether
    m_t = rho * L;

    % Calculates tensor matrices accounting for Parallel Axis Theorem
    I_c = [(2/5) * m_c * R_c^2, 0, 0;
          0, (2/5) * m_c * R_c^2 + m_c * d^2, 0;
          0, 0, (2/5) * m_c * R_c^2 + m_c * d^2];

    I_p = [(2/5) * m_p * R_p^2, 0, 0;
          0, (2/5) * m_p * R_p^2 + m_p * (L-d)^2, 0;
          0, 0, (2/5) * m_p * R_p^2 + m_p * (L-d)^2];

    I_t = [(1/2) * m_t * R_t^2, 0, 0;
          0, (1/12) * m_t * L^2 + m_t * (L/2 - d)^2, 0;
          0, 0, (1/12) * m_t * L^2 + m_t * (L/2 - d)^2];

    I_tot = I_c + I_p + I_t;

    % Outputs MOI about each axis as vector
    I_COM = [I_tot(1), I_tot(5), I_tot(9)];
end


%% PLOTS

figure(1);

subplot(4, 1, 1);
plot(t, omega_x, 'b-', 'LineWidth', 2);
title('Angular Velocity about X axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 2);
plot(t, omega_y, 'b-', 'LineWidth', 2);
title('Angular Velocity about Y axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 3);
plot(t, omega_z, 'b-', 'LineWidth', 2);
title('Angular Velocity about Z axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 4);
plot(t, L_vals, 'r-', 'LineWidth', 2);
title('Cable Length L over Time');
xlabel('Time (s)');
ylabel('Length (m)');
grid on;
