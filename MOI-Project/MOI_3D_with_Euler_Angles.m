clear;
clf('reset');

%% INITIAL CONDITIONS

rho = 1;      % linear density of cable (mass/length)
L0 = 500;     % total length of cable

m_c = 5000;  % mass of countermass
m_p = 1000;   % mass of payload

R_c = 1;      % radius of countermass
R_p = 1;      % radius of payload
R_t = 0.01;   % radius of tether

M1 = 0;  % moment about x-axis
M2 = 0;  % moment about y-axis
M3 = 0;  % moment about z-axis

H = [1e4, 1e9, 1e9];  % angular momentum vector


%% SOLVES DYNAMICS

% Initializations
t_span = linspace(0, 20, 1000);   % time span
I_COM = get_I_COM(rho, L0, m_c, m_p, R_c, R_p, R_t);  % MOI vector
initial_state = [H(1) / I_COM(1), H(2) / I_COM(2), H(3) / I_COM(3), L0, 0, 0, 0]; % Angular velocities (x,y,z), Initial Length, Euler Angles

% Set Tolerances
rel_tol = 1e-6;
abs_tol = 1e-9;
ode_opts = odeset('RelTol', rel_tol, 'AbsTol', abs_tol);

% ODE
dynamics = @(t,x) euler_dynamics(t, x, M1, M2, M3, m_p, m_c, rho, R_c, R_p, R_t);

% Solve ODE
[t, state] = ode45(dynamics, t_span, initial_state, ode_opts);

% Extract states
omega_x = state(:, 1);
omega_y = state(:, 2);
omega_z = state(:, 3);
L_vals = state(:, 4);
yaw_vals   = state(:, 5);
pitch_vals = state(:, 6);
roll_vals  = state(:, 7);


%% FUNCTIONS

% Determines angular accelerations
function dx = euler_dynamics(t, x, M1, M2, M3, m_p, m_c, rho, R_c, R_p, R_t)

    omega = x(1:3); % Angular velocities
    L = x(4);       % Length
    theta = x(5:7); % Euler angles
    L_dot = 10;     % Rate of change of the tether length

    % Recalculates MOI's
    I_COM = get_I_COM(rho, L, m_c, m_p, R_c, R_p, R_t);

    % MOI's
    I1 = I_COM(1);
    I2 = I_COM(2);
    I3 = I_COM(3);
        
    d = getCtoCOM_L(L, rho, m_p, m_c); % Distance from counter mass to COM
    m_t = rho * L; % Tether mass

    % MOI derivatives with respect to L
    if L_dot == 0
        I1dot = 0;
        I2dot = 0;
        I3dot = 0;
    elseif L_dot > 0
        I1dot = 0;
        I2dot = L_dot * (2*m_p*(L-d) + (1/6) * m_t * L + m_t * (L/2 - d));
        I3dot = L_dot * (2*m_p*(L-d) + (1/6) * m_t * L + m_t * (L/2 - d)); 
    end

    % Euler angle derivatives (using B(theta) * omega)
    B = bodyToEuler(theta); % 3-2-1 Rotation
    theta_dot = B * omega;

    dx = zeros(7, 1);

    dx(1) = (M1 - I1dot*x(1) - x(2)*x(3)*(I3 - I2))/I1;
    dx(2) = (M2 - I2dot*x(2) - x(1)*x(3)*(I1 - I3))/I2;
    dx(3) = (M3 - I3dot*x(3) - x(1)*x(2)*(I2 - I1))/I3;
    dx(4) = L_dot;

    % Euler angle derivatives
    dx(5:7) = theta_dot; % Update Euler angles
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

% Body to Euler Rate Matrix
function B = bodyToEuler(theta)
    %theta1 = theta(1); % yaw
    theta2 = theta(2); % pitch
    theta3 = theta(3); % roll
    
    B = (1/cos(theta2)) * ...
        [0, sin(theta3), cos(theta3);
         0, cos(theta2)*cos(theta3), -cos(theta2)*sin(theta3);
         cos(theta2), sin(theta2)*sin(theta3), sin(theta2)*cos(theta3)];
end


%% PLOTS

figure(1);

subplot(4, 1, 1);
plot(t, rad2deg(omega_x), 'b-', 'LineWidth', 2);
title('Angular Velocity about X axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 2);
plot(t, rad2deg(omega_y), 'b-', 'LineWidth', 2);
title('Angular Velocity about Y axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 3);
plot(t, rad2deg(omega_z), 'b-', 'LineWidth', 2);
title('Angular Velocity about Z axis');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
grid on;

subplot(4, 1, 4);
plot(t, L_vals, 'r-', 'LineWidth', 2);
title('Cable Length vs Time');
xlabel('Time (s)');
ylabel('Length (m)');
grid on;

figure(2);

subplot(3, 1, 1);
plot(t, mod(rad2deg(yaw_vals), 360), 'b-', 'LineWidth', 2);
title('Yaw vs Time');
xlabel('Time (s)');
ylabel('Yaw (degrees)');
grid on;
ylim([0 360]);

subplot(3, 1, 2);
plot(t, mod(rad2deg(pitch_vals), 360), 'b-', 'LineWidth', 2);
title('Pitch vs Time');
xlabel('Time (s)');
ylabel('Pitch (degrees)');
grid on;
ylim([0 360]);

subplot(3, 1, 3);
plot(t, mod(rad2deg(roll_vals), 360), 'b-', 'LineWidth', 2);
title('Roll vs Time');
xlabel('Time (s)');
ylabel('Roll (degrees)');
grid on;
ylim([0 360]);

%% 3D ANIMATION SETUP

% Create rotation matrices for each time step
R = zeros(3, 3, length(t));
for i = 1:length(t)
    % Create rotation matrix from Euler angles (yaw, pitch, roll)
    % ZYX rotation sequence (yaw -> pitch -> roll)
    cy = cos(yaw_vals(i));  % Fixed: use yaw_vals instead of yaw
    sy = sin(yaw_vals(i));  % Fixed: use yaw_vals instead of yaw
    cp = cos(pitch_vals(i)); % Fixed: use pitch_vals instead of pitch
    sp = sin(pitch_vals(i)); % Fixed: use pitch_vals instead of pitch
    cr = cos(roll_vals(i));  % Fixed: use roll_vals instead of roll
    sr = sin(roll_vals(i));  % Fixed: use roll_vals instead of roll
    
    % ZYX rotation matrix
    R(:,:,i) = [cp*cy, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
                cp*sy, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
                -sp, cp*sr, cp*cr];
end

% Calculate positions of countermass, COM, and payload at each time
positions = zeros(3, 3, length(t)); % [countermass; COM; payload] x [x,y,z] x time

for i = 1:length(t)
    % Calculate d (distance from countermass to COM)
    L = L_vals(i);
    d = getCtoCOM_L(L, rho, m_p, m_c);
    
    % In the initial frame, the tether is along the x-axis
    % COM is at origin (0,0,0)
    % Countermass at (-d, 0, 0)
    % Payload at (L-d, 0, 0)
    baseline = [-d, 0, 0;
                0, 0, 0;
                L-d, 0, 0];
    
    % Rotate these positions according to the rotation matrix for this time
    for j = 1:3  % For countermass, COM, payload
        positions(j,:,i) = (R(:,:,i) * baseline(j,:)')';
    end
end

% Create rotation matrices for each time step
R = zeros(3, 3, length(t));
for i = 1:length(t)
    % Create rotation matrix from Euler angles (yaw, pitch, roll)
    % ZYX rotation sequence (yaw -> pitch -> roll)
    cy = cos(yaw_vals(i));
    sy = sin(yaw_vals(i));
    cp = cos(pitch_vals(i));
    sp = sin(pitch_vals(i));
    cr = cos(roll_vals(i));
    sr = sin(roll_vals(i));
    
    % ZYX rotation matrix
    R(:,:,i) = [cp*cy, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
                cp*sy, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
                -sp, cp*sr, cp*cr];
end

% Calculate positions of countermass, COM, and payload at each time
positions = zeros(3, 3, length(t)); % [countermass; COM; payload] x [x,y,z] x time

for i = 1:length(t)
    % Calculate d (distance from countermass to COM)
    L = L_vals(i);
    d = getCtoCOM_L(L, rho, m_p, m_c);
    
    % In the initial frame, the tether is along the x-axis
    % COM is at origin (0,0,0)
    % Countermass at (-d, 0, 0)
    % Payload at (L-d, 0, 0)
    baseline = [-d, 0, 0;
                0, 0, 0;
                L-d, 0, 0];
    
    % Rotate these positions according to the rotation matrix for this time
    for j = 1:3  % For countermass, COM, payload
        positions(j,:,i) = (R(:,:,i) * baseline(j,:)')';
    end
end

%% Animation
figure(3);
axis_limit = max(L_vals) * 1.2;  % Set axis limits based on tether length

% Initialize plot objects
h_countermass = scatter3([], [], [], 200, 'filled', 'MarkerFaceColor', 'b');
hold on;
h_payload = scatter3([], [], [], 100, 'filled', 'MarkerFaceColor', 'r');
h_com = scatter3(0, 0, 0, 150, 'filled', 'MarkerFaceColor', 'g');
h_tether = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);

% Initialize tether path trace with NaN values to avoid empty object error
trace_length = 30;  % Number of past positions to show
h_trace_c = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', [0, 0, 1, 0.5]);  % Blue with 0.5 alpha
h_trace_p = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', [1, 0, 0, 0.5]);  % Red with 0.5 alpha

axis([-axis_limit, axis_limit, -axis_limit, axis_limit, -axis_limit, axis_limit]);
grid on;
box on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Momentum Exchange Tether 3D Animation');
view(30, 20);  % Set initial view angle

% Add axis lines through origin
line([-axis_limit, axis_limit], [0, 0], [0, 0], 'Color', 'k', 'LineStyle', ':');
line([0, 0], [-axis_limit, axis_limit], [0, 0], 'Color', 'k', 'LineStyle', ':');
line([0, 0], [0, 0], [-axis_limit, axis_limit], 'Color', 'k', 'LineStyle', ':');

% Add information text (static position)
h_info = annotation('textbox', [0.02, 0.02, 0.1, 0.1], 'String', '', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black');

% Add a legend with clear labels instead of moving text
legend([h_countermass, h_payload, h_com], {'Countermass', 'Payload', 'COM'}, ...
    'Location', 'northeast');

% Animation loop
for i = 1:length(t)
    % Update countermass and payload positions
    countermass_pos = squeeze(positions(1,:,i));
    payload_pos = squeeze(positions(3,:,i));
    
    % Update scatter plots
    set(h_countermass, 'XData', countermass_pos(1), 'YData', countermass_pos(2), 'ZData', countermass_pos(3));
    set(h_payload, 'XData', payload_pos(1), 'YData', payload_pos(2), 'ZData', payload_pos(3));
    
    % Update tether line
    set(h_tether, 'XData', [countermass_pos(1), payload_pos(1)], ...
                 'YData', [countermass_pos(2), payload_pos(2)], ...
                 'ZData', [countermass_pos(3), payload_pos(3)]);
    
    % Update traces - show path of countermass and payload
    start_idx = max(1, i - trace_length);
    
    % Extract path data for trace
    c_trace_x = zeros(i - start_idx + 1, 1);
    c_trace_y = zeros(i - start_idx + 1, 1);
    c_trace_z = zeros(i - start_idx + 1, 1);
    
    p_trace_x = zeros(i - start_idx + 1, 1);
    p_trace_y = zeros(i - start_idx + 1, 1);
    p_trace_z = zeros(i - start_idx + 1, 1);
    
    for j = start_idx:i
        idx = j - start_idx + 1;
        c_trace_x(idx) = positions(1,1,j);
        c_trace_y(idx) = positions(1,2,j);
        c_trace_z(idx) = positions(1,3,j);
        
        p_trace_x(idx) = positions(3,1,j);
        p_trace_y(idx) = positions(3,2,j);
        p_trace_z(idx) = positions(3,3,j);
    end
    
    % Update trace plots
    set(h_trace_c, 'XData', c_trace_x, 'YData', c_trace_y, 'ZData', c_trace_z);
    set(h_trace_p, 'XData', p_trace_x, 'YData', p_trace_y, 'ZData', p_trace_z);
    
    % Update info text
    info_str = sprintf('Time: %.2f s', t(i)); %\nYaw: %.1f°\nPitch: %.1f°\nRoll: %.1f°\nTether Length: %.1f m', ...
    %                  t(i), yaw_vals(i), pitch_vals(i), roll_vals(i), L_vals(i));
    set(h_info, 'String', info_str);
    
    drawnow;
    pause(0.001);  % Control animation speed
end