clear;
clc;
format long;

% MOMENT OF INERTIA FUNCTIONS ------------------------------------------

% Moment of inertia
function I = I_fun(t)
    M_cw = 50000; % mass of countermass
    M_pay = 1000; % mass of payload
    density = 1; % linear density of cable (mass/length)
    lengthTotal = 1000; % total length of cable
    r_cw = 1; % radius of countermass
    r_pay = 1; % radius of payload

    r = t + 10; % radius from countermass to center of mass

    % total mass of system
    M_tot = M_cw + M_pay + density*lengthTotal;

    % Length of spooled out cable (r+R)
    R_tot = (-M_pay + sqrt(M_pay^2 + 2*density*M_tot*r))/density;

    % Counterweight MOI
    I_cw = (4/3)*(M_cw + density*(lengthTotal - R_tot))*r_cw^2;

    % Extended wire MOI
    I_wire = (1/3)*density*R_tot.^3;

    % Payload MOI
    I_pay = M_pay*R_tot.^2 + (4/3)*M_pay*r_pay^2;

    % Total MOI about center of mass
    I = I_cw + I_wire + I_pay - M_tot*r.^2;
end

% Derivative of MOI using finite difference
function dI = dI_fun(t)
    dt = 1e-5;
    dI = (I_fun(t+dt) - I_fun(t-dt)) / (2 * dt);
end

% KINEMATICS -----------------------------------------------------------
L = 1e10; % angular momentum
tspan = [0 75]; % time span
theta0 = 0; % initial angle

angle_ode = @(t, theta) L / I_fun(t);

[t, theta] = ode45(angle_ode, tspan, theta0);

omega = L ./ I_fun(t);
alpha = -(omega ./ I_fun(t)) .* dI_fun(t);

% PLOTS ----------------------------------------------------------------
figure(1)
subplot(3,1,1);
plot(t, theta)
xlabel('Time')
ylabel('Angle \theta')
title('Angle v. Time')
grid on

subplot(3,1,2);
plot(t, omega)
xlabel('Time')
ylabel('Angular Velocity \omega')
title('Angular Velocity v. Time')
grid on

subplot(3,1,3);
plot(t, alpha)
xlabel('Time')
ylabel('Angular Acceleration \alpha')
title('Angular Acceleration v. Time')
grid on