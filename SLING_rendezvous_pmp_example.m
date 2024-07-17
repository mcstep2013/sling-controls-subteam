%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Matthew Stephens (7/15/2024)
% Indirect Method Example Script
%
%   This script completes a minimum-energy orbital transfer problem using
%   Pontryagin's Minimum Principle to derive the EOMs. The nonlinear solver
%   used to find the initial costates was fsolve. Please refer to AAE590ACA
%   notes for useful derivations. Note that this is a fixed-time two point
%   boundary value problem. A free time, minimum-time, or minimum-fuel
%   problem would require a different analytical derivation.  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

format long

tic

%% Parameters

% Constants
mu_Earth = 398600;                  % Gravitational parameter (km3/s2)

% Boundary conditions and timespan
t0 = 0;                             % Initial time (s)
tf = 6000;                          % Final time (s)
npts = 10000;
t_span = linspace(t0,tf,npts);      % Timespan (s)
r0_dim = [7000; 0; 0];              % Initial position (km)
v0_dim = [0; 7.5; 0];               % Initial velocity (km/s)
rf_dim = [42164; 0; 0];             % Target position (km)
vf_dim = [0; 3.07; 0];              % Target velocity (km/s)

% Characteristic parameters (for nondimensionalization)
l_star = norm(r0_dim);              % Characteristic length (km)
t_star = sqrt(l_star^3/mu_Earth);   % Characteristic time (s)

% Nondimensional boundary conditions
r0 = r0_dim/l_star; 
v0 = v0_dim/(l_star/t_star);
x0 = [r0; v0];
rf = rf_dim/l_star;
vf = vf_dim/(l_star/t_star);
xf = [rf; vf];
tau_span = t_span/t_star;

%% Solve the nonlinear system using fsolve

% Options and tolerances
fsolve_opts = optimoptions('fsolve', 'OptimalityTolerance', 1e-8);
ode_opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

% Obtaining initial costate through nonlinear solver
lambda0_guess = zeros(6,1);         % Initial costate guess  
[lambda0,Psi,exitflag,~] = fsolve(@shootingfunction, lambda0_guess, ...
                                   fsolve_opts, x0, xf, ...
                                   tau_span, ode_opts);

% Given that an initial costate guess is close enough to the actual 
% initial costate, the solver will converge. Psi represents the value of 
% the shooting function when lambda0 is passed (this should be nearly 
% zero). The exitflag returns 1 if fsolve was successful and 0 if it was 
% not. If 0 is returned, another initial costate guess should be tried.

% Propagate uncontrolled trajectory
X0_uctl = [x0; zeros(6,1)];    % Augmented initial state [x0; lambda0]
[~,X_uctl] = ode45(@augEOM, tau_span, X0_uctl, ode_opts);
x_uctl = X_uctl(:,1:6)';       % Uncontrolled state 
lambda_uctl = X_uctl(:,7:12)'; % Uncontrolled costate 

% Propagate controlled trajectory
X0 = [x0; lambda0];
[t,X] = ode45(@augEOM, tau_span, X0, ode_opts);
x = X(:,1:6)';                 % Controlled state 
lambda = X(:,7:12)';           % Controlled state 

% Calculate time history of Hamiltonian
B = [zeros(3); eye(3)];
u = -1/2*B'*lambda;            % Control acceleration
H = NaN(1,length(t));          % Hamiltonian 
for c=1:length(t)
    r_vec = x(1:3,c);
    r = norm(r_vec);
    v_vec = x(4:6,c);
    f0 = [v_vec; -r_vec./r^3];
    f = f0+B*u(:,c);
    H(c) = norm(u(:,c))^2+lambda(:,c)'*f;
end

% Re-dimensionalizing 
x_dim = [x(1:3,:)*l_star; 
         x(4:6,:)*l_star/t_star];
lambda_dim = [lambda(1:3,:)*l_star; 
              lambda(4:6,:)*l_star/t_star];
x_uctl_dim = [x_uctl(1:3,:)*l_star; 
              x_uctl(4:6,:)*l_star/t_star];
lambda_uctl_dim = [lambda_uctl(1:3,:)*l_star; 
                   lambda_uctl(4:6,:)*l_star/t_star];
u_dim = u*l_star/t_star^2;

% Other important quantities
u_mag = vecnorm(u_dim)';       % Control accel magnitude (km/s2)
fuel_cost = trapz(t,u_mag);    % Delta v (km/s)


%% Relevant plots

% Trajectory
figure(1)
scatter3(x_dim(1,1),x_dim(2,1),x_dim(3,1),15,'k','filled')
hold on
scatter3(x_dim(1,end),x_dim(2,end),x_dim(3,end),35,'rp','filled')
scatter3(x_uctl_dim(1,end),x_uctl_dim(2,end),x_uctl_dim(3,end),35,'bp','filled')
plot3(x_dim(1,:),x_dim(2,:),x_dim(3,:),'r')
plot3(x_uctl_dim(1,:),x_uctl_dim(2,:),x_uctl_dim(3,:),'b')
grid on
title('Spacecraft trajectory','Interpreter','latex')
xlabel('$r_1$ (km)','Interpreter','latex')
ylabel('$r_2$ (km)','Interpreter','latex')
zlabel('$r_3$ (km)','Interpreter','latex')
legend('$t_0$','$r_{f,Controlled}$','$r_{f,Unontrolled}$','Controlled path','Uncontrolled path','Interpreter','latex')
axis square

% State and costate vs time
figure(2)
subplot(4,1,1);
plot(t_span,x_dim(1,:),'g')
hold on
plot(t_span,x_dim(2,:),'m')
plot(t_span,x_dim(3,:),'c')
grid on
title('Position vs time','Interpreter','latex')
xlabel('$t$ (s)','Interpreter','latex')
ylabel('$r$ (km)','Interpreter','latex')
xlim([t0 tf]);
legend('$r_1$','$r_2$','$r_3$','Interpreter','latex')

subplot(4,1,2);
plot(t_span,x_dim(4,:),'g')
hold on
plot(t_span,x_dim(5,:),'m')
plot(t_span,x_dim(6,:),'c')
grid on
title('Velocity vs time','Interpreter','latex')
xlabel('$t$ (s)','Interpreter','latex')
ylabel('$v$ (km/s)','Interpreter','latex')
xlim([t0 tf]);
legend('$v_1$','$v_2$','$v_3$','Interpreter','latex')

subplot(4,1,3);
plot(t_span,lambda_dim(1,:),'g')
hold on
plot(t_span,lambda_dim(2,:),'m')
plot(t_span,lambda_dim(3,:),'c')
grid on
title('Co-position vs time','Interpreter','latex')
xlabel('$t$ (s)')
ylabel('$\lambda_r$ (km)','Interpreter','latex')
xlim([t0 tf]);
legend('$\lambda_{r_1}$','$\lambda_{r_2}$','$\lambda_{r_3}$','Interpreter','latex')

subplot(4,1,4);
plot(t_span,lambda_dim(4,:),'g')
hold on
plot(t_span,lambda_dim(5,:),'m')
plot(t_span,lambda_dim(6,:),'c')
grid on
title('Co-velocity vs time','Interpreter','latex')
xlabel('$t$ (s)')
ylabel('$\lambda_v$ (km/s)','Interpreter','latex')
xlim([t0 tf]);
legend('$\lambda_{v_1}$','$\lambda_{v_2}$','$\lambda_{v_3}$','Interpreter','latex')

% Control accel vs time
figure(3)
plot(t_span,u_dim(1,:),'g')
hold on
plot(t_span,u_dim(2,:),'m')
plot(t_span,u_dim(3,:),'c')
plot(t_span,u_mag,'k')
grid on
title('Control acceleration vs time','Interpreter','latex')
xlabel('$t$ (s)','Interpreter','latex')
ylabel('$u$ (km/s2)','Interpreter','latex')
xlim([t0 tf]);
legend('$u_1$','$u_2$','$u_3$','$||u||$','Interpreter','latex')

% H-H0 vs t
figure(4)
plot(t_span,H-H(1),'k')
grid on
title('Hamiltonian variance vs time','Interpreter','latex')
xlabel('$t$ (s)','Interpreter','latex')
ylabel('$H-H_0$ (unitless)','Interpreter','latex')
xlim([t0 tf]);

toc

%% Functions

% Shooting function, representing satisfaction of boundary conditions
function Psi = shootingfunction(lambda0,x0,xf_target,tspan,ode_opts)

    X0 = [x0; lambda0];
    [~,X] = ode45(@augEOM, tspan, X0, ode_opts);
    xf = X(end,1:6)';
    Psi = xf_target - xf;

end

% Differential equations of motion for state and costate with optimal
% control
function [dX] = augEOM(t, X)
    
    x = X(1:6);                % State
    lambda = X(7:12);          % Costate

    r_vec = x(1:3);            % Position
    v_vec = x(4:6);            % Velocity
    r = norm(r_vec);           % Radius

    % State EOMs with optimal control input
    f0 = [v_vec; 
         -r_vec./r^3];         % Keplerian dynamics
    B = [zeros(3); eye(3)];    % Control state space matrix
    u_star = -1/2*B'*lambda;   % Optimal control input *check 590ACA notes*
    dx = f0+B*u_star;          % State EOMs

    % Costate EOMs
    dadr = -(1/r^3*eye(3)-3/ ...
        r^5*(r_vec*r_vec'));   % Derivative of accel wrt position
    G = [zeros(3) eye(3);      % Jacobian of f0
         dadr zeros(3)];
    dlambda = -G'*lambda;      % Costate EOMs *check 590ACA notes*

    % Augmented state EOMs
    dX = [dx; dlambda];        

end
