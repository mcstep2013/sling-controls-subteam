%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Matthew Stephens (7/23/2024)
% Obj 3 PMP min energy
%
%   This script plots 3 satellite orbits in the cr3bp problem with
%   different methods
%
% Note: script uses ...
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

format long

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% options
fsolve_opts = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'OptimalityTolerance', 1e-8);
ode_opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);


% parameters
G = 6.6743e-20;
R_Earth = 6378.1;
R_Moon = 1737.4;
m_Earth = 5.974E24;
m_Moon = 7.348E22;
mu_Earth = 3.968E5;
mu_Moon = 4.9028695E3;
P_Moon = 27.3*24*3600;

d_EarthMoon = 3.8475E5; % [km]
d_EarthBary = 4671; % [km]
GM_EarthMoon = 4.0350E5; % [km^3/s^2]
mu = m_Moon/(m_Earth+m_Moon);

l_star = d_EarthMoon;
m_star = m_Earth+m_Moon;

n_Moon = sqrt(G*m_star/l_star^3);
t_star = sqrt(l_star^3/(G*m_star));


% other parameters
delta = 0.99;
uT_max = 0.001/(l_star/t_star^2);
ldot_max = 0.01/(l_star/t_star);


% boundary conditions

tau0 = 0.3133;
proptime = 0.956394366196015;

% works better if prop time is longer for some reason
%proptime = 0.956394366196015; % this is deltatau

tauf = tau0 + proptime;

l0 = 200 / l_star;
alpha0 = 0.572; 
%alpha0 = 0;
alpha_tilde = 710.68;  % Back calculated from the sim
alphadot0 = (1+alpha_tilde)*n_Moon;

x0 = [0.7472 0.3784 0.12 0.4912 0.0116 -0.1739 l0 alpha0 alphadot0]'; % for payload spacecraft
x0_cm_tether = [0.831987145310295 0 0.125755524140790 0 0.239887839725451 0]'; % from saved data L1 Halos folder


% initial arbitrary guess
lambda0_guess = [-5.52841219848958 9.21855598420545 0.117183146701839 -1.96430544916108 2.46838169719764 0.0379798368692089 0 0 0]'; % elements 1-6 from saved data minimum energy solutions


% Solve the nonlinear system using fsolve
npts = 1000;
[lambda0,Psi,exitflag,output] = fsolve(@shootingfunction, lambda0_guess, fsolve_opts, x0, tau0, tauf, ...
    mu, npts, delta, t_star, uT_max, ldot_max, x0_cm_tether, ode_opts);


%% Simulate trajectory
tau_vec = linspace(tau0,tauf,npts);

% propagate trajectory
X0 = [x0; lambda0];
[~,X] = ode45(@augEOM, tau_vec, X0, ode_opts, mu, delta, uT_max, ldot_max);
x = X(:,1:9)';
lambda = X(:,10:18)';

u = NaN(4,npts);
for ct=1:npts
    gamma = -3*x(9,ct)/x(7,ct);
    B = [zeros(3,4); eye(4); zeros(1,4); zeros(1,3) gamma];
    B_payload = B(:,1:3);
    B_tether = B(:,4);
    uT = -B_payload'*lambda(:,ct)/(2*delta);
    ldot = -B_tether'*lambda(:,ct)/(2*(1-delta));
    u(:,ct) = [uT; 
               ldot];
end

uT_mag = vecnorm(u(1:3,:)*l_star/t_star^2)';
fuel_cost = trapz(tau_vec*t_star,uT_mag);
fuel_cost_dim = fuel_cost*l_star/t_star^2;



% trajectory
figure(1)
scatter3(x(1,1),x(2,1),x(3,1),15,'k','filled')
hold on
scatter3(x(1,end),x(2,end),x(3,end),35,'kp','filled')
hold on
plot3(x(1,:),x(2,:),x(3,:),'r')
grid on
title('controlled trajectory')
xlabel('x_1')
ylabel('x_2')
ylabel('x_3')
legend('t_0','t_f','trajectory')
axis equal

% x, lambda, and u vs t
figure(2)
subplot(2,2,1);
plot(tau_vec,x(1,:),'g')
hold on
plot(tau_vec,x(2,:),'m')
hold on
plot(tau_vec,x(3,:),'c')
grid on
title('r vs t')
xlabel('t')
ylabel('r')
xlim([tau0 tauf]);
legend('x','y','z')

subplot(2,2,2);
plot(tau_vec,x(4,:),'g')
hold on
plot(tau_vec,x(5,:),'m')
hold on
plot(tau_vec,x(6,:),'c')
grid on
title('v vs t')
xlabel('t')
ylabel('v')
xlim([tau0 tauf]);
legend('v_x','v_y','v_z')

subplot(2,2,3);
plot(tau_vec,lambda(1,:),'g')
hold on
plot(tau_vec,lambda(2,:),'m')
hold on
plot(tau_vec,lambda(3,:),'c')
grid on
title('lambda_r vs t')
xlabel('t')
ylabel('lambda_r')
xlim([tau0 tauf]);
legend('lambda_x','lambda_y','lambda_z')

subplot(2,2,4);
plot(tau_vec,lambda(4,:),'g')
hold on
plot(tau_vec,lambda(5,:),'m')
hold on
plot(tau_vec,lambda(6,:),'c')
grid on
title('lambda_v vs t')
xlabel('t')
ylabel('lambda_v')
xlim([tau0 tauf]);
legend('lambda_vx','lambda_vy','lambda_vz')

figure(3)
plot(tau_vec,u(1,:),'g')
hold on
plot(tau_vec,u(2,:),'m')
hold on
plot(tau_vec,u(3,:),'c')
grid on
title('u vs t')
xlabel('t')
ylabel('u')
xlim([tau0 tauf]);
legend('u_1','u_2','u_3')


%% Functions

% shooting function required for finding initial costate (lambda0)
function Psi = shootingfunction(lambda0, x0, tau0, tauf, mu, npts, delta, t_star, uT_max, ldot_max, x0_cm_tether, ode_opts)

    tau_vec = linspace(tau0,tauf,npts);
    X0 = [x0; lambda0];
    [tau_hist,X] = ode45(@augEOM, tau_vec, X0, ode_opts, mu, delta, uT_max, ldot_max);
    x = X(:,1:9)';
    xf = X(end,1:9)';
    lf = xf(7,end);
    alphaf = xf(8,end);
    alphadotf = xf(9,end);
    lambda = X(:,10:18)';
    
    ldot = NaN(1,length(tau_hist));
    for ct=1:length(tau_hist)
        gamma = -3*x(9,ct)/x(7,ct);
        B = [zeros(3,4); eye(4); zeros(1,4); zeros(1,3) gamma];
        B_tether = B(:,4);
        ldot(ct) = -B_tether'*lambda(:,ct)/(2*(1-delta));
    end

    [x_tip_tether] = tether_tip_state(tauf, tau0, t_star, lf, ldot(end), alphaf, alphadotf);
    
    [~,x_cm_tether] = ode45(@tether_cm_EOM, tau_vec, x0_cm_tether, ode_opts, mu);
    xf_cm_tether = x_cm_tether(end,:)';

    xf_target = xf_cm_tether+x_tip_tether(1:6);

    Psi = xf_target - xf(1:6);
end

% EOMs for state and costate
function [dX] = augEOM(tau, X, mu, delta, uT_max, ldot_max)
    
    x = X(1:9); % state
    lambda = X(10:18); % costate

    % nondim state and opt control input
    r = x(1:3);
    v = x(4:6);
    l = x(7);
    alphadot = x(9);
    gamma = -3*alphadot/l;

    r13 = sqrt((r(1)+mu)^2+r(2)^2+r(3)^2);
    r23 = sqrt((r(1)-1+mu)^2+r(2)^2+r(3)^2);

    % Opt cntrl input
    B = [zeros(3,4); eye(4); zeros(1,4); zeros(1,3) gamma];
    B_payload = B(:,1:3);
    B_tether = B(:,4);

    uT = -B_payload'*lambda/(2*delta);
    if norm(uT) > uT_max
        uT = uT_max*uT/norm(uT);
    end
    ldot = -B_tether'*lambda/(2*(1-delta));
    if ldot > ldot_max
        ldot = ldot_max;
    end
    u = [uT; ldot];

    % Identify f and A (df/dX)
    f = [v(1);
         v(2);
         v(3);
         2*v(2)+r(1)-(1-mu)*(r(1)+mu)/r13^3-mu*(r(1)-1+mu)/r23^3;
         -2*v(1)+r(2)-(1-mu)*r(2)/r13^3-mu*r(2)/r23^3;
         -(1-mu)*r(3)/r13^3-mu*r(3)/r23^3;
         0;
         alphadot;
         0];

    A1 = zeros(3);
    A2 = eye(3);
    A3 = [1+(1-mu)*(3*(r(1)+mu)^2-r13^2)/r13^5+mu*(3*(r(1)-1+mu)^2-r23^2)/r23^5 3*(1-mu)*(r(1)+mu)*r(2)/r13^5+3*mu*(r(1)-1+mu)*r(2)/r23^5 3*(1-mu)*(r(1)+mu)*r(3)/r13^5+3*mu*(r(1)-1+mu)*r(3)/r23^5;
          3*(1-mu)*(r(1)+mu)*r(2)/r13^5+3*mu*(r(1)-1+mu)*r(2)/r23^5 1+(1-mu)*(3*r(2)^2-r13^2)/r13^5+mu*(3*r(2)^2-r23^2)/r23^5 3*(1-mu)*r(3)*r(2)/r13^5+3*mu*r(3)*r(2)/r23^5;
          3*(1-mu)*(r(1)+mu)*r(3)/r13^5+3*mu*(r(1)-1+mu)*r(3)/r23^5 3*(1-mu)*r(2)*r(3)/r13^5+3*mu*r(2)*r(3)/r23^5 (1-mu)*(3*r(3)^2-r13^2)/r13^5+mu*(3*r(3)^2-r23^2)/r23^5];
    
    A4 = [0 2 0;
         -2 0 0;
          0 0 0];
    A = [A1 A2;
         A3 A4];
    
    M = [0 0 0;
         0 0 1;
         3*alphadot*ldot/l^2 0 -3*ldot/l];
    G = [A zeros(6,3);
         zeros(3,6) M];

    % Assign values in the state's derivative:
    dx = f+B*u;
    dlambda = -G'*lambda;
    dX = [dx; dlambda];

end

% EOMs for tether center of mass (in an uncontrolled L1 HALO)
function [dx] = tether_cm_EOM(tau, x, mu)

    % nondim state and opt control input
    r = x(1:3);
    v = x(4:6);

    r13 = sqrt((r(1)+mu)^2+r(2)^2+r(3)^2);
    r23 = sqrt((r(1)-1+mu)^2+r(2)^2+r(3)^2);

    % Identify f and A (df/dX)
    f0 = [
             v(1);
             v(2);
             v(3);
             2*v(2)+r(1)-(1-mu)*(r(1)+mu)/r13^3-mu*(r(1)-1+mu)/r23^3;
             -2*v(1)+r(2)-(1-mu)*r(2)/r13^3-mu*r(2)/r23^3;
             -(1-mu)*r(3)/r13^3-mu*r(3)/r23^3
        ];


    % Assign values in the state's derivative:
    dx = f0;

end

% Finding the position, velocity, and acceleration (rva) of the tether tip
% in the synodic frame from rotation matrices and transport theorem
function [rva] = tether_tip_state(tau, tau0, tstar, l, ldot, alpha, alphadot)

    n = 1/tstar;
    beta = tau - tau0 -alpha;
    betadot = n - alphadot;
    alphaddot = -3*alphadot*ldot/l;
    

    C = [cos(beta) -sin(beta) 0;
         sin(beta)  cos(beta) 0;
         0          0         1];

    C_tilde = [-sin(beta)  cos(beta) 0;
               -cos(beta) -sin(beta) 0;
                0          0         0];

    r_body = [l 0 0]';
    v_body = [ldot 0 0]';
    a_body = zeros(3,1);

    r_syn = C*r_body;
    v_syn = betadot*C_tilde*r_body + C*v_body;
    a_syn = -(betadot^2*C + alphaddot*C_tilde)*r_body + 2*betadot*C_tilde*v_body + C*a_body;

    rva = [r_syn;
           v_syn;
           a_syn];

end