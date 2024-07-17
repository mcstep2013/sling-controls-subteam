function [Xplus, Pplus] = EKF_attacker_est_defender(Xk, Uin, Q, R, Pk, y)

% f is the nonlinear state equations
% Xplus is the state
% Pplus is the covariance
% h is the nonlinear measurement equation
% y is the measurement
% Q is the covariance of the process noise
% R is the covariance of the measurement noise
% Ajaco is the jacobian matrix of the state equations
% Hjaco is the jacobian matrix of the measurement


% Retrieve physical parameters from pvstate
l1 = 0.45; %pvstate(1);    % Length of link 1 (m)
s1 = l1/2; %pvstate(2);    % Distance from joint 1 to center of mass of link 1 (m)
m1 = 0.2; %pvstate(3);    % Mass of link 1 (kg)
J1 = (1/3)*m1*l1^2; %pvstate(4);    % Moment of inertia of link 1 (kg*m^2)
l2 = 0.35; %pvstate(5);    % Length of link 2 (m)
s2 = l2/2; %pvstate(6);    % Distance from joint 2 to center of mass of link 2 (m)
m2 = 0.6; %pvstate(7);    % Mass of link 2 (kg)
J2 = (1/3)*m2*l2^2; %pvstate(8);    % Moment of inertia of link 2 (kg*m^2)
g  = 9.81; %pvstate(9);    % Acceleration due to gravity (m/s^2)

% Measurement equation (h)
Hmat=eye(4);

% Xminus and Pminus are the propagated state and
% covariance
q1 = Xk(1);
q2 = Xk(2);
q1dot = Xk(3);
q2dot = Xk(4);
tau(1,1) = Uin(1);
tau(2,1) = Uin(2);
% Manipulator inertia matrix
H = zeros(2, 2);
H(1, 1) = m1 * s1^2 + J1 + m2 * (l1^2 + s2^2 + 2 * l1 * s2 * cos(q2)) + J2;
H(1, 2) = m2 * l1 * s2 * cos(q2) + m2 * s2^2 + J2;
H(2, 1) = H(1, 2);
H(2, 2) = m2 * s2^2 + J2;
% Coriolis matrix
h = m2 * l1 * s2 * sin(q2);
C = zeros(2, 2);
C(1, 1) = -h * q2dot;
C(1, 2) = -h * (q1dot + q2dot);
C(2, 1) =  h * q1dot;
% Gravity vector
G = zeros(2, 1);
G(1) = m1 * s1 * g * cos(q1) + m2 * g * (l2 * cos(q1 + q2) + l1 * cos(q1));
G(2) = m2 * s2 * g * cos(q1 + q2);
% Model equation
dxdt = [q1dot; q2dot; H \ (tau - C * [q1dot; q2dot] - G)];

Xminus = Xk+dxdt*2e-3;

% A is the Jacobian matrix at Xplus
% A = subs(Ajaco, x, Xk);
A = EKF_attacker_jacobHelper(Xk, Uin);

Pminus = A*Pk*A' + Q;

% hx is the predicted measurement
% hx = h(Xminus);
hx = Hmat*Xminus;

% H is the Jacobian matrix at Xminus
% H = subs(Hjaco, x, Xminus);
% H = hx;

% Compute Kalman Gain
L = Pminus*Hmat'*inv(Hmat*Pminus*Hmat' + R);
% Measurement Update equations
% Xplus and Pplus are the updated state and
% covariance
Xplus = double(Xminus + L*(y - hx));
Pplus = double(Pminus - L*Hmat*Pminus);


end