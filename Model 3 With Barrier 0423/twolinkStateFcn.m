function dxdt = twolinkStateFcn(x, tau, pvstate)
% twolinkStateFcn
%   dxdt = twolinkStateFcn(x, tau, pvstate)
% 
%   State-space function for the two-link robot manipulator example. The
%   states x are joint angles and velocities. The control input is the
%   torque tau.
%
%   Inputs:
%       - x         Current state vector
%       - u         Control action vector at the previous interval
%       - pvstate   Vector containing various stage parameters
%
%   Output:
%       - dxdt      State derivative vector

% Copyright 2023 The MathWorks, Inc.

% States
q1 = x(1);
q2 = x(2);
q1dot = x(3);
q2dot = x(4);

% Retrieve physical parameters from pvstate
l1 = pvstate(1);    % Length of link 1 (m)
s1 = pvstate(2);    % Distance from joint 1 to center of mass of link 1 (m)
m1 = pvstate(3);    % Mass of link 1 (kg)
J1 = pvstate(4);    % Moment of inertia of link 1 (kg*m^2)
l2 = pvstate(5);    % Length of link 2 (m)
s2 = pvstate(6);    % Distance from joint 2 to center of mass of link 2 (m)
m2 = pvstate(7);    % Mass of link 2 (kg)
J2 = pvstate(8);    % Moment of inertia of link 2 (kg*m^2)
g  = pvstate(9);    % Acceleration due to gravity (m/s^2)

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
end