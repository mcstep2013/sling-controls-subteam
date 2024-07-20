clear; clc; close all;
% twolinkStateFcn
%   dxdt = twolinkStateFcn(x, tau, pvstate)
% 
%   State-space function for the unextensible tether rendezvous problem
%
%   Inputs:
%       - x         Current state vector (size 12, for s/c, tether COM, tether angle and length)
%       - u         Control action vector at the previous interval
%       - pvstate   Vector containing various stage parameters
%
%   Output:
%       - dxdt      State derivative vector

% Copyright 2023 The MathWorks, Inc.

% Spcecraft States

syms x [15,1] real
syms u [4,1] real
syms pvstate [1,1] real

x1=x(1);
x2=x(2);
x3=x(3);
x1d=x(4);
x2d=x(5);
x3d=x(6);
%Tether COM states
x1tt=x(7);
x2tt=x(8);
x3tt=x(9);
x1ttd=x(10);
x2ttd=x(11);
x3ttd=x(12);
%Tether orientation and length states
l=x(13);
alpha=x(14);
alphad=x(15);

%inputs
ux=u(1);
uy=u(2);
uz=u(3);
ld=u(4);

% Retrieve physical parameters from pvstate
mu=pvstate(1); %nondimensionalized grav parameter

% EOM for s/c
d = sqrt((x1+mu)^2 + x2^2 + x3^2);
r = sqrt((x1-1+mu)^2 + x2^2 + x3^2);
xdot(1)=x1d;
xdot(2)=x2d;
xdot(3)=x3d;
xdot(4)=2*x2d + x1 - (1-mu)*(x1+mu)/(d^3) - mu*(x1-1+mu)/(r^3) + ux;
xdot(5)=-2*x1d + x2 - (1-mu)*x2/(d^3) - mu*x2/(r^3) + uy;
xdot(6)=-(1-mu)*x3/(d^3) - mu*x3/r^3 + uz;

%EOM for tether COM
dtt = sqrt((x1tt+mu)^2 + x2tt^2 + x3tt^2);
rtt = sqrt((x1tt-1+mu)^2 + x2tt^2 + x3tt^2);
xdot(7)=x1ttd;
xdot(8)=x2ttd;
xdot(9)=x3ttd;
xdot(10)=2*x2ttd + x1tt - (1-mu)*(x1tt+mu)/(dtt^3) - mu*(x1tt-1+mu)/(rtt^3);
xdot(11)=-2*x1ttd + x2tt - (1-mu)*x2tt/(dtt^3) - mu*x2tt/(rtt^3);
xdot(12)=-(1-mu)*x3tt/(dtt^3) - mu*x3tt/rtt^3;

%EOM for tether rotation and length
xdot(13)=ld;
xdot(14)=alphad;
xdot(15)=-3*alphad*ld/l;

%% Computing jacobians
stateJac=jacobian(xdot,x);
inputJac=jacobian(xdot,u);

% %% Creating a function for them
% JacobianFunc=matlabFunction(stateJac, inputJac,'Vars', {x, u, pvstate}, ...
%           'File', 'ExtendableTetherJacobians');
