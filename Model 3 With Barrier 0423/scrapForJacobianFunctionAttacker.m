clear
clc
close all

syms xStateFull [6,1] real
syms uIn [2,1] real
syms sysParams [3,1] real;

%% From attackerEOMCartesian.m

x=xStateFull(1);
y=xStateFull(2);
theta=xStateFull(3);
xdot=xStateFull(4);
ydot=xStateFull(5);
thetadot=xStateFull(6);

tau=uIn(1);
f=uIn(2);

m=sysParams(1);
I=sysParams(2);
g=sysParams(3);

xdotOut=[xdot; ydot; thetadot; (f/m)*cos(theta); (f/m)*sin(theta)-g; tau/I];

%% Computing Jacobians
dfdx=jacobian(xdotOut,xStateFull);
dfdu=jacobian(xdotOut,uIn);

% %% Creating a function for them
% JacobianFunc=matlabFunction(dfdx, dfdu,'Vars', {xStateFull, uIn, sysParams}, ...
%           'File', 'attackerJacobiansCartesian');

