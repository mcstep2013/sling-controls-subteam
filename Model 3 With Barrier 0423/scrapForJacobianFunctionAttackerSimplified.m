clear
clc
close all

syms xStateFull [2,1] real
syms uIn [2,1] real
syms sysParams [3,1] real;

%% From attackerEOMCartesian.m

x=xStateFull(1);
y=xStateFull(2);

theta=uIn(1);
v=uIn(2);

m=sysParams(1);
I=sysParams(2);
g=sysParams(3);

xdotOut=[v*cos(theta); v*sin(theta)];

%% Computing Jacobians
dfdx=jacobian(xdotOut,xStateFull);
dfdu=jacobian(xdotOut,uIn);

%% Creating a function for them
JacobianFunc=matlabFunction(dfdx, dfdu,'Vars', {xStateFull, uIn, sysParams}, ...
          'File', 'attackerJacobiansCartesianSimplified');

