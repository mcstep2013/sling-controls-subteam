clear;clc;
syms x [15,1] real
syms u [4,1] real
syms pvcost [20,1] real

mu=pvcost(1); %nondimensionalized grav parameter
n=pvcost(2); %dimensional mean motion of earth-moon
timedimensional=pvcost(3);
p=pvcost(4); %prediction horizon

%Extract weight matrices
Sf = diag(pvcost(5:10));                 % Terminal Weight Matrix
Q  = diag(pvcost(11:16));                % State Weight Matrix
R  = diag(pvcost(17:20));    % Control Weight Matrix

%calculate state of tether tip:
xtip=getTetherTipState(x,u,[mu,n,timedimensional]);
xrel=x(1:6)-xtip;


% L = 0.5*xrel.'*Sf*xrel; %PT1
L = 0.5*xrel.'*Q*xrel + 0.5*u.'*R*u; %PT2
dLdx=jacobian(L,x);
dLdu=jacobian(L,u);

%% Creating a function for them
JacobianFunc=matlabFunction(dLdx, dLdu,'Vars', {x, u, pvcost}, ...
          'File', 'ExtendableTetherCostJacobiansPT2');

