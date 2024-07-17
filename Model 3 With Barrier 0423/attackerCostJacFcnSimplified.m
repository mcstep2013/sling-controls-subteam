function [Gx, Gmv] = attackerCostJacFcnSimplified(stage, x, u, pvcost)
% twolinkCostJacFcn
%   [Gx, Gmv] = twolinkCostJacFcn(stage, x, u, pvcost)
%
%   This function computes the Jacobian matrices Gx and Gmv for the cost
%   function of a two-link robot manipulator.
% 
%   Inputs:
%       - stage     Current stage of the Multistage MPC problem
%       - x         Current state vector
%       - u         Control action vector at the previous interval
%       - pvstate   Vector containing various stage parameters
%
%   Output:
%       - Gx        Jacobian matrix of the cost with respect to the x
%       - Gmv       Jacobian matrix of the cost with respect to the u

% Copyright 2023 The MathWorks, Inc.

% Extract Desired State
xt = pvcost(1:2);

% Extract and Initialize Wiegthing Matrices
Sf = diag(pvcost(3:4));                 % Terminal Weight Matrix
Q  = diag(pvcost(5:6));                % State Weight Matrix
R  = diag(pvcost(7:8));    % Control Weight Matrix
p  = pvcost(9);                        % Prediction Horizon

if stage == p + 1
    % Calculate Jacobian matrices for the terminal stage
    Gx  = Sf*(x - xt);
    Gmv = zeros(length(u), 1);
else
    % Calculate Jacobian matrices for intermediate stages
    Gx  = Q*(x - xt);
    Gmv = R*u;
end
end